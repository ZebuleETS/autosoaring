import csv
import asyncio
import time
import math
import matplotlib.pyplot as plt
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
import signal
from collections import deque
import threading
import json
import yaml
from std_msgs.msg import String, Float32MultiArray
import os  # For file saving
from mavsdk.action import OrbitYawBehavior

# --- ROS2 Imports ---
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from example_interfaces.srv import Trigger

# --- Global storage ---
telemetry_data = []  # (time, altitude, climb_rate, v_energy, throttle, airspeed)
thermal_events = []
thermal_detection_complete = False  # Track if we have Enter → Core → End cycle
config_data = None  # Global config data

# Thermal exploitation variables
exploiting_thermal = False
thermal_center_lat = None
thermal_center_lon = None
thermal_center_alt = None
orbit_radius = 40.0  # meters (default, will be overridden by config)
orbit_altitude = None
original_mission = None
mission_paused = False
target_altitude = 600.0  # Target altitude for thermal exploitation (default, will be overridden by config)
exploited_thermals = set()  # Track exploited thermal locations to avoid re-exploitation


circle_radius = 40.0  # meters (use same as orbit)
circle_samples = []  # (lat, lon, climb_rate, bearing_deg, timestamp)
circle_center_lat = None
circle_center_lon = None
circle_completed = False
centering_active = False  # Whether centering is currently active
centering_iteration = 0
max_centering_iterations = 10  # Prevent infinite loops (default, will be overridden by config)
CLIMB_RATE_DIFF_THRESHOLD = 0.5  # Stop centering if max climb rate difference is less than 0.5 m/s (default, will be overridden by config)

# Internal tracking for circle completion
_circle_start_bearing = None
_circle_total_angle = 0.0
_circle_last_bearing = None

# Enhanced centering parameters
negative_circle_count = 0  # Count consecutive circles with all negative climb rates
previous_circle_performance = None  # Store previous circle's max and average climb rates
previous_center_lat = None  # Store previous center for reverting
previous_center_lon = None

# Glide mode monitoring tasks
glide_monitoring_task = None
altitude_monitoring_task = None

# Glide mode state variables
glide_mode_active = False
altitude_monitor_active = False
glide_start_altitude = None
glide_start_time = None

# ROS2 node globals
ros_node = None
alt_pub = None
airspeed_pub = None
throttle_pub = None
pos_pub = None
position_pub = None  # For publishing UAV position (lat, lon, alt)
battery_service = None

# ---------- Logging utilities ----------
def save_log():
    # Create flights_data directory if it doesn't exist
    flights_data_dir = "flights_data"
    if not os.path.exists(flights_data_dir):
        os.makedirs(flights_data_dir)
        print(f" Created directory: {flights_data_dir}")
    
    filename = os.path.join(flights_data_dir, "telemetry_log_detection.csv")
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Time (s)", "Altitude (m)", "Climb Rate (m/s)", "V_Energy (m/s)", "Throttle (%)", "Airspeed (m/s)"])
        writer.writerows(telemetry_data)
    print(f"Telemetry log saved to {filename}")

def plot_telemetry_data():
    if not telemetry_data:
        print("No telemetry data recorded. Skipping plot.")
        return

    times, alts, climbs, energies, throttles, airspeeds = zip(*telemetry_data)
    plt.figure(figsize=(12, 10))
    labels = ["Altitude (m)", "Climb Rate (m/s)", "V_Energy (m/s)", "Throttle (%)", "Airspeed (m/s)"]
    data = [alts, climbs, energies, throttles, airspeeds]

    for i, (label, y) in enumerate(zip(labels, data), start=1):
        plt.subplot(5, 1, i)
        plt.plot(times, y)
        for event_time, _, _, event_type in thermal_events:
            color = 'green' if event_type == 'start' else 'red'
            plt.axvline(x=event_time, linestyle='--', color=color, alpha=0.7)
        plt.ylabel(label)
        plt.grid()

    plt.xlabel("Time (s)")
    plt.tight_layout()

    # Create flights_data directory if it doesn't exist
    flights_data_dir = "flights_data"
    if not os.path.exists(flights_data_dir):
        os.makedirs(flights_data_dir)
        print(f" Created directory: {flights_data_dir}")
    
    plot_filename = os.path.join(flights_data_dir, "telemetry_plot.png")
    plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
    print(f"Telemetry plot saved to {os.path.abspath(plot_filename)}")
    plt.close()

 
    print(f"Telemetry plot saved to {plot_filename}")
    print("To view the plot, open the file manually in your image viewer")

def handle_exit(signum, frame):
    print("Simulation interrupted! Saving and plotting...")
    save_log()
    plot_telemetry_data()
    time.sleep(1.0)
    print("Telemetry analysis complete. Check the generated files:")
    print("- flights_data/telemetry_log_detection.csv: Raw telemetry data")
    print("- flights_data/telemetry_plot.png: Visual telemetry analysis")
    rclpy.shutdown()
    exit(0)

signal.signal(signal.SIGINT, handle_exit)

# ---------- Geometry helpers ----------
def _norm_angle_deg(a):
    a = a % 360.0
    return a if a >= 0.0 else a + 360.0

def _shortest_signed_delta_deg(a_from, a_to):
    return (a_to - a_from + 540.0) % 360.0 - 180.0

def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371000.0
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def calculate_bearing(lat1, lon1, lat2, lon2):
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    y = math.sin(dlon) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
    return _norm_angle_deg(math.degrees(math.atan2(y, x)))

def calculate_arc_center(lat, lon, bearing, distance):
    R = 6371000.0
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    bearing_rad = math.radians(bearing)
    ad = distance / R
    new_lat_rad = math.asin(math.sin(lat_rad) * math.cos(ad) + math.cos(lat_rad) * math.sin(ad) * math.cos(bearing_rad))
    new_lon_rad = lon_rad + math.atan2(math.sin(bearing_rad) * math.sin(ad) * math.cos(lat_rad),
                                       math.cos(ad) - math.sin(lat_rad) * math.sin(new_lat_rad))
    return math.degrees(new_lat_rad), math.degrees(new_lon_rad)


# ---------- Centering control ----------
def start_thermal_centering(center_lat, center_lon):
    global circle_center_lat, circle_center_lon, circle_samples
    global _circle_start_bearing, _circle_total_angle, _circle_last_bearing
    global circle_completed, centering_active, centering_iteration
    global negative_circle_count, previous_circle_performance, previous_center_lat, previous_center_lon

    circle_center_lat = center_lat
    circle_center_lon = center_lon
    circle_samples = []
    _circle_start_bearing = None
    _circle_total_angle = 0.0
    _circle_last_bearing = None
    circle_completed = False
    centering_active = True
    centering_iteration = 0
    
    # Initialize enhanced centering variables
    negative_circle_count = 0
    previous_circle_performance = None
    previous_center_lat = center_lat
    previous_center_lon = center_lon

    print(f" STARTING THERMAL CENTERING at {center_lat:.6f}, {center_lon:.6f}; radius={circle_radius} m")
    print("   Enhanced Rule: Mixed/negative → halfway movement; All positive → 1/3 movement + performance comparison")
    print("   Exit after 2 consecutive circles with all negative climb rates")

def add_centering_sample(lat, lon, climb_rate, node):
    """
    Add a sample during centering. Returns True when a circle is completed.
    Completion: accumulated absolute angle change >= 300° and >= 12 samples.
    """
    global circle_samples, _circle_start_bearing, _circle_total_angle, _circle_last_bearing, circle_completed
    if circle_center_lat is None or circle_center_lon is None:
        return False

    brg = calculate_bearing(circle_center_lat, circle_center_lon, lat, lon)
    now = time.time()

    if _circle_start_bearing is None:
        _circle_start_bearing = brg
        _circle_last_bearing = brg
        print(" Centering circle started")

    d = _shortest_signed_delta_deg(_circle_last_bearing, brg)
    _circle_total_angle += abs(d)
    _circle_last_bearing = brg

    circle_samples.append((lat, lon, climb_rate, brg, now))

    # Show progress every 90 degrees
    if len(circle_samples) % 4 == 0:
        node.get_logger().info(f"CENTERING PROGRESS: {len(circle_samples)} samples, {_circle_total_angle:.1f}° / 300°")

    if not circle_completed and _circle_total_angle >= 300.0 and len(circle_samples) >= 12:
        circle_completed = True
        node.get_logger().info(f" Circle completed with {len(circle_samples)} samples; total angle ≈ {_circle_total_angle:.1f}°")
        return True

    return False

def analyze_circle_and_center(node):
    """
    ENHANCED LOGIC: 
    1. If ALL climb rates negative → increment counter, exit after 2 circles
    2. If mixed positive/negative → reset counter, move halfway (original logic)
    3. If ALL positive → reset counter, move 1/3 distance + performance comparison
    Returns: (new_lat, new_lon, should_continue, should_exit_thermal)
    """
    global circle_samples, circle_center_lat, circle_center_lon, centering_iteration
    global negative_circle_count, previous_circle_performance, previous_center_lat, previous_center_lon

    if not circle_completed or len(circle_samples) < 8:
        return None, None, False, False

    # Extract climb rates from samples
    climb_rates = [sample[2] for sample in circle_samples]
    max_climb_rate = max(climb_rates)
    min_climb_rate = min(climb_rates)
    avg_climb_rate = sum(climb_rates) / len(climb_rates)
    
    # Check climb rate distribution
    all_negative = all(rate < 0 for rate in climb_rates)
    all_positive = all(rate > 0 for rate in climb_rates)
    mixed = not all_negative and not all_positive
    
    node.get_logger().info(f" Circle analysis: max={max_climb_rate:.2f}, avg={avg_climb_rate:.2f} m/s, distribution: {'all_negative' if all_negative else 'all_positive' if all_positive else 'mixed'}")
    
    # 1. If ALL climb rates are negative
    if all_negative:
        negative_circle_count += 1
        node.get_logger().warn(f" ALL NEGATIVE CLIMB RATES: Circle {negative_circle_count}/2")
        
        if negative_circle_count >= 2:
            node.get_logger().warn(" EXITING THERMAL: 2 consecutive circles with all negative climb rates")
            return None, None, False, True  # Exit thermal
        else:
            node.get_logger().info(f" Continuing centering (negative circle {negative_circle_count}/2)")
            return None, None, True, False  # Continue centering
    
    # Reset negative counter if we found any positive air
    if negative_circle_count > 0:
        negative_circle_count = 0
    
    # 2. If mixed positive/negative (original logic)
    if mixed:
        node.get_logger().info(" MIXED CLIMB RATES: Using original logic (halfway movement)")
        
        # Check if the difference between any point and the maximum is less than threshold
        max_diff = 0.0
        for sample in circle_samples:
            diff = max_climb_rate - sample[2]
            max_diff = max(max_diff, diff)
        
        if max_diff < CLIMB_RATE_DIFF_THRESHOLD:
            node.get_logger().info(f" Center found: max climb rate difference ({max_diff:.2f} m/s) < threshold ({CLIMB_RATE_DIFF_THRESHOLD} m/s)")
            return None, None, False, False

        # Highest climb sample
        max_sample = max(circle_samples, key=lambda s: s[2])
        max_lat, max_lon, max_climb, max_brg, _ = max_sample

        # Move halfway toward the max-climb point
        d = calculate_distance(circle_center_lat, circle_center_lon, max_lat, max_lon)
        half = d * 0.5
        new_lat, new_lon = calculate_arc_center(circle_center_lat, circle_center_lon, max_brg, half)

        centering_iteration += 1
        node.get_logger().info(
            f" Iter {centering_iteration}: max climb={max_climb:.2f} m/s at {max_lat:.6f},{max_lon:.6f}; "
            f"max diff={max_diff:.2f} m/s; shift center by {half:.1f} m to {new_lat:.6f},{new_lon:.6f}"
        )

        if centering_iteration >= max_centering_iterations:
            node.get_logger().warn(" Max centering iterations reached; stopping centering here.")
            return None, None, False, False

        return new_lat, new_lon, True, False
    
    if all_positive:
        node.get_logger().info(" ALL POSITIVE CLIMB RATES: Using enhanced logic (1/3 movement + performance comparison)")
        
        # Store current performance
        current_performance = {
            'max_climb_rate': max_climb_rate,
            'avg_climb_rate': avg_climb_rate,
            'center_lat': circle_center_lat,
            'center_lon': circle_center_lon
        }
        
        # Performance comparison with previous circle
        if previous_circle_performance is not None:
            prev_max = previous_circle_performance['max_climb_rate']
            prev_avg = previous_circle_performance['avg_climb_rate']
            
            # If current performance is worse, revert to previous center
            if max_climb_rate < prev_max or avg_climb_rate < prev_avg:
                node.get_logger().warn(f" PERFORMANCE DEGRADED: Reverting to previous center (prev_max={prev_max:.2f}, curr_max={max_climb_rate:.2f})")
                return previous_center_lat, previous_center_lon, False, False
        
        # Update previous performance and center
        previous_circle_performance = current_performance
        previous_center_lat = circle_center_lat
        previous_center_lon = circle_center_lon
        
        # Find highest climb sample
        max_sample = max(circle_samples, key=lambda s: s[2])
        max_lat, max_lon, max_climb, max_brg, _ = max_sample

        # Move 1/3 distance toward the max-climb point
        d = calculate_distance(circle_center_lat, circle_center_lon, max_lat, max_lon)
        third = d * (1.0/3.0)
        new_lat, new_lon = calculate_arc_center(circle_center_lat, circle_center_lon, max_brg, third)

        centering_iteration += 1
        node.get_logger().info(
            f" Iter {centering_iteration}: max climb={max_climb:.2f} m/s at {max_lat:.6f},{max_lon:.6f}; "
            f"shift center by {third:.1f} m (1/3 distance) to {new_lat:.6f},{new_lon:.6f}"
        )

        if centering_iteration >= max_centering_iterations:
            node.get_logger().warn(" Max centering iterations reached; stopping centering here.")
            return None, None, False, False

        return new_lat, new_lon, True, False
    
    # Fallback (should not reach here)
    node.get_logger().warn(" Unexpected climb rate distribution - using original logic")
    return None, None, False, False

def start_new_centering_circle(center_lat, center_lon, node):
    global circle_center_lat, circle_center_lon, circle_samples
    global _circle_start_bearing, _circle_total_angle, _circle_last_bearing, circle_completed
    circle_center_lat = center_lat
    circle_center_lon = center_lon
    circle_samples = []
    _circle_start_bearing = None
    _circle_total_angle = 0.0
    _circle_last_bearing = None
    circle_completed = False
    node.get_logger().info(f" New centering circle at {center_lat:.6f},{center_lon:.6f}")

# ---------- Exploitation control ----------
async def start_thermal_exploitation(drone, lat, lon, alt, node):
    """Start thermal exploitation by orbiting around the thermal core; disables detection during centering."""
    global exploiting_thermal, thermal_center_lat, thermal_center_lon, thermal_center_alt, orbit_altitude, exploited_thermals
    global glide_mode_active, altitude_monitor_active  

    # Skip if already exploited nearby (~100 m precision)
    thermal_key = f"{lat:.3f},{lon:.3f}"
    if thermal_key in exploited_thermals:
        node.get_logger().info(f" SKIPPING THERMAL - Already exploited near {thermal_key}")
        return False

    # RESET GLIDE MODE if it was active (new thermal detected)
    if glide_mode_active or altitude_monitor_active:
        node.get_logger().info(" RESETTING GLIDE MODE: New thermal detected during glide")
        glide_mode_active = False
        altitude_monitor_active = False
        glide_start_altitude = None
        glide_start_time = None
        node.get_logger().info(" GLIDE MODE RESET COMPLETED")

    node.get_logger().info(f" STARTING THERMAL EXPLOITATION at core lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}m")
    exploited_thermals.add(thermal_key)
    exploiting_thermal = True
    thermal_center_lat = lat
    thermal_center_lon = lon
    thermal_center_alt = alt
    orbit_altitude = alt

    try:
        # Step 1: Set PX4 parameters FIRST (before orbit to avoid mode switching during orbit)
        await drone.param.set_param_int("THRM_MODE_EN", 1)
        node.get_logger().info("    Enabled PX4 thermal soaring (THRM_MODE_EN=1)")
        
        # Ensure manual throttle is OFF during thermal exploitation
        await drone.param.set_param_int("GL_MODE_EN", 0)
        node.get_logger().info("    ENSURED MANUAL THROTTLE OFF: GL_MODE_EN=0 (for thermal exploitation)")
        
        # Step 3: Pause mission to allow orbit mode
        await drone.mission.pause_mission()
        node.get_logger().info("    Paused mission to allow orbit mode")
        
        # Step 4: Start orbit for centering (orbit mode = loiter mode in PX4)
        await drone.action.do_orbit(
            radius_m=40.0,
            velocity_ms=10.0,
            yaw_behavior=OrbitYawBehavior.UNCONTROLLED,
            latitude_deg=lat,
            longitude_deg=lon,
            absolute_altitude_m=alt
        )
        node.get_logger().info("    Started orbiting around thermal center")
        
        # Ensure manual throttle is OFF during thermal exploitation
        await drone.param.set_param_int("GL_MODE_EN", 0)
        node.get_logger().info("    ENSURED MANUAL THROTTLE OFF: GL_MODE_EN=0 (for thermal exploitation)")
        
    except Exception as e:
        node.get_logger().warn(f"    Could not start orbit: {e}")
        try:
            await drone.action.goto_location(lat, lon, alt, 0)
            node.get_logger().info("    Fallback: Goto thermal center")
        except Exception as e2:
            node.get_logger().error(f"    Fallback failed: {e2}")

    # Begin centering (this implicitly disables detection because detect_thermal gates on not centering)
    start_thermal_centering(lat, lon)
    return True

async def stop_thermal_exploitation(drone, node):
    """Stop thermal exploitation and start glide mode."""
    global exploiting_thermal, mission_paused, thermal_detection_complete
    global thermal_center_lat, thermal_center_lon, thermal_center_alt, orbit_altitude
    global centering_active, circle_samples, circle_completed

    node.get_logger().info(" STOPPING THERMAL EXPLOITATION")
    exploiting_thermal = False
    centering_active = False

    thermal_center_lat = None
    thermal_center_lon = None
    thermal_center_alt = None
    orbit_altitude = None
    circle_samples = []
    circle_completed = False

    # Allow new detections later
    thermal_detection_complete = False

    try:
        await drone.param.set_param_int("THRM_MODE_EN", 0)
        node.get_logger().info("    Disabled PX4 thermal soaring (THRM_MODE_EN=0)")
        
        try:
            await drone.mission.start_mission()
            node.get_logger().info("    MISSION MODE: Started to exit orbit mode")
        except Exception as e2:
            node.get_logger().warn(f"    Could not start mission mode: {e2}")
        
        # Step 3: Set manual throttle for gliding
        await drone.param.set_param_int("GL_MODE_EN", 1)
        node.get_logger().info("     MANUAL THROTTLE: GL_MODE_EN=1 (for gliding)")
            
    except Exception as e:
        node.get_logger().warn(f"    Could not reset PX4 parameters: {e}")

    # SIMPLIFIED SEQUENCE: THRM_MODE_EN=0 → MISSION mode → GL_MODE_EN=1
    # Since orbit mode = loiter mode, no special handling needed

async def monitor_thermal_exploitation(drone, node):
    """Monitor thermal exploitation: one circle → recenter → repeat; stop at target altitude or weakening."""
    global exploiting_thermal, orbit_altitude, thermal_center_lat, thermal_center_lon, centering_active

    if not exploiting_thermal:
        return

    try:
        position = await drone.telemetry.position().__anext__()
        current_alt = position.absolute_altitude_m
        current_lat = position.latitude_deg
        current_lon = position.longitude_deg

        # Climb rate estimate from last sample
        now = time.time()
        if hasattr(monitor_thermal_exploitation, 'last_alt'):
            dt = max(1e-3, now - getattr(monitor_thermal_exploitation, 'last_time', now))
            climb_rate = (current_alt - monitor_thermal_exploitation.last_alt) / dt
        else:
            climb_rate = 0.0
        monitor_thermal_exploitation.last_alt = current_alt
        monitor_thermal_exploitation.last_time = now

        # Stop conditions
        if current_alt >= target_altitude:
            node.get_logger().info(f"   TARGET ALTITUDE REACHED: {current_alt:.1f}m >= {target_altitude}m")
            node.get_logger().info("   TRANSITIONING DIRECTLY TO GLIDE MODE...")
            
            # CORRECT SEQUENCE: Stop thermal exploitation FIRST, then configure glide mode
            # Step 1: Stop thermal exploitation (THRM_MODE_EN=0 → MISSION mode)
            await stop_thermal_exploitation(drone, node)
            node.get_logger().info("   Step 1:  THERMAL EXPLOITATION STOPPED")
            
            # Step 2: Set manual throttle for gliding (after mission mode is active)
            try:
                await drone.param.set_param_int("GL_MODE_EN", 1)
                node.get_logger().info("   Step 2:  MANUAL THROTTLE: GL_MODE_EN=1 (for gliding)")
                
                # Step 3: Start glide mode monitoring
                node.get_logger().info("   Step 3: STARTING GLIDE MODE MONITORING...")
                await start_glide_mode(drone, node)
                node.get_logger().info("   Step 3:  GLIDE MODE MONITORING STARTED SUCCESSFULLY")
                
            except Exception as e:
                node.get_logger().error(f"   Could not configure glide mode: {e}")
            
            return
        
        # Debug: Show altitude progress every 10m
        if hasattr(monitor_thermal_exploitation, 'last_alt_debug'):
            if current_alt - monitor_thermal_exploitation.last_alt_debug >= 10.0:
                node.get_logger().info(f" ALTITUDE PROGRESS: {current_alt:.1f}m / {target_altitude}m")
                monitor_thermal_exploitation.last_alt_debug = current_alt
        else:
            monitor_thermal_exploitation.last_alt_debug = current_alt

        if current_alt > (orbit_altitude + 10.0):
            orbit_altitude = current_alt
            # Update orbit altitude (orbit mode = loiter mode, so this works)
            try:
                await drone.action.do_orbit(
                    radius_m=40.0,
                    velocity_ms=10.0,
                    yaw_behavior=OrbitYawBehavior.UNCONTROLLED,
                    latitude_deg=thermal_center_lat,
                    longitude_deg=thermal_center_lon,
                    absolute_altitude_m=current_alt
                )
                node.get_logger().info(f"    Updated orbit altitude to {current_alt:.1f}m")
            except Exception as e:
                node.get_logger().warn(f"    Could not update orbit altitude: {e}")
        elif current_alt < (orbit_altitude - 30.0):
            node.get_logger().warn(f"    Thermal weakening - altitude: {current_alt:.1f}m")
            await stop_thermal_exploitation(drone, node)
            return

        # Centering step: exactly one circle per iteration
        if centering_active:
            if add_centering_sample(current_lat, current_lon, climb_rate, node):
                node.get_logger().info("    ANALYZING CIRCLE AND CENTERING...")
                new_lat, new_lon, should_continue, should_exit_thermal = analyze_circle_and_center(node)
                
                # Check if we should exit the thermal
                if should_exit_thermal:
                    node.get_logger().warn("    EXITING THERMAL: Poor performance detected")
                    await stop_thermal_exploitation(drone, node)
                    return
                
                if should_continue and new_lat is not None and new_lon is not None:
                    thermal_center_lat, thermal_center_lon = new_lat, new_lon
                    node.get_logger().info(f" MOVING CENTER: lat={new_lat:.6f}, lon={new_lon:.6f}")
                    
                    # Update orbit center (orbit mode = loiter mode, so this works)
                    try:
                        await drone.action.do_orbit(
                            radius_m=40.0,
                            velocity_ms=10.0,
                            yaw_behavior=OrbitYawBehavior.UNCONTROLLED,
                            latitude_deg=new_lat,
                            longitude_deg=new_lon,
                            absolute_altitude_m=current_alt
                        )
                        node.get_logger().info("    Orbit updated to new center")
                    except Exception as e:
                        node.get_logger().warn(f"    Could not update orbit: {e}")
                        try:
                            await drone.action.goto_location(new_lat, new_lon, current_alt, 0)
                            node.get_logger().info("   Fallback: Goto new center")
                        except Exception as e2:
                            node.get_logger().error(f"    Fallback failed: {e2}")
                    start_new_centering_circle(new_lat, new_lon, node)
                else:
                    # Centering complete → continue loitering until target altitude
                    centering_active = False
                    node.get_logger().info("    THERMAL CENTERING COMPLETE! Continuing loiter until target altitude.")

    except Exception as e:
        node.get_logger().error(f"    Error monitoring thermal exploitation: {e}")

async def monitor_thermal_exploitation_loop(drone, node):
    """Continuous monitoring loop for thermal exploitation."""
    while True:
        await monitor_thermal_exploitation(drone, node)
        await asyncio.sleep(1.0)

# ---------- ROS2 spin in a separate thread ----------
def ros_spin_thread(node):
    rclpy.spin(node)

# ---------- Load waypoints from .plan file ----------
def load_plan_waypoints(plan_file):
    """
    Load waypoints from a QGroundControl .plan file (handles SimpleItem and ComplexItem).
    Returns a list of (latitude, longitude, altitude).
    """
    with open(plan_file, 'r') as f:
        plan = json.load(f)

    waypoints = []

    def extract_from_items(items):
        for item in items:
            cmd = item.get("command")
            if cmd == 16 or cmd == 22:  # Waypoint or Takeoff
                params = item.get("params", [])
                if len(params) >= 7 and params[4] and params[5]:
                    lat = float(params[4])
                    lon = float(params[5])
                    alt = float(params[6]) if params[6] else 50.0
                    waypoints.append((lat, lon, alt))
            # Handle ComplexItem transects
            if "TransectStyleComplexItem" in item:
                extract_from_items(item["TransectStyleComplexItem"].get("Items", []))

    items = plan.get("mission", {}).get("items", [])
    extract_from_items(items)
    return waypoints

# ---------- Mission setup ----------
async def waypoints_mission(drone, plan_file):
    mission_items = []
    speed = 5

    waypoints = load_plan_waypoints(plan_file)
    print(f"Loaded {len(waypoints)} waypoints from {plan_file}:")
    for i, (lat, lon, alt) in enumerate(waypoints):
        print(f"  WP{i}: lat={lat}, lon={lon}, alt={alt}")

    if not waypoints:
        raise RuntimeError("No valid waypoints found in plan file!")

    for lat, lon, alt in waypoints:
        mission_items.append(MissionItem(
            lat, lon, alt,
            speed, True,
            float('nan'), float('nan'),
            MissionItem.CameraAction.NONE,
            float('nan'), float('nan'), float('nan'), float('nan'), float('nan'),
            MissionItem.VehicleAction.NONE
        ))

    lat, lon, _ = waypoints[-1]
    mission_items.append(MissionItem(
        lat, lon, 0,
        speed, True,
        float('nan'), float('nan'),
        MissionItem.CameraAction.NONE,
        float('nan'), float('nan'), float('nan'), float('nan'), float('nan'),
        MissionItem.VehicleAction.LAND
    ))

    await drone.mission.clear_mission()
    await drone.mission.upload_mission(MissionPlan(mission_items))
    print(f"-- Mission uploaded with {len(mission_items)} items from {plan_file}")
    await drone.action.set_current_speed(speed)

async def monitor_mission_progress(drone, node):
    """Monitor mission progress and handle thermal exploitation completion"""
    global exploiting_thermal, target_altitude, current_mission_index
    
    while True:
        try:
            # Check if we're in thermal exploitation and have reached target altitude
            # NOTE: This check is now handled by monitor_thermal_exploitation function
            # to avoid conflicts and ensure proper transition sequence
            
            # Check mission progress
            async for prog in drone.mission.mission_progress():
                node.get_logger().info(f"Mission progress: {prog.current}/{prog.total}")
                
                # Update current mission index for glide monitoring
                if prog.current > current_mission_index:
                    current_mission_index = prog.current - 1
                    node.get_logger().info(f"📍 Advanced to waypoint {current_mission_index}")
                
                if prog.current == prog.total:
                    node.get_logger().info(" Final waypoint reached.")
                    return
                await asyncio.sleep(1)
                
        except Exception as e:
            node.get_logger().error(f" Error in mission monitoring: {e}")
            await asyncio.sleep(1)

# ---------- Realtime telemetry readers ----------
latest_pos = None
latest_vel = None
latest_met = None
async def pos_reader(pos_stream):
    global latest_pos
    async for pos in pos_stream:
        latest_pos = pos

async def vel_reader(vel_stream):
    global latest_vel
    async for vel in vel_stream:
        latest_vel = vel

async def met_reader(met_stream):
    global latest_met
    async for met in met_stream:
        latest_met = met

# ---------- Thermal detection loop (trend-based) ----------
async def detect_thermal(drone, alt_pub, airspeed_pub, throttle_pub, detection_pub, pos_pub, node):
    global thermal_events, start_time, thermal_detection_complete, centering_active, exploiting_thermal
    global glide_mode_active, altitude_monitor_active  # Add glide mode globals

    current_thermal_points = []
    thermal_detection_complete = False

    print("Thermal detection running (trend-based)...")

    g = 9.81
    smoothing_alpha = 0.1
    TREND_WINDOW_SEC = 4.0
    DETECTION_WINDOW = 5
    DETECTION_RATIO = 0.5

    # Thresholds
    """v_energy_min = 0.2
    climb_min = 0.2
    throttle_max = 0.6
    slope_threshold = 0.08"""
    
    v_energy_min = 0.7      # 5x increase (0.2 → 1.0)
    climb_min = 0.7         # 5x increase (0.2 → 1.0) 
    throttle_max = 0.2     # Much more restrictive (0.6 → 0.2)
    slope_threshold = 0.2
    
    in_thermal = False
    detection_window = deque(maxlen=DETECTION_WINDOW)
    trend_history = deque()

    prev_time = None
    smoothed_alt = None
    smoothed_airspeed = None
    prev_smoothed_alt = None
    prev_smoothed_airspeed = None

    print("Waiting for initial data...")
    while prev_time is None:
        pos = await drone.telemetry.position().__anext__()
        vel = await drone.telemetry.velocity_ned().__anext__()
        met = await drone.telemetry.fixedwing_metrics().__anext__()
        gs = math.hypot(vel.north_m_s, vel.east_m_s)
        airspeed = met.airspeed_m_s if met.airspeed_m_s > 0.1 else gs
        if airspeed > 0.1:
            smoothed_alt = prev_smoothed_alt = pos.relative_altitude_m
            smoothed_airspeed = prev_smoothed_airspeed = airspeed
            prev_time = asyncio.get_event_loop().time()

    if 'start_time' not in globals():
        start_time = prev_time

    print("Started detecting thermals.")
    while True:
        pos = await drone.telemetry.position().__anext__()
        vel = await drone.telemetry.velocity_ned().__anext__()
        met = await drone.telemetry.fixedwing_metrics().__anext__()

        t = asyncio.get_event_loop().time()
        dt = t - prev_time
        if dt <= 0:
            await asyncio.sleep(0.05)
            continue

        raw_alt = pos.relative_altitude_m
        gs = math.hypot(vel.north_m_s, vel.east_m_s)
        raw_airspeed = met.airspeed_m_s if met.airspeed_m_s > 0.1 else gs

        smoothed_alt = smoothing_alpha * raw_alt + (1 - smoothing_alpha) * smoothed_alt
        smoothed_airspeed = smoothing_alpha * raw_airspeed + (1 - smoothing_alpha) * smoothed_airspeed

        climb_rate = (smoothed_alt - prev_smoothed_alt) / dt
        airspeed_dot = (smoothed_airspeed - prev_smoothed_airspeed) / dt
        v_energy = climb_rate + (smoothed_airspeed * airspeed_dot) / g

        # Publish telemetry to ROS2
        alt_pub.publish(Float32(data=raw_alt))
        airspeed_pub.publish(Float32(data=smoothed_airspeed))
        throttle_pub.publish(Float32(data=met.throttle_percentage))

        # Publish UAV position (lat, lon, alt)
        pos_msg = Float32MultiArray()
        pos_msg.data = [pos.latitude_deg, pos.longitude_deg, raw_alt]
        pos_pub.publish(pos_msg)

        telemetry_data.append(
            (t - start_time, raw_alt, climb_rate, v_energy, met.throttle_percentage, smoothed_airspeed)
        )

        # Trend tracking
        trend_history.append((t, climb_rate, met.throttle_percentage, v_energy))
        while trend_history and (t - trend_history[0][0]) > TREND_WINDOW_SEC:
            trend_history.popleft()

        climb_slope = v_energy_slope = throttle_slope = 0
        if len(trend_history) >= 2:
            t0, climb0, thr0, ve0 = trend_history[0]
            t1, climb1, thr1, ve1 = trend_history[-1]
            dt_trend = max(t1 - t0, 1e-3)
            climb_slope = (climb1 - climb0) / dt_trend
            v_energy_slope = (ve1 - ve0) / dt_trend
            throttle_slope = (thr1 - thr0) / dt_trend

        is_trend_positive = (climb_slope > slope_threshold) or (v_energy_slope > slope_threshold)
        is_thermal_now = (
            climb_rate > climb_min and
            v_energy > v_energy_min and
            met.throttle_percentage < throttle_max and
            throttle_slope <= slope_threshold and
            is_trend_positive
        )

        detection_window.append(is_thermal_now)
        detected_ratio = sum(detection_window) / len(detection_window)

        if in_thermal:
            current_thermal_points.append((met.throttle_percentage, pos.latitude_deg, pos.longitude_deg))

        # Check if in high altitude zone (simple altitude-based detection control)
        # Use a fixed high altitude threshold (e.g., 600m) instead of undefined target_altitude
        high_altitude_threshold = 600.0  # meters
        in_high_altitude_zone = raw_alt > high_altitude_threshold
        if in_high_altitude_zone and (not hasattr(detect_thermal, 'last_high_alt_log') or (time.time() - getattr(detect_thermal, 'last_high_alt_log', 0)) > 10):
            node.get_logger().info(f" HIGH ALTITUDE ZONE: {raw_alt:.1f}m > {high_altitude_threshold:.1f}m - Thermal detection disabled")
            detect_thermal.last_high_alt_log = time.time()
        
        # Check if in gliding zone (above target altitude - 100m) - no thermal detection during gliding
        #in_gliding_zone = raw_alt > (target_altitude - 100.0)
        in_gliding_zone = raw_alt > target_altitude 

        if in_gliding_zone and (not hasattr(detect_thermal, 'last_gliding_log') or (time.time() - getattr(detect_thermal, 'last_gliding_log', 0)) > 10):
            node.get_logger().info(f" GLIDING ZONE: {raw_alt:.1f}m > {target_altitude - 100.0:.1f}m - Thermal detection disabled (gliding mode)")
            detect_thermal.last_gliding_log = time.time()
        
        # Only detect thermals if not currently exploiting or centering, not already completed this cycle, not in high altitude zone, and not in gliding zone
        if (not exploiting_thermal) and (not thermal_detection_complete) and (not centering_active) and detected_ratio >= DETECTION_RATIO and not in_thermal and not in_high_altitude_zone and not in_gliding_zone:
            detection_time = t - start_time
            thermal_events.append((detection_time, pos.latitude_deg, pos.longitude_deg, 'start'))
            msg = String()
            msg.data = f" Thermal START at t={detection_time:.1f}s, lat={pos.latitude_deg:.6f}, lon={pos.longitude_deg:.6f}"
            detection_pub.publish(msg)
            node.get_logger().info(msg.data)
            current_thermal_points = [(met.throttle_percentage, pos.latitude_deg, pos.longitude_deg)]
            in_thermal = True
            print("    THERMAL DETECTION: Waiting for CORE and END...")

        elif (not exploiting_thermal) and (not centering_active) and detected_ratio < DETECTION_RATIO and in_thermal and not in_high_altitude_zone and not in_gliding_zone:
            detection_time = t - start_time
            thermal_events.append((detection_time, pos.latitude_deg, pos.longitude_deg, 'end'))

            if current_thermal_points:
                core = min(current_thermal_points, key=lambda x: x[0])  # min throttle proxy for strongest core
                core_lat, core_lon, core_thr = core[1], core[2], core[0]
                msg = String()
                msg.data = f" Thermal CORE lat={core_lat:.6f}, lon={core_lon:.6f}, throttle={core_thr:.1f}%"
                detection_pub.publish(msg)
                node.get_logger().info(msg.data)

                # Complete detection cycle and start exploitation at core
                node.get_logger().info("    THERMAL DETECTION COMPLETE: Enter → Core → End")
                node.get_logger().info(f"    STARTING EXPLOITATION at CORE: lat={core_lat:.6f}, lon={core_lon:.6f}")
                thermal_detection_complete = True

                # Start thermal exploitation with centering
                exploitation_started = await start_thermal_exploitation(drone, core_lat, core_lon, raw_alt, node)
                if not exploitation_started:
                    # If skipped (already exploited), re-arm detection
                    thermal_detection_complete = False
                else:
                    node.get_logger().info("    THERMAL EXPLOITATION STARTED - Centering algorithm active!")

            msg = String()
            msg.data = f" Thermal END at t={detection_time:.1f}s, lat={pos.latitude_deg:.6f}, lon={pos.longitude_deg:.6f}"
            detection_pub.publish(msg)
            node.get_logger().info(msg.data)

            current_thermal_points = []
            in_thermal = False

        prev_time = t
        prev_smoothed_alt = smoothed_alt
        prev_smoothed_airspeed = smoothed_airspeed
        await asyncio.sleep(0.1)

# ---------- Enhanced Autosoaring Scenario ----------
# Global variables for enhanced scenario
glide_mode_active = False
glide_start_altitude = None
glide_start_time = None
next_waypoint_altitude = None
mission_waypoints = []
current_mission_index = 0
altitude_monitor_active = False  # New variable for parallel altitude monitoring

def load_mission_waypoints(plan_file):
    """Load mission waypoints for altitude monitoring."""
    global mission_waypoints
    waypoints = load_plan_waypoints(plan_file)
    mission_waypoints = waypoints
    return waypoints

def get_next_waypoint_info():
    """Get information about the next waypoint."""
    global current_mission_index, mission_waypoints
    if current_mission_index < len(mission_waypoints):
        lat, lon, alt = mission_waypoints[current_mission_index]
        return lat, lon, alt, current_mission_index
    return None, None, None, None

def advance_to_next_waypoint():
    """Advance to the next waypoint in the mission."""
    global current_mission_index
    current_mission_index += 1

async def start_glide_mode(drone, node):
    """Start glide mode after thermal exploitation."""
    global glide_mode_active, glide_start_altitude, glide_start_time, altitude_monitor_active
    
    node.get_logger().info(" START_GLIDE_MODE: SIMPLIFIED VERSION")
    
    try:
        # SIMPLIFIED: Just set manual throttle and start monitoring
        await drone.param.set_param_int("GL_MODE_EN", 1)
        node.get_logger().info(" SET MANUAL THROTTLE: GL_MODE_EN=1")
        
        # Get current position and altitude
        position = await drone.telemetry.position().__anext__()
        glide_start_altitude = position.absolute_altitude_m
        glide_start_time = time.time()
        glide_mode_active = True
        altitude_monitor_active = True  # Start parallel altitude monitoring
        
        node.get_logger().info(f"    Glide start: {glide_start_altitude:.1f}m altitude")
        
        # Start BOTH monitoring functions in parallel
        node.get_logger().info(" STARTING GLIDE MONITORING")
        glide_task = asyncio.create_task(monitor_glide_mode(drone, node))
        node.get_logger().info(" GLIDE MONITORING TASK CREATED")
        
        parallel_task = asyncio.create_task(parallel_altitude_monitor(drone, node))
        node.get_logger().info(" PARALLEL ALTITUDE MONITOR TASK CREATED")
        
        # Store tasks globally to prevent garbage collection
        global glide_monitoring_task, altitude_monitoring_task
        glide_monitoring_task = glide_task
        altitude_monitoring_task = parallel_task
        
        node.get_logger().info(" GLIDE MODE SETUP COMPLETED - BOTH MONITORS ACTIVE")
        
    except Exception as e:
        node.get_logger().error(f" Failed to start glide mode: {e}")
        raise  # Re-raise the exception so the calling function can catch it

async def monitor_glide_mode(drone, node):
    """Monitor glide mode and check waypoint reachability."""
    global glide_mode_active, glide_start_altitude, glide_start_time
    global current_mission_index, mission_waypoints
    
    node.get_logger().info(" GLIDE MODE MONITORING STARTED")
    node.get_logger().info(f" GLIDE MODE: glide_mode_active={glide_mode_active}")
    
    if not glide_mode_active:
        node.get_logger().error(" GLIDE MODE: glide_mode_active is False, exiting monitoring")
        return
    
    while glide_mode_active:
        try:
            position = await drone.telemetry.position().__anext__()
            current_alt = position.absolute_altitude_m
            current_lat = position.latitude_deg
            current_lon = position.longitude_deg
            
            # Get next waypoint information
            wp_lat, wp_lon, wp_alt, wp_index = get_next_waypoint_info()
            
            if wp_lat is None:
                node.get_logger().info(" No more waypoints - mission complete")
                await end_glide_mode(drone, node)
                break
            
            # Calculate target altitude (waypoint altitude - 150m safety margin)
            target_altitude = wp_alt - 150.0
            
            # Check if we can reach the waypoint (for logging purposes)
            distance_to_wp = calculate_distance(current_lat, current_lon, wp_lat, wp_lon)
            glide_ratio = 18.0  # Conservative glide ratio
            altitude_loss = distance_to_wp / glide_ratio
            required_altitude = wp_alt + altitude_loss
            
            # Log glide status every 30 seconds
            if not hasattr(monitor_glide_mode, 'last_log_time') or (time.time() - getattr(monitor_glide_mode, 'last_log_time', 0)) > 30:
                node.get_logger().info(f" GLIDE STATUS:")
                node.get_logger().info(f"   Current: {current_alt:.1f}m at {current_lat:.6f}, {current_lon:.6f}")
                node.get_logger().info(f"   Target Altitude: 200.0m (switch to automatic throttle)")
                node.get_logger().info(f"   Distance: {distance_to_wp:.1f}m")
                monitor_glide_mode.last_log_time = time.time()
            
            # NOTE: 200m altitude check is now handled by parallel_altitude_monitor function
            # This function continues to monitor other glide conditions
            
            # Check if UAV is still climbing (thermal exploitation not stopped)
            if hasattr(monitor_glide_mode, 'last_alt_check'):
                altitude_gain = current_alt - monitor_glide_mode.last_alt_check
                if altitude_gain > 5.0:  # More than 5m altitude gain
                    node.get_logger().warn(f" THERMAL EXPLOITATION STILL ACTIVE: Gained {altitude_gain:.1f}m altitude")
                    node.get_logger().warn(f"   Forcing THRM_MODE_EN=0 again")
                    try:
                        # Disable thermal soaring parameters first
                        await drone.param.set_param_int("THRM_MODE_EN", 0)
                        node.get_logger().info("    Disabled thermal soaring parameters")
                        
                        # Wait for orbit to stop
                        await asyncio.sleep(1.0)
                        
                        # Then hold
                        await drone.action.hold()
                        node.get_logger().info("    Forced hold mode")
                    except Exception as e:
                        node.get_logger().error(f"    Could not force stop thermal: {e}")
            
            # Check current flight mode during glide
            if not hasattr(monitor_glide_mode, 'last_mode_check') or (time.time() - getattr(monitor_glide_mode, 'last_mode_check', 0)) > 10:
                try:
                    async for flight_mode in drone.telemetry.flight_mode():
                        node.get_logger().info(f"    Glide mode flight mode: {flight_mode}")
                        if "HOLD" in str(flight_mode):
                            node.get_logger().warn(f"    UAV in HOLD mode instead of MISSION mode!")
                            # Wait a bit before trying to force mission mode
                            await asyncio.sleep(2.0)
                            try:
                                await drone.mission.start_mission()
                                node.get_logger().info("    Forced mission mode")
                            except Exception as e:
                                node.get_logger().warn(f"    Could not force mission mode: {e}")
                        break
                except Exception as e:
                    node.get_logger().warn(f"    Could not check flight mode: {e}")
                monitor_glide_mode.last_mode_check = time.time()
            
            monitor_glide_mode.last_alt_check = current_alt
            
            # Check if we're getting too low (emergency landing)
            min_safe_altitude = 100.0  # meters
            if current_alt < min_safe_altitude:
                node.get_logger().warn(f" EMERGENCY: Altitude too low ({current_alt:.1f}m) - initiating landing")
                await emergency_landing(drone, node)
                break
            
            # EMERGENCY: Stop glide monitoring if UAV gets too low (landed)
            if current_alt < 10.0:  # If UAV is below 10m, it has likely landed
                node.get_logger().warn(f" GLIDE MONITOR: UAV TOO LOW ({current_alt:.1f}m) - LIKELY LANDED!")
                node.get_logger().warn(f" GLIDE MONITOR: STOPPING GLIDE MONITORING - UAV ON GROUND")
                
                # Stop both monitoring functions
                glide_mode_active = False
                altitude_monitor_active = False
                
                node.get_logger().info(" GLIDE MONITOR: Monitoring stopped due to low altitude")
                break
            
            # Continue gliding
            await asyncio.sleep(2.0)  # Check every 2 seconds
            
        except Exception as e:
            node.get_logger().error(f" Error in glide monitoring: {e}")
            await asyncio.sleep(5.0)

async def end_glide_mode(drone, node):
    """End glide mode and resume mission."""
    global glide_mode_active, glide_start_altitude, glide_start_time, altitude_monitor_active
    
    try:
        # Set back to automatic throttle mode (mission continues normally)
        await drone.param.set_param_int("GL_MODE_EN", 0)
        node.get_logger().info(" ENDING GLIDE MODE: GL_MODE_EN=0 (automatic throttle - mission continues)")
        
        # Calculate glide performance
        if glide_start_altitude and glide_start_time:
            position = await drone.telemetry.position().__anext__()
            current_alt = position.absolute_altitude_m
            glide_time = time.time() - glide_start_time
            altitude_lost = glide_start_altitude - current_alt
            
            node.get_logger().info(f" GLIDE PERFORMANCE:")
            node.get_logger().info(f"   Duration: {glide_time:.1f}s")
            node.get_logger().info(f"   Altitude lost: {altitude_lost:.1f}m")
            node.get_logger().info(f"   Average sink rate: {altitude_lost/glide_time:.2f} m/s")
        
        # Stop both monitoring functions
        glide_mode_active = False
        altitude_monitor_active = False
        glide_start_altitude = None
        glide_start_time = None
        
        # Cancel monitoring tasks
        global glide_monitoring_task, altitude_monitoring_task
        if glide_monitoring_task and not glide_monitoring_task.done():
            glide_monitoring_task.cancel()
            node.get_logger().info(" GLIDE MONITORING TASK CANCELLED")
        if altitude_monitoring_task and not altitude_monitoring_task.done():
            altitude_monitoring_task.cancel()
            node.get_logger().info(" ALTITUDE MONITORING TASK CANCELLED")
        
        node.get_logger().info(" BOTH MONITORING FUNCTIONS STOPPED")
        
        # Ensure mission mode is active and resume mission
        try:
            # Double-check we're in mission mode
            await drone.mission.start_mission()
            node.get_logger().info(" MISSION MODE CONFIRMED AND RESUMED")
            
            # Verify automatic throttle is set
            param_value = await drone.param.get_param_int("GL_MODE_EN")
            if param_value == 0:
                node.get_logger().info(" AUTOMATIC THROTTLE CONFIRMED: GL_MODE_EN=0")
            else:
                node.get_logger().warn(f" THROTTLE NOT AUTOMATIC: GL_MODE_EN={param_value}")
                
        except Exception as e:
            node.get_logger().warn(f" Could not resume mission: {e}")
        
    except Exception as e:
        node.get_logger().error(f" Failed to end glide mode: {e}")

async def parallel_altitude_monitor(drone, node):
    """Parallel function to monitor altitude during glide mode and switch to automatic throttle at 200m."""
    global altitude_monitor_active, glide_mode_active
    
    node.get_logger().info(" PARALLEL ALTITUDE MONITOR: Started monitoring")
    node.get_logger().info(f" PARALLEL ALTITUDE MONITOR: altitude_monitor_active={altitude_monitor_active}, glide_mode_active={glide_mode_active}")
    
    while altitude_monitor_active and glide_mode_active:
        try:
            # Get current altitude
            position = await drone.telemetry.position().__anext__()
            current_alt = position.absolute_altitude_m
            
            # Log altitude every 30 seconds for debugging
            if not hasattr(parallel_altitude_monitor, 'last_log_time') or (time.time() - getattr(parallel_altitude_monitor, 'last_log_time', 0)) > 30:
                node.get_logger().info(f" ALTITUDE MONITOR: {current_alt:.1f}m (target: 200m)")
                parallel_altitude_monitor.last_log_time = time.time()
            
            # Check if we have reached 200m altitude (more flexible check)
            if current_alt <= 200.0:  # Switch when we reach or go below 200m
                # Set automatic throttle
                await drone.param.set_param_int("GL_MODE_EN", 0)
                node.get_logger().info(f" ALTITUDE SWITCH: {current_alt:.1f}m -> GL_MODE_EN=0 (automatic throttle)")
                
                # Stop both monitoring functions
                altitude_monitor_active = False
                glide_mode_active = False
                break
            
            if current_alt < 10.0:  # If UAV is below 10m, it has likely landed
                # Stop both monitoring functions
                altitude_monitor_active = False
                glide_mode_active = False
                break
            
            # Check every 1 second
            await asyncio.sleep(1.0)
            
        except Exception as e:
            node.get_logger().error(f" ALTITUDE MONITOR ERROR: {e}")
            await asyncio.sleep(2.0)
    
    node.get_logger().info(" PARALLEL ALTITUDE MONITOR: Stopped monitoring")

async def emergency_landing(drone, node):
    """Emergency landing procedure."""
    node.get_logger().error(" EMERGENCY LANDING INITIATED")
    
    try:
        # Step 1: Disable thermal soaring parameters first
        await drone.param.set_param_int("THRM_MODE_EN", 0)
        node.get_logger().info("    Disabled thermal soaring for emergency landing")
        
        # Step 2: Wait for orbit to stop
        await asyncio.sleep(1.0)
        
        # Step 3: Switch to mission mode
        try:
            await drone.mission.start_mission()
            node.get_logger().info("    Mission mode activated for emergency landing")
            await asyncio.sleep(1.0)
        except Exception as e:
            node.get_logger().warn(f"    Could not switch to mission mode: {e}")
        
        # Step 4: Set manual throttle for gliding
        await drone.param.set_param_int("GL_MODE_EN", 1)
        node.get_logger().info("    Set manual throttle for emergency gliding")
        
        # Find nearest safe landing area
        position = await drone.telemetry.position().__anext__()
        current_lat = position.latitude_deg
        current_lon = position.longitude_deg
        
        # For now, land at current position
        await drone.action.land()
        node.get_logger().info(" Emergency landing at current position")
        
    except Exception as e:
        node.get_logger().error(f" Emergency landing failed: {e}")

# ---------- Main ----------
async def run(config_file=None):
    global ros_node, alt_pub, airspeed_pub, throttle_pub, pos_pub, battery_service, original_mission, config_data

    # Init ROS2 node and publishers
    rclpy.init()
    ros_node = rclpy.create_node('thermal_detection_node')
    
    # Load configuration file
    if config_file:
        ros_node.get_logger().info(f"Loading config from: {config_file}")
        try:
            with open(config_file, 'r') as f:
                config_data = yaml.safe_load(f)
            ros_node.get_logger().info("Config loaded successfully")
        except Exception as e:
            ros_node.get_logger().error(f"Failed to load config file: {e}")
            ros_node.get_logger().error("Using default configuration")
            config_data = {}
    else:
        ros_node.get_logger().info("No config file provided, using default configuration")
        config_data = {}
    
    # Load configurable parameters from config
    global target_altitude, orbit_radius, max_centering_iterations, CLIMB_RATE_DIFF_THRESHOLD, circle_radius
    if config_data:
        target_altitude = config_data.get('target_altitude', target_altitude)
        orbit_radius = config_data.get('orbit_radius', orbit_radius)
        max_centering_iterations = config_data.get('max_centering_iterations', max_centering_iterations)
        CLIMB_RATE_DIFF_THRESHOLD = config_data.get('climb_rate_diff_threshold', CLIMB_RATE_DIFF_THRESHOLD)
        circle_radius = orbit_radius  # Keep circle_radius in sync with orbit_radius
        
        # ros_node.get_logger().info(f"Configuration loaded:")
        # ros_node.get_logger().info(f"  Target altitude: {target_altitude}m")
        # ros_node.get_logger().info(f"  Orbit radius: {orbit_radius}m")
        # ros_node.get_logger().info(f"  Max centering iterations: {max_centering_iterations}")
        # ros_node.get_logger().info(f"  Climb rate threshold: {CLIMB_RATE_DIFF_THRESHOLD}m/s")
    
    alt_pub = ros_node.create_publisher(Float32, '/uav/altitude', 10)
    airspeed_pub = ros_node.create_publisher(Float32, '/uav/airspeed', 10)
    throttle_pub = ros_node.create_publisher(Float32, '/uav/throttle', 10)
    detection_pub = ros_node.create_publisher(String, '/thermal_detected', 10)
    pos_pub = ros_node.create_publisher(Float32MultiArray, '/uav/position', 10)

    # Battery service
    battery_service = ros_node.create_service(Trigger, '/start_battery_system',
                                             lambda request, response: Trigger.Response(success=True))

    threading.Thread(target=ros_spin_thread, args=(ros_node,), daemon=True).start()

    # MAVSDK setup
    drone = System()
    await drone.connect(system_address="udp://:14540")
    ros_node.get_logger().info("-- Connecting...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            ros_node.get_logger().info("-- Connected")
            break
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            ros_node.get_logger().info("-- Position OK")
            break

    pos_stream = drone.telemetry.position()
    vel_stream = drone.telemetry.velocity_ned()
    met_stream = drone.telemetry.fixedwing_metrics()

    asyncio.create_task(pos_reader(pos_stream))
    asyncio.create_task(vel_reader(vel_stream))
    asyncio.create_task(met_reader(met_stream))

    # Save a snapshot of the current mission (optional)
    try:
        original_mission = await drone.mission.download_mission()
    except Exception:
        original_mission = None

    ros_node.get_logger().info("Uploading mission from .plan...")
    
    # Get plan file from config or use default
    if config_data and 'qgc_plan_path' in config_data:
        plan_path = config_data['qgc_plan_path']
        # ros_node.get_logger().info(f"Using plan file from config: {plan_path}")
    else:
        plan_path = 'area2.plan'  # Default fallback
        # ros_node.get_logger().info(f"Using default plan file: {plan_path}")
    
    # Resolve plan file path
    if not os.path.isabs(plan_path):
        # If it's a relative path, resolve it relative to the config file directory
        if config_file:
            config_dir = os.path.dirname(config_file)
            plan_file = os.path.join(config_dir, plan_path)
        else:
            # Fallback to package config directory
            pkg_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
            plan_file = os.path.join(pkg_dir, plan_path)
    else:
        plan_file = plan_path
    
    # Check if plan file exists
    if not os.path.exists(plan_file):
        ros_node.get_logger().error(f"Plan file not found: {plan_file}")
        ros_node.get_logger().error(f"Current working directory: {os.getcwd()}")
        ros_node.get_logger().error(f"Script directory: {os.path.dirname(__file__)}")
        if config_file:
            ros_node.get_logger().error(f"Config file directory: {os.path.dirname(config_file)}")
        ros_node.get_logger().error("Available files in config directory:")
        config_dir = os.path.dirname(plan_file) if os.path.dirname(plan_file) else os.path.join(os.path.dirname(__file__), '..', 'config')
        if os.path.exists(config_dir):
            for file in os.listdir(config_dir):
                ros_node.get_logger().error(f"  - {file}")
        else:
            ros_node.get_logger().error(f"Config directory does not exist: {config_dir}")
        return
    
    # ros_node.get_logger().info(f"Using plan file: {plan_file}")
    await waypoints_mission(drone, plan_file)
    
    # Load mission waypoints for enhanced autosoaring scenario
    # ros_node.get_logger().info("Loading mission waypoints for glide analysis...")
    load_mission_waypoints(plan_file)
    # ros_node.get_logger().info(f"Loaded {len(mission_waypoints)} waypoints for altitude monitoring")

    ros_node.get_logger().info("Arming...")
    await drone.action.arm()
    await asyncio.sleep(5)
    ros_node.get_logger().info("Starting mission...")

    ros_node.get_logger().info("Taking off...")
    await drone.action.takeoff()
    await asyncio.sleep(2)
    while not latest_pos or latest_pos.relative_altitude_m < 5:
        await asyncio.sleep(0.5)
    ros_node.get_logger().info("Takeoff complete")

    # Start detection (this is automatically gated off during centering/exploitation)
    task_det = asyncio.create_task(
        detect_thermal(drone, alt_pub, airspeed_pub, throttle_pub, detection_pub, pos_pub, ros_node)
    )

    await drone.mission.start_mission()

    # Start thermal exploitation monitoring
    task_monitor = asyncio.create_task(
        monitor_thermal_exploitation_loop(drone, ros_node)
    )

    # Start mission progress monitoring
    task_mission = asyncio.create_task(
        monitor_mission_progress(drone, ros_node)
    )

    # Wait for mission to complete
    await task_mission

    print("Landing...")
    await drone.action.land()
    await asyncio.sleep(2)

    task_det.cancel()
    task_monitor.cancel()
    await asyncio.sleep(0.5)

    print("Finished. Saving and plotting...")
    save_log()
    plot_telemetry_data()

    ros_node.destroy_node()
    rclpy.shutdown()

def main(args=None):
    import sys
    config_file = None
    
    # Check for config file argument
    if len(sys.argv) > 1:
        config_file = sys.argv[1]
        print(f"Using config file: {config_file}")
    else:
        print("No config file provided, using default configuration")
        print("Usage: ros2 run autosoaring_pkg thermal_detection_node [path_to_config.yaml]")
    
    asyncio.run(run(config_file))

if __name__ == "__main__":
    main()