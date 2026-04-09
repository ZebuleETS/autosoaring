import json
import yaml
import random
import time
import math
import os
import sys
import gz.transport13 as gz

# Protobuf message (copié dans le package pour que colcon l'installe)
from autosoaring_pkg import thermal_msg_pb2

# ROS2 Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

# Origin in GPS (lat, lon) corresponding to (x=0, y=0) in Gazebo
ORIGIN_LAT = 47.397971057728974
ORIGIN_LON = 8.546163739800146
EARTH_RADIUS = 6371000  # in meters

# Number of floats per thermal in the ROS message
# [id, lon, lat, x_enu, y_enu, radius, strength, lifetime, birth_time]
FIELDS_PER_THERMAL = 9


def haversine_xy(lat, lon):
    """Convert (lat, lon) to Gazebo (x, y) using the Haversine approximation from the origin."""
    dlat = math.radians(lat - ORIGIN_LAT)
    dlon = math.radians(lon - ORIGIN_LON)
    lat1 = math.radians(ORIGIN_LAT)
    lat2 = math.radians(lat)
    x = EARTH_RADIUS * dlon * math.cos((lat1 + lat2) / 2)
    y = EARTH_RADIUS * dlat
    return x, y


def enu_to_gps(x, y):
    """Convert ENU (x=East, y=North) in meters back to (lat, lon) using the origin."""
    lat = ORIGIN_LAT + math.degrees(y / EARTH_RADIUS)
    lon = ORIGIN_LON + math.degrees(x / (EARTH_RADIUS * math.cos(math.radians(ORIGIN_LAT))))
    return lat, lon


class ThermalGenerator(Node):
    def __init__(self, config_file):
        super().__init__('thermal_generator_node')
        self.get_logger().info("Thermal Generator Started")

        # Check if config file exists
        if not os.path.exists(config_file):
            self.get_logger().error(f"Config file not found: {config_file}")
            self.get_logger().error(f"Current working directory: {os.getcwd()}")
            self.get_logger().error("Please ensure the config file path is correct")
            raise FileNotFoundError(f"Config file not found: {config_file}")

        self.get_logger().info(f"Loading config from: {config_file}")
        with open(config_file, 'r') as f:
            cfg = yaml.safe_load(f)

        self.n_thermals = random.randint(cfg['n_thermals_range'][0], cfg['n_thermals_range'][1])
        self.zi_range = cfg['zi_range']
        self.w_star_range = cfg['w_star_range']
        self.lifespan_range = cfg['lifespan_range']
        self.min_distance = cfg.get('min_distance_between_thermals', 150.0)  # Read from YAML

        # Algorithm-relevant params (radius/strength derived from Gazebo zi/wi)
        # zi is convective boundary layer height → thermal radius ≈ 0.1 * zi (Allen model)
        # wi is convective velocity scale → maps directly to updraft strength (m/s)
        self.radius_factor = cfg.get('radius_factor', 0.1)  # radius = zi * factor
        self.min_radius = cfg.get('min_thermal_radius', 100.0)
        self.max_radius = cfg.get('max_thermal_radius', 300.0)

        self.thermals = []
        self.sim_time = 0
        self.next_id = 0

        # Simulation bounds (ENU meters) — thermals are generated directly in this space
        self.x_min = cfg.get('x_lower_bound', -1500.0)
        self.x_max = cfg.get('x_upper_bound', 1500.0)
        self.y_min = cfg.get('y_lower_bound', -1500.0)
        self.y_max = cfg.get('y_upper_bound', 1500.0)
        self.get_logger().info(
            f"Simulation bounds (ENU): X=[{self.x_min}, {self.x_max}], Y=[{self.y_min}, {self.y_max}]")

        # ROS2 Publishers
        # /generated_thermals: incremental (new thermals only) compatible with mapping node
        self.ros_pub = self.create_publisher(Float32MultiArray, '/generated_thermals', 10)
        # /thermal_snapshot: full list of ALL active thermals (for algorithm bridge)
        self.snapshot_pub = self.create_publisher(Float32MultiArray, '/thermal_snapshot', 10)
        # /thermal_removed: IDs of thermals that expired (for algorithm cleanup)
        self.removed_pub = self.create_publisher(Float32MultiArray, '/thermal_removed', 10)

        # Gazebo publisher
        self.gz_node = gz.Node()
        self.gz_pub = self.gz_node.advertise("/world/default/thermal_updrafts", thermal_msg_pb2.ThermalGroup)
        time.sleep(1.0)

        # Initial thermals
        self.generate_initial_thermals()
        self.run_loop()

    def _compute_radius(self, zi):
        """Derive algorithm-usable thermal radius from Gazebo zi parameter."""
        r = zi * self.radius_factor
        return max(self.min_radius, min(self.max_radius, r))

    def is_valid_thermal_location(self, x, y):
        """Check if a thermal location (ENU meters) is valid (inside bounds and far enough from others)."""
        if not (self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max):
            return False

        # Check minimum distance from existing thermals (Euclidean in ENU)
        for thermal in self.thermals:
            dx = x - thermal["x"]
            dy = y - thermal["y"]
            if math.sqrt(dx * dx + dy * dy) < self.min_distance:
                return False

        return True

    def generate_thermal(self, thermal_id):
        for _ in range(1000):  # Try up to 1000 times to find a valid location
            x = random.uniform(self.x_min, self.x_max)
            y = random.uniform(self.y_min, self.y_max)

            if self.is_valid_thermal_location(x, y):
                lat, lon = enu_to_gps(x, y)
                zi = random.uniform(*self.zi_range)
                wi = random.uniform(*self.w_star_range)
                radius = self._compute_radius(zi)
                return {
                    "id": thermal_id,
                    "lat": lat,
                    "lon": lon,
                    "x": x,
                    "y": y,
                    "zi": zi,
                    "wi": wi,
                    "radius": radius,      # algorithm-usable radius (m)
                    "strength": wi,         # updraft strength (m/s)
                    "lifetime": random.uniform(*self.lifespan_range),
                    "birth_time": self.sim_time
                }
        return None

    def generate_initial_thermals(self):
        while len(self.thermals) < self.n_thermals:
            t = self.generate_thermal(self.next_id)
            if t:
                self.thermals.append(t)
                self.next_id += 1
            else:
                self.get_logger().warning("Failed to place some thermals after max attempts.")
                break
        self.publish_thermals(self.thermals)

    def update_thermals(self):
        self.sim_time += 2
        # Find expired thermals before removing
        prev_ids = {t["id"] for t in self.thermals}
        self.thermals = [t for t in self.thermals if self.sim_time <= t["birth_time"] + t["lifetime"]]
        curr_ids = {t["id"] for t in self.thermals}
        removed_ids = prev_ids - curr_ids

        # Publish removed thermal IDs so subscribers can clean up
        if removed_ids:
            rm_msg = Float32MultiArray()
            rm_msg.data = [float(tid) for tid in removed_ids]
            self.removed_pub.publish(rm_msg)

        new_thermals = []

        # Generate new thermals to maintain count
        while len(self.thermals) < self.n_thermals:
            new_t = self.generate_thermal(self.next_id)
            if new_t:
                self.thermals.append(new_t)
                new_thermals.append(new_t)
                self.next_id += 1
            else:
                break

        return new_thermals  # Only return newly created thermals

    def publish_thermals(self, thermals_to_publish):
        if not thermals_to_publish:
            return

        msg = thermal_msg_pb2.ThermalGroup()
        ros_msg = Float32MultiArray()        # mapping-compatible (id, lon, lat)
        ros_msg_full = Float32MultiArray()   # full data for algorithm bridge

        for t in thermals_to_publish:
            m = msg.thermals.add()
            m.id = t["id"]
            m.x = t["x"]
            m.y = t["y"]
            m.zi = t["zi"]
            m.wi = t["wi"]
            m.lifetime = t["lifetime"]
            m.birth_time = t["birth_time"]

            # Compact message for mapping node (backward compatible)
            ros_msg.data.extend([float(t["id"]), float(t["lon"]), float(t["lat"])])

            # Full message for algorithm bridge (FIELDS_PER_THERMAL floats per thermal)
            ros_msg_full.data.extend([
                float(t["id"]),
                float(t["lon"]),
                float(t["lat"]),
                float(t["x"]),          # ENU x (meters)
                float(t["y"]),          # ENU y (meters)
                float(t["radius"]),     # algorithm radius (meters)
                float(t["strength"]),   # updraft strength (m/s)
                float(t["lifetime"]),   # duration (seconds)
                float(t["birth_time"]), # birth time in sim seconds
            ])

        self.gz_pub.publish(msg)
        self.ros_pub.publish(ros_msg)

    def publish_snapshot(self):
        """Publish the full list of ALL currently active thermals."""
        if not self.thermals:
            return
        snap_msg = Float32MultiArray()
        for t in self.thermals:
            snap_msg.data.extend([
                float(t["id"]),
                float(t["lon"]),
                float(t["lat"]),
                float(t["x"]),
                float(t["y"]),
                float(t["radius"]),
                float(t["strength"]),
                float(t["lifetime"]),
                float(t["birth_time"]),
            ])
        self.snapshot_pub.publish(snap_msg)

    def run_loop(self):
        while rclpy.ok():
            new_thermals = self.update_thermals()
            if new_thermals:
                self.publish_thermals(new_thermals)
            # Always publish full snapshot so late-joining subscribers get all thermals
            self.publish_snapshot()
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(2)


def main(args=None):
    import sys
    if len(sys.argv) < 2:
        print("Usage: ros2 run autosoaring_pkg thermal_generator_node path_to_config.yaml")
        return
    config_file = sys.argv[1]
    rclpy.init(args=args)
    node = ThermalGenerator(config_file)
    rclpy.shutdown()


if __name__ == '__main__':
    main() 