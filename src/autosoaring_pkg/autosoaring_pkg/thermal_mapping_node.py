#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Float32
from sensor_msgs.msg import NavSatFix
import csv
import time
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
import math
import subprocess
import sys
import os
import signal
import re

# Earth parameters for coordinate conversion
EARTH_RADIUS = 6371000  # meters

def latlon_to_xy(lat, lon, origin_lat=None, origin_lon=None):
    """Convert (lat, lon) to local x (m), y (m) coordinates using equirectangular approximation."""
    if origin_lat is None or origin_lon is None:
        # Use the first point as origin if not specified
        return lat, lon  # Return as-is for now, will be set later
    
    dlat = math.radians(lat - origin_lat)
    dlon = math.radians(lon - origin_lon)
    lat_avg = math.radians((lat + origin_lat) / 2.0)
    
    x = EARTH_RADIUS * dlon * math.cos(lat_avg)
    y = EARTH_RADIUS * dlat
    
    return x, y

class ThermalMappingNode(Node):
    def __init__(self):
        super().__init__('thermal_mapping_node')
        print(" Initializing Thermal Mapping Node...")
        
        # Data storage
        self.uav_path = []  # 2D path data
        self.uav_path_3d = []  # 3D path data
        self.thermal_data_3d = []  # Thermal detection data
        self.thermal_tracks = []  # Thermal track data
        self.generated_points = []  # Generated thermal points
        self.generated_thermals = []  # Generated thermal data (for CSV)
        self.thermal_detections = []  # Thermal detection data (for CSV)
        self.thermal_skipped = []  # Thermal skipped data (for CSV)
        self.current_throttle = 0.0  # Current throttle value
        self.start_time = time.time()
        
        # Flag to prevent multiple saves
        self.data_saved = False
        
        # Flag to track if initial plot was created
        self.initial_plot_created = False
        
        # Set up signal handlers for immediate data saving
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        print(" Signal handlers registered for Ctrl+C and SIGTERM")
        
        # Subscribers
        self.create_subscription(
            String, '/thermal_detected', self.callback_detected, 10)
        self.create_subscription(
            String, '/thermal_skipped', self.callback_skipped, 10)
        self.create_subscription(
            Float32MultiArray, '/generated_thermals', self.callback_generated, 10)
        self.create_subscription(
            Float32MultiArray, '/uav/position', self.callback_position, 10)
        self.create_subscription(
            Float32, '/uav/throttle', self.callback_throttle, 10)
        self.create_subscription(
            String, '/uav/flight_mode', self.callback_flight_mode, 10)
        
        # Timer for status logging (commented out to reduce verbose output)
        # self.create_timer(5.0, self.log_status)
        
        # Keep a minimal timer to ensure the node stays responsive to signals
        self.create_timer(1.0, self.minimal_heartbeat)
        
        self.get_logger().info(" Thermal Mapping Node started!")
        # self.get_logger().info(" Subscribed to topics:")
        # self.get_logger().info("   - /thermal_detected")
        # self.get_logger().info("   - /thermal_skipped") 
        # self.get_logger().info("   - /generated_thermals")
        # self.get_logger().info("   - /uav/position")
        # self.get_logger().info("   - /uav/flight_mode")
        
        # Initial plot will be triggered when first thermal data is received
        # No timer needed - will be triggered by thermal data callbacks

    def minimal_heartbeat(self):
        """Minimal heartbeat to keep the node responsive to signals"""
        # Do nothing, just keep the node alive and responsive
        pass

    def signal_handler(self, signum, frame):
        """Handle shutdown signals to save data immediately"""
        print(f"\n SIGNAL HANDLER TRIGGERED! Signal {signum} received.")
        if not self.data_saved:
            print(f" Saving data immediately...")
            self.data_saved = True
            self.save_data_to_csv()
            print(" Data saved successfully!")
            
            # Also create the final thermal map
            print(" Creating final thermal map...")
            try:
                png_filename = self.create_simple_plot_with_title("Final Thermal Map - Simulation End")
                if png_filename:
                    print(f" Final thermal map created: {png_filename}")
                    # Try to open the thermal map
                    try:
                        subprocess.Popen(['xdg-open', png_filename])
                        print(" Final thermal map opened in image viewer")
                    except:
                        print(f" Please open manually: {png_filename}")
                else:
                    print("  No final thermal map created (no data to plot)")
            except Exception as e:
                print(f" Error creating final thermal map: {e}")
        else:
            print(f"\n Signal {signum} received. Data already saved.")
        
        print("\n" + "="*60)
        print(" Final thermal map created!")
        print(" Close the plot window when you're done viewing.")
        print("="*60)
        
        # Exit gracefully
        sys.exit(0)

    def log_status(self):
        """Log current data collection status"""
        self.get_logger().info("="*60)
        self.get_logger().info(" DATA COLLECTION STATUS:")
        self.get_logger().info(f"   UAV Path Points: {len(self.uav_path)}")
        self.get_logger().info(f"   UAV Path 3D Points: {len(self.uav_path_3d)}")
        self.get_logger().info(f"   Thermal Detections: {len(self.thermal_detections)}")
        self.get_logger().info(f"   Thermal Skipped: {len(self.thermal_skipped)}")
        self.get_logger().info(f"   Generated Thermals: {len(self.generated_thermals)}")
        
        if self.uav_path_3d:
            latest = self.uav_path_3d[-1]
            self.get_logger().info(f"   Latest UAV Position: Lat={latest['lat']:.6f}, Lon={latest['lon']:.6f}, Alt={latest['alt']:.1f}m")
        
        if self.generated_thermals:
            self.get_logger().info(f"   Generated Thermal Locations:")
            for i, thermal in enumerate(self.generated_thermals[:5]):  # Show first 5
                self.get_logger().info(f"     Thermal {i}: Lat={thermal['lat']:.6f}, Lon={thermal['lon']:.6f}")
            if len(self.generated_thermals) > 5:
                self.get_logger().info(f"     ... and {len(self.generated_thermals) - 5} more")
        
        self.get_logger().info("="*60)

    def create_initial_plot(self):
        """Create initial plot showing the first thermal map when thermal data is received"""
        if self.initial_plot_created:
            return
            
        # self.get_logger().info(" Creating initial thermal map with received thermal data...")
        try:
            filename = self.create_simple_plot_with_title("Initial Thermal Map - First Thermal Data Received")
            if filename:
                self.get_logger().info(f" Initial thermal map created: {filename}")
                self.initial_plot_created = True
                # Try to open the initial plot
                try:
                    subprocess.Popen(['xdg-open', filename])
                    # self.get_logger().info(" Initial thermal map opened in image viewer")
                except:
                    # self.get_logger().info(f" Please open manually: {filename}")
                    pass
            else:
                self.get_logger().warn(" No initial plot created (no data available yet)")
        except Exception as e:
            self.get_logger().error(f" Error creating initial plot: {e}")

    def callback_detected(self, msg):
        """Handle thermal detection messages"""
        try:
            data = msg.data.strip()
            self.get_logger().info(f" THERMAL DETECTED: '{data}'")
            
            parts = data.split()
            if len(parts) < 2:
                self.get_logger().warn(f"  Invalid thermal detection message format: '{data}'")
                return
            
            if "Thermal START" in data:
                self.get_logger().info(f"    Thermal START detected")
                # Parse thermal start data using regex-like string parsing
                try:
                    
                    # Extract lat, lon, alt, climb, and v_energy using regex patterns
                    lat_match = re.search(r'lat=([+-]?\d+\.?\d*)', data)
                    lon_match = re.search(r'lon=([+-]?\d+\.?\d*)', data)
                    alt_match = re.search(r'alt=([+-]?\d+\.?\d*)', data)
                    climb_match = re.search(r'climb=([+-]?\d+\.?\d*)', data)
                    v_energy_match = re.search(r'v_energy=([+-]?\d+\.?\d*)', data)
                    
                    if lat_match and lon_match:
                        lat = float(lat_match.group(1))
                        lon = float(lon_match.group(1))
                        
                        # Try to get altitude if available, otherwise use 0
                        if alt_match:
                            alt = float(alt_match.group(1))
                        else:
                            alt = 0.0  # Default altitude if not provided
                        
                        # Try to get climb rate and v_energy if available
                        climb_rate = float(climb_match.group(1)) if climb_match else 0.0
                        v_energy = float(v_energy_match.group(1)) if v_energy_match else 0.0
                        
                        self.get_logger().info(f"    Thermal START at: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.1f}m, Climb={climb_rate:.2f}m/s, V_energy={v_energy:.2f}m/s")
                        
                        # Store thermal detection with enhanced data
                        thermal_data = {
                            'lat': lat, 'lon': lon, 'alt': alt, 'climb_rate': climb_rate, 'v_energy': v_energy,
                            'time': time.time() - self.start_time,
                            'type': 'start'
                        }
                        self.thermal_data_3d.append(thermal_data)
                        self.thermal_detections.append(thermal_data)
                    else:
                        self.get_logger().error(f"    Could not extract lat/lon from: '{data}'")
                    
                except (ValueError, AttributeError) as e:
                    self.get_logger().error(f"    Error parsing thermal START data: {e}")
                    
            elif "Thermal CORE" in data:
                self.get_logger().info(f"    Thermal CORE detected")
                # Parse thermal core data using regex-like string parsing
                try:
                    
                    # Extract lat, lon, alt, climb, v_energy, and throttle using regex patterns
                    lat_match = re.search(r'lat=([+-]?\d+\.?\d*)', data)
                    lon_match = re.search(r'lon=([+-]?\d+\.?\d*)', data)
                    alt_match = re.search(r'alt=([+-]?\d+\.?\d*)', data)
                    climb_match = re.search(r'climb=([+-]?\d+\.?\d*)', data)
                    v_energy_match = re.search(r'v_energy=([+-]?\d+\.?\d*)', data)
                    throttle_match = re.search(r'throttle=([+-]?\d+\.?\d*)', data)
                    
                    if lat_match and lon_match:
                        lat = float(lat_match.group(1))
                        lon = float(lon_match.group(1))
                        
                        # Try to get altitude if available, otherwise use 0
                        if alt_match:
                            alt = float(alt_match.group(1))
                        else:
                            alt = 0.0  # Default altitude if not provided
                        
                        # Try to get climb rate, v_energy, and throttle if available
                        climb_rate = float(climb_match.group(1)) if climb_match else 0.0
                        v_energy = float(v_energy_match.group(1)) if v_energy_match else 0.0
                        throttle = float(throttle_match.group(1)) if throttle_match else 0.0
                        
                        self.get_logger().info(f"    Thermal CORE at: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.1f}m, Climb={climb_rate:.2f}m/s, V_energy={v_energy:.2f}m/s, Throttle={throttle:.1f}%")
                        
                        # Store thermal core with enhanced data
                        thermal_data = {
                            'lat': lat, 'lon': lon, 'alt': alt, 'climb_rate': climb_rate, 'v_energy': v_energy, 'throttle': throttle,
                            'time': time.time() - self.start_time,
                            'type': 'core'
                        }
                        self.thermal_data_3d.append(thermal_data)
                        self.thermal_detections.append(thermal_data)
                    else:
                        self.get_logger().error(f"    Could not extract lat/lon from: '{data}'")
                    
                except (ValueError, AttributeError) as e:
                    self.get_logger().error(f"    Error parsing thermal CORE data: {e}")
                    
            elif "Thermal END" in data:
                self.get_logger().info(f"    Thermal END detected")
                # Parse thermal end data using regex-like string parsing
                try:
                    
                    # Extract lat, lon, alt, climb, and v_energy using regex patterns
                    lat_match = re.search(r'lat=([+-]?\d+\.?\d*)', data)
                    lon_match = re.search(r'lon=([+-]?\d+\.?\d*)', data)
                    alt_match = re.search(r'alt=([+-]?\d+\.?\d*)', data)
                    climb_match = re.search(r'climb=([+-]?\d+\.?\d*)', data)
                    v_energy_match = re.search(r'v_energy=([+-]?\d+\.?\d*)', data)
                    
                    if lat_match and lon_match:
                        lat = float(lat_match.group(1))
                        lon = float(lon_match.group(1))
                        
                        # Try to get altitude if available, otherwise use 0
                        if alt_match:
                            alt = float(alt_match.group(1))
                        else:
                            alt = 0.0  # Default altitude if not provided
                        
                        # Try to get climb rate and v_energy if available
                        climb_rate = float(climb_match.group(1)) if climb_match else 0.0
                        v_energy = float(v_energy_match.group(1)) if v_energy_match else 0.0
                        
                        self.get_logger().info(f"    Thermal END at: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.1f}m, Climb={climb_rate:.2f}m/s, V_energy={v_energy:.2f}m/s")
                        
                        # Store thermal end with enhanced data
                        thermal_data = {
                            'lat': lat, 'lon': lon, 'alt': alt, 'climb_rate': climb_rate, 'v_energy': v_energy,
                            'time': time.time() - self.start_time,
                            'type': 'end'
                        }
                        self.thermal_data_3d.append(thermal_data)
                    else:
                        self.get_logger().error(f"    Could not extract lat/lon from: '{data}'")
                    
                except (ValueError, AttributeError) as e:
                    self.get_logger().error(f"    Error parsing thermal END data: {e}")
            else:
                self.get_logger().warn(f"     Unknown thermal detection message type: '{data}'")
                
        except Exception as e:
            self.get_logger().error(f" Error in thermal detection callback: {e}")
            self.get_logger().error(f"   Raw message: '{msg.data}'")

    def callback_skipped(self, msg):
        """Handle thermal skipped messages"""
        try:
            data = msg.data.strip()
            self.get_logger().info(f"  THERMAL SKIPPED: '{data}'")
            
            parts = data.split()
            if len(parts) >= 4:
                try:
                    lat = float(parts[0])
                    lon = float(parts[1])
                    alt = float(parts[2])
                    reason = parts[3] if len(parts) > 3 else "unknown"
                    
                    self.get_logger().info(f"     Skipped at: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.1f}m, Reason: {reason}")
                    
                    # Store skipped thermal
                    thermal_data = {
                        'lat': lat, 'lon': lon, 'alt': alt,
                        'time': time.time() - self.start_time,
                        'type': 'skipped',
                        'reason': reason
                    }
                    self.thermal_data_3d.append(thermal_data)
                    self.thermal_skipped.append(thermal_data)
                    
                except (ValueError, IndexError) as e:
                    self.get_logger().error(f"    Error parsing thermal skipped data: {e}")
            else:
                self.get_logger().warn(f"     Invalid thermal skipped message format: '{data}'")
                
        except Exception as e:
            self.get_logger().error(f" Error in thermal skipped callback: {e}")
            self.get_logger().error(f"   Raw message: '{msg.data}'")

    def callback_generated(self, msg):
        """Handle generated thermal messages"""
        try:
            data = msg.data
            
            # Process every 3 elements: [id, lon, lat, id, lon, lat, ...]
            for i in range(0, len(data), 3):
                if i + 2 < len(data):
                    thermal_id = int(data[i])
                    lon = data[i + 1]
                    lat = data[i + 2]
                    
                    # Store generated thermal (no logging)
                    self.generated_points.append((lat, lon))
                    
                    # Also store for CSV
                    thermal_data = {
                        'lat': lat, 'lon': lon, 'id': thermal_id,
                        'time': time.time() - self.start_time
                    }
                    self.generated_thermals.append(thermal_data)
                else:
                    self.get_logger().warn(f"     Incomplete thermal data at index {i}, need 3 elements but only have {len(data) - i}")
            
            # self.get_logger().info(f" Generated {len(data)//3} thermals received and stored")
            
            # Create initial plot when first thermal data is received
            if not self.initial_plot_created and self.generated_points:
                self.get_logger().info(" First thermal data received - creating initial thermal map...")
                self.create_initial_plot()
                
        except Exception as e:
            self.get_logger().error(f" Error in generated thermals callback: {e}")
            self.get_logger().error(f"   Raw message data: {msg.data}")

    def callback_position(self, msg):
        """Handle UAV position messages"""
        try:
            data = msg.data
            if len(data) >= 3:
                lat = data[0]
                lon = data[1]
                alt = data[2]
                
                # Store 2D path data (no logging)
                self.uav_path.append({
                    'lat': lat, 'lon': lon, 'time': time.time() - self.start_time
                })
                
                # Store 3D path data with throttle (no logging)
                self.uav_path_3d.append({
                    'lat': lat, 'lon': lon, 'alt': alt, 'throttle': self.current_throttle, 'time': time.time() - self.start_time
                })
            else:
                self.get_logger().warn(f"  Invalid UAV position data: {data}")
            
        except Exception as e:
            self.get_logger().error(f" Error in UAV position callback: {e}")
            self.get_logger().error(f"   Raw message data: {msg.data}")

    def callback_throttle(self, msg):
        """Handle throttle messages"""
        try:
            throttle = msg.data
            self.current_throttle = throttle
            # No logging to reduce terminal output
            
        except Exception as e:
            self.get_logger().error(f" Error in throttle callback: {e}")

    def callback_flight_mode(self, msg):
        """Handle flight mode messages"""
        try:
            mode = msg.data
            self.get_logger().info(f"  FLIGHT MODE: {mode}")
            
        except Exception as e:
            self.get_logger().error(f" Error in flight mode callback: {e}")

    def save_data_to_csv(self):
        """Save UAV 3D path and generated thermals data to CSV files"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.get_logger().info(f" Saving CSV data with timestamp: {timestamp}")
            # print(f" UAV Path 3D Points available: {len(self.uav_path_3d)}")
            # print(f" Generated Points available: {len(self.generated_points)}")
            
            # Create flights_data directory if it doesn't exist
            flights_data_dir = "flights_data"
            if not os.path.exists(flights_data_dir):
                os.makedirs(flights_data_dir)
                print(f" Created directory: {flights_data_dir}")
            
            # 1. Save UAV 3D path data
            if self.uav_path_3d:
                uav_3d_filename = os.path.join(flights_data_dir, f"uav_path_3d_{timestamp}.csv")
                # print(f" Creating UAV 3D path file: {uav_3d_filename}")
                
                with open(uav_3d_filename, 'w', newline='') as csvfile:
                    fieldnames = ['timestamp', 'lat', 'lon', 'altitude', 'throttle', 'time_from_start']
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    writer.writeheader()
                    
                    for i, point in enumerate(self.uav_path_3d):
                        writer.writerow({
                            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                            'lat': point['lat'],
                            'lon': point['lon'],
                            'altitude': point['alt'],
                            'throttle': point['throttle'],
                            'time_from_start': point['time']
                        })
                        
                        # Print progress every 50 points
                        # if (i + 1) % 50 == 0:
                        #     print(f"    Written {i + 1}/{len(self.uav_path_3d)} points...")
                
                self.get_logger().info(f" UAV 3D path saved to: {uav_3d_filename}")
                # print(f" File size: {os.path.getsize(uav_3d_filename)} bytes")
            else:
                # print("  No UAV 3D path data to save")
                pass
            
            # 2. Save generated thermals data
            if self.generated_points:
                generated_filename = os.path.join(flights_data_dir, f"generated_thermals_{timestamp}.csv")
                # print(f"💾 Creating generated thermals file: {generated_filename}")
                
                with open(generated_filename, 'w', newline='') as csvfile:
                    fieldnames = ['thermal_id', 'lat', 'lon', 'radius_meters']
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    writer.writeheader()
                    for i, (lat, lon) in enumerate(self.generated_points):
                        writer.writerow({
                            'thermal_id': i,
                            'lat': lat,
                            'lon': lon,
                            'radius_meters': 70.0
                        })
                self.get_logger().info(f" Generated thermals saved to: {generated_filename}")
            else:
                # print("  No generated thermals data to save")
                pass
            
            self.get_logger().info(f" Data saved with timestamp: {timestamp}")
            # print(" CSV files created:")
            # if self.uav_path_3d:
            #     print(f"   - {uav_3d_filename}")
            # if self.generated_points:
            #     print(f"   - {generated_filename}")
            
            # Verify files were created
            if self.uav_path_3d and os.path.exists(uav_3d_filename):
                # print(f" Verified: {uav_3d_filename} exists and is readable")
                pass
            elif self.uav_path_3d:
                self.get_logger().error(f" ERROR: {uav_3d_filename} was not created!")
            
        except Exception as e:
            print(f" Error saving CSV data: {e}")
            import traceback
            traceback.print_exc()

    def create_simple_plot_with_title(self, title="Thermal Mapping - Generated Thermals (70m radius)"):
        """Create a simple plot to verify data with custom title"""
        try:
            print(f" Creating plot: {title}")
            print(f"   UAV path points: {len(self.uav_path)}")
            print(f"   Generated points: {len(self.generated_points)}")
            
            if not self.uav_path and not self.generated_points:
                print("    No data to plot!")
                return None
            
            fig, ax = plt.subplots(figsize=(12, 10))
            
            # Determine origin point for coordinate conversion
            all_lats = []
            all_lons = []
            
            if self.uav_path:
                all_lats.extend([point['lat'] for point in self.uav_path])
                all_lons.extend([point['lon'] for point in self.uav_path])
            
            if self.generated_points:
                all_lats.extend([point[0] for point in self.generated_points])
                all_lons.extend([point[1] for point in self.generated_points])
            
            if self.thermal_detections:
                all_lats.extend([detection['lat'] for detection in self.thermal_detections])
                all_lons.extend([detection['lon'] for detection in self.thermal_detections])
            
            if not all_lats or not all_lons:
                self.get_logger().warn("  No coordinate data to plot!")
                return None
            
            # Use the center of all data as origin
            origin_lat = sum(all_lats) / len(all_lats)
            origin_lon = sum(all_lons) / len(all_lons)
            
            self.get_logger().info(f"    Origin point: Lat={origin_lat:.6f}, Lon={origin_lon:.6f}")
            
            # Plot UAV path in meters
            if self.uav_path:
                uav_x = []
                uav_y = []
                for point in self.uav_path:
                    x, y = latlon_to_xy(point['lat'], point['lon'], origin_lat, origin_lon)
                    uav_x.append(x)
                    uav_y.append(y)
                
                ax.plot(uav_x, uav_y, 'b-', linewidth=2, label='UAV Path')
                self.get_logger().info(f"    Plotted {len(self.uav_path)} UAV path points in meters")
            
            # Plot generated thermals as circles with 70m radius
            if self.generated_points:
                for lat, lon in self.generated_points:
                    # Convert to meters
                    x, y = latlon_to_xy(lat, lon, origin_lat, origin_lon)
                    
                    # Create circle with 70m radius
                    circle = plt.Circle((x, y), 70.0, 
                                      color='red', alpha=0.3, fill=True, 
                                      edgecolor='red', linewidth=2)
                    ax.add_patch(circle)
                    
                    # Add center point
                    ax.scatter(x, y, c='red', marker='o', s=50, alpha=0.8)
                
                self.get_logger().info(f"    Plotted {len(self.generated_points)} generated thermals with 70m radius circles")
            
            # Plot thermal detection points (start, core, end)
            if self.thermal_detections:
                start_points = [d for d in self.thermal_detections if d['type'] == 'start']
                core_points = [d for d in self.thermal_detections if d['type'] == 'core']
                end_points = [d for d in self.thermal_detections if d['type'] == 'end']
                
                # Plot thermal START points (green)
                if start_points:
                    start_x = []
                    start_y = []
                    for point in start_points:
                        x, y = latlon_to_xy(point['lat'], point['lon'], origin_lat, origin_lon)
                        start_x.append(x)
                        start_y.append(y)
                    ax.scatter(start_x, start_y, c='green', marker='o', s=100, alpha=0.8, label=f'Thermal Start ({len(start_points)})')
                    self.get_logger().info(f"    Plotted {len(start_points)} thermal START points (green)")
                
                # Plot thermal CORE points (black)
                if core_points:
                    core_x = []
                    core_y = []
                    for point in core_points:
                        x, y = latlon_to_xy(point['lat'], point['lon'], origin_lat, origin_lon)
                        core_x.append(x)
                        core_y.append(y)
                    ax.scatter(core_x, core_y, c='black', marker='o', s=100, alpha=0.8, label=f'Thermal Core ({len(core_points)})')
                    self.get_logger().info(f"    Plotted {len(core_points)} thermal CORE points (black)")
                
                # Plot thermal END points (yellow)
                if end_points:
                    end_x = []
                    end_y = []
                    for point in end_points:
                        x, y = latlon_to_xy(point['lat'], point['lon'], origin_lat, origin_lon)
                        end_x.append(x)
                        end_y.append(y)
                    ax.scatter(end_x, end_y, c='yellow', marker='o', s=100, alpha=0.8, label=f'Thermal End ({len(end_points)})')
                    self.get_logger().info(f"    Plotted {len(end_points)} thermal END points (yellow)")
            
            ax.set_xlabel('X (meters)')
            ax.set_ylabel('Y (meters)')
            ax.set_title(title)
            
            # Create custom legend with color-coded thermal detections
            from matplotlib.patches import Circle
            legend_elements = [
                plt.Line2D([0], [0], color='blue', linewidth=2, label='UAV Path'),
                Circle((0, 0), 70, color='red', alpha=0.3, fill=True, label='Generated Thermals (70m radius)')
            ]
            
            # Add thermal detection legend elements if they exist
            if self.thermal_detections:
                start_points = [d for d in self.thermal_detections if d['type'] == 'start']
                core_points = [d for d in self.thermal_detections if d['type'] == 'core']
                end_points = [d for d in self.thermal_detections if d['type'] == 'end']
                
                if start_points:
                    legend_elements.append(plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='green', markersize=8, label=f'Thermal Start ({len(start_points)})'))
                if core_points:
                    legend_elements.append(plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='black', markersize=8, label=f'Thermal Core ({len(core_points)})'))
                if end_points:
                    legend_elements.append(plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='yellow', markersize=8, label=f'Thermal End ({len(end_points)})'))
            
            
            ax.legend(handles=legend_elements, loc='upper right', fontsize=10)
            
            ax.grid(True, alpha=0.3)
            
            # Set equal aspect ratio to make circles appear circular
            ax.set_aspect('equal', adjustable='box')
            
            # Save plot with appropriate filename based on title
            # Create flights_data directory if it doesn't exist
            flights_data_dir = "flights_data"
            if not os.path.exists(flights_data_dir):
                os.makedirs(flights_data_dir)
                print(f" Created directory: {flights_data_dir}")
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            if "Initial" in title:
                filename = os.path.join(flights_data_dir, f"thermal_initial_map_{timestamp}.png")
            elif "Final" in title:
                filename = os.path.join(flights_data_dir, f"thermal_final_map_{timestamp}.png")
            else:
                filename = os.path.join(flights_data_dir, f"thermal_simple_plot_{timestamp}.png")
                
            # print(f" Saving plot to: {filename}")
            plt.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"  Plot saved successfully: {filename}")
            self.get_logger().info(f" Plot saved as: {filename}")
            
            # Save plot and close figure (don't show interactive window)
            plt.close(fig)  # Close the figure to free memory
            
            print(f"  Returning filename: {filename}")
            return filename  # Return the filename so main() can use it
            
        except Exception as e:
            self.get_logger().error(f" Error creating plot: {e}")
            import traceback
            traceback.print_exc()
            return None

    def create_simple_plot(self):
        """Create a simple plot to verify data (backward compatibility)"""
        return self.create_simple_plot_with_title("Thermal Mapping - Generated Thermals (70m radius)")

def main():
    print(" Starting Thermal Mapping Node...")
    print(" Current working directory:", os.getcwd())
    
    try:
        rclpy.init()
        print(" ROS2 initialized successfully!")
        
        node = ThermalMappingNode()
        print(" Thermal Mapping Node created successfully!")
        
        print(" Starting to spin thermal mapping node...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(" Ctrl+C detected. Signal handler will save data...")
        
        # The signal handler will save the data and create the final map automatically
        # Just wait a moment for the signal handler to complete
        time.sleep(2)
        
        # Shutdown ROS2 to avoid conflicts
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            print(f"ROS2 shutdown error (ignored): {e}")
        
        print("\n" + "="*60)
        print("  Final thermal map should be created by signal handler!")
        print(" Check for PNG files in the current directory.")
        print("="*60)

if __name__ == '__main__':
    main()
