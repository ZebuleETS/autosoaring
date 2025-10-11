import json
import yaml
import random
import time
import math
import gz.transport13 as gz
import thermal_msg_pb2
from shapely.geometry import Polygon, Point

# ROS2 Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Origin in GPS (lat, lon) corresponding to (x=0, y=0) in Gazebo
ORIGIN_LAT = 47.397971057728974
ORIGIN_LON = 8.546163739800146
EARTH_RADIUS = 6371000  # in meters


def haversine_xy(lat, lon):
    """Convert (lat, lon) to Gazebo (x, y) using the Haversine approximation from the origin."""
    dlat = math.radians(lat - ORIGIN_LAT)
    dlon = math.radians(lon - ORIGIN_LON)
    lat1 = math.radians(ORIGIN_LAT)
    lat2 = math.radians(lat)
    x = EARTH_RADIUS * dlon * math.cos((lat1 + lat2) / 2)
    y = EARTH_RADIUS * dlat
    return x, y


def calculate_distance(lat1, lon1, lat2, lon2):
    """Calculate the distance between two GPS coordinates using Haversine formula."""
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    
    a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = EARTH_RADIUS * c
    
    return distance


class ThermalGenerator(Node):
    def __init__(self, config_file):
        super().__init__('thermal_generator_node')
        self.get_logger().info("Thermal Generator Started")

        with open(config_file, 'r') as f:
            cfg = yaml.safe_load(f)

        self.plan_file = cfg['qgc_plan_path']
        self.n_thermals = cfg['num_thermals']
        self.zi_range = cfg['zi_range']
        self.w_star_range = cfg['w_star_range']
        self.lifespan_range = cfg['lifespan_range']
        self.min_distance = cfg.get('min_distance_between_thermals', 150.0)  # Read from YAML
        self.thermals = []
        self.sim_time = 0
        self.next_id = 0

        # ROS2 Publisher
        self.ros_pub = self.create_publisher(Float32MultiArray, '/generated_thermals', 10)

        # Geofence
        self.gps_polygon = self.read_geofence(self.plan_file)
        self.bounding_box = self.gps_polygon.bounds
        self.get_logger().info(f"Geofence bounds: {self.bounding_box}")
        self.get_logger().info(f"Minimum distance between thermals: {self.min_distance}m")

        # Gazebo publisher
        self.gz_node = gz.Node()
        self.gz_pub = self.gz_node.advertise("/world/default/thermal_updrafts", thermal_msg_pb2.ThermalGroup)
        time.sleep(1.0)

        # Initial thermals
        self.generate_initial_thermals()
        self.run_loop()

    def read_geofence(self, plan_file):
        with open(plan_file, 'r') as f:
            data = json.load(f)
            coords = data['geoFence']['polygons'][0]['polygon']
            return Polygon([(lat, lon) for lat, lon in coords])

    def is_valid_thermal_location(self, lat, lon):
        """Check if a thermal location is valid (within geofence and minimum distance from others)."""
        # Check if within geofence
        if not self.gps_polygon.covers(Point(lat, lon)):
            return False
        
        # Check minimum distance from existing thermals
        for thermal in self.thermals:
            distance = calculate_distance(lat, lon, thermal["lat"], thermal["lon"])
            if distance < self.min_distance:
                return False
        
        return True

    def generate_thermal(self, thermal_id):
        minx, miny, maxx, maxy = self.bounding_box
        for _ in range(1000):  # Try up to 1000 times to find a valid location
            lat = random.uniform(minx, maxx)
            lon = random.uniform(miny, maxy)
            
            if self.is_valid_thermal_location(lat, lon):
                x, y = haversine_xy(lat, lon)
                return {
                    "id": thermal_id,
                    "lat": lat,
                    "lon": lon,
                    "x": x,
                    "y": y,
                    "zi": random.uniform(*self.zi_range),
                    "wi": random.uniform(*self.w_star_range),
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
        # Remove expired thermals
        self.thermals = [t for t in self.thermals if self.sim_time <= t["birth_time"] + t["lifetime"]]
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
        ros_msg = Float32MultiArray()

        for t in thermals_to_publish:
            m = msg.thermals.add()
            m.id = t["id"]
            m.x = t["x"]
            m.y = t["y"]
            m.zi = t["zi"]
            m.wi = t["wi"]
            m.lifetime = t["lifetime"]
            m.birth_time = t["birth_time"]

            # Pack all thermals into a single ROS message
            ros_msg.data.extend([float(t["id"]), float(t["lon"]), float(t["lat"])])

        self.gz_pub.publish(msg)
        self.ros_pub.publish(ros_msg)

        self.get_logger().info(
            f"Published {len(thermals_to_publish)} new thermals at time {self.sim_time}s"
        )
        self.get_logger().info(f"ROS message data: {ros_msg.data}")

    def run_loop(self):
        while rclpy.ok():
            new_thermals = self.update_thermals()
            if new_thermals:
                self.publish_thermals(new_thermals)
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