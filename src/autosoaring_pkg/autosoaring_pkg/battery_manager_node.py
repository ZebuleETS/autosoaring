import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float32

class BatteryManager(Node):
    def __init__(self):
        super().__init__('battery_manager')

        # Subscribers for Airspeed and Throttle
        self.airspeed_sub = self.create_subscription(Float32, 'airspeed', self.airspeed_callback, 10)
        self.throttle_sub = self.create_subscription(Float32, 'throttle', self.throttle_callback, 10)

        # Publisher for Battery Status
        self.battery_pub = self.create_publisher(Float32, 'battery_status', 10)

        # Battery Parameters
        self.battery_capacity = 30.0  # Wh (Initial full battery)
        self.max_battery_capacity = 30.0  # Wh
        self.time_step = 1.0  # Update every second
        self.propeller_eff = 0.75  # Propeller efficiency
        self.energy_eff = 0.6  # Energy conversion efficiency
        self.low_battery_threshold = 20.0  # 20% battery warning
        self.running = True  # Start immediately (no service dependency)

        # Store latest values
        self.airspeed = 0.0
        self.throttle = 0.0

        # Timer (runs immediately)
        self.timer = self.create_timer(self.time_step, self.update_battery)

        self.get_logger().info("BatteryManager Node Started - Self-starting mode")


    
    def airspeed_callback(self, msg):
        """Updates airspeed from MAVSDK main program."""
        self.airspeed = msg.data
        # self.get_logger().debug(f"Received Airspeed: {self.airspeed} m/s")

    def throttle_callback(self, msg):
        """Updates throttle from MAVSDK main program."""
        self.throttle = msg.data
        # self.get_logger().debug(f"Received Throttle: {self.throttle}")

    def compute_power_consumption(self):
        """Computes power consumption using airspeed and throttle."""
        thrust = self.throttle * 9.81  # Approximate thrust from throttle
        power = (thrust * self.airspeed) / (self.propeller_eff * self.energy_eff)
        return max(0, power)  # Ensure no negative power values

    def update_battery(self):
        """Updates battery level at each time step."""
        if not self.running:
            self.get_logger().warn("Battery system is not running yet.")
            return  # Only run after receiving start request

        power_consumption = self.compute_power_consumption()
        energy_used = (power_consumption * self.time_step) / 3600  # Convert W·s to Wh
        self.battery_capacity -= energy_used
        self.battery_capacity = max(0, self.battery_capacity)  # Prevent negative battery

        # Compute battery percentage
        battery_percentage = (self.battery_capacity / self.max_battery_capacity) * 100

        # Publish battery status
        msg = Float32()
        msg.data = battery_percentage
        self.battery_pub.publish(msg)

        # Only log battery status every 10 seconds to avoid flooding the terminal
        if not hasattr(self, 'last_battery_log') or (time.time() - getattr(self, 'last_battery_log', 0)) >= 10.0:
            #self.get_logger().info(f"Battery Status: {battery_percentage:.2f}%")
            self.last_battery_log = time.time()

        # Log warning if battery is low
        if battery_percentage <= self.low_battery_threshold:
            self.get_logger().warn(f"Low Battery! ({battery_percentage:.2f}%) - Consider landing soon.")

        # Stop if battery is fully depleted
        if self.battery_capacity == 0:
            #self.get_logger().error("Battery depleted! UAV must land.")
            self.running = False  # Stop battery system


def main(args=None):
    rclpy.init(args=args)
    battery_manager = BatteryManager()
    rclpy.spin(battery_manager)
    battery_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 