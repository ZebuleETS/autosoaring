import thermal_pb2
from gz.transport13 import Node
import time

def main():
    node = Node()
    thermal_topic = "/thermal_msg"

    # Advertise the topic with the Thermal message type
    pub = node.advertise(thermal_topic, thermal_pb2.Thermal)

    # Create and populate the Thermal message
    thermal = thermal_pb2.Thermal()
    thermal.height = 10.5
    thermal.radius = 5.0
    thermal.x_coordinates = 100.0
    thermal.y_coordinates = 200.0
    thermal.force_applied = 15.0
    thermal.life_stamp = 1234567890

    # Publish the message in a loop to keep the publisher alive
    while True:
        pub.publish(thermal)
        print("Published message to /thermal_msg")
        time.sleep(1)

if __name__ == "__main__":
    main()
