import thermal_pb2
from gz.transport13 import Node
import time

def main():
    node = Node()
    thermal_group_topic = "/thermal_msg_group"

    # Advertise the topic with the ThermalGroup message type
    pub = node.advertise(thermal_group_topic, thermal_pb2.ThermalGroup)

    # Create and populate the ThermalGroup message
    thermal_group = thermal_pb2.ThermalGroup()

    # Add three Thermal messages to the group
    thermal1 = thermal_group.thermals.add()
    thermal1.id=1
    thermal1.latitude = 500.5
    thermal1.longitude = 20.0
    thermal1.radius = 20.0
    thermal1.initial_strength = 30.0
    thermal1.strength = 150.0
    thermal1.life_cycle = 12
    thermal1.birth_time = 12.12

    thermal2 = thermal_group.thermals.add()
    thermal2.id=2
    thermal2.latitude = 600.5
    thermal2.longitude = 30.0
    thermal2.radius = 30.0
    thermal2.initial_strength = 40.0
    thermal2.strength = 180.0
    thermal2.life_cycle = 15
    thermal2.birth_time = 13.14
    
    thermal3 = thermal_group.thermals.add()
    thermal3.id=3
    thermal3.latitude = 700.5
    thermal3.longitude = 40.0
    thermal3.radius = 50.0
    thermal3.initial_strength = 50.0
    thermal3.strength = 200.0
    thermal3.life_cycle = 20
    thermal3.birth_time = 19.99
    # Publish the ThermalGroup message in a loop to keep the publisher alive
    while True:
        pub.publish(thermal_group)
        print("Published ThermalGroup message with 3 thermals to /thermal_msg_group")
        time.sleep(1)

if __name__ == "__main__":
    main()
