#include <gz/transport/Node.hh>
#include <gz/msgs/thermal.pb.h>
#include <iostream>

// Callback function to process received messages
void ThermalCallback(const gz::msgs::Thermal &msg)
{
    std::cout << "Received Thermal Message:" << std::endl;
    std::cout << "  Height: " << msg.height() << std::endl;
    std::cout << "  Radius: " << msg.radius() << std::endl;
    std::cout << "  X Coordinates: " << msg.x_coordinates() << std::endl;
    std::cout << "  Y Coordinates: " << msg.y_coordinates() << std::endl;
    std::cout << "  Force Applied: " << msg.force_applied() << std::endl;
    std::cout << "  Life Stamp: " << msg.life_stamp() << std::endl;
}

int main(int argc, char **argv)
{
    // Initialize Gazebo Transport node
    gz::transport::Node node;

    // Subscribe to the thermal message topic
    std::string thermalTopic = "/thermal_msg";
    if (!node.Subscribe(thermalTopic, ThermalCallback))
    {
        std::cerr << "Error subscribing to topic [" << thermalTopic << "]" << std::endl;
        return -1;
    }

    std::cout << "Subscribed to topic [" << thermalTopic << "]" << std::endl;

    // Keep the node running to listen for messages
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
