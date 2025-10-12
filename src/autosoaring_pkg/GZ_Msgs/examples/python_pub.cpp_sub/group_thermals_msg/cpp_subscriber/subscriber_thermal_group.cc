#include <gz/transport/Node.hh>
#include <gz/msgs/thermal.pb.h>
#include <iostream>

// Callback function to process received ThermalGroup messages
void ThermalGroupCallback(const gz::msgs::ThermalGroup &msg)
{
    std::cout << "Received ThermalGroup Message:" << std::endl;

    for (int i = 0; i < msg.thermals_size(); ++i)
    {
        const auto &thermal = msg.thermals(i);
        std::cout << "  Thermal " << i + 1 << ":" << std::endl;
        std::cout << "    Height: " << thermal.height() << std::endl;
        std::cout << "    Radius: " << thermal.radius() << std::endl;
        std::cout << "    X Coordinates: " << thermal.x_coordinates() << std::endl;
        std::cout << "    Y Coordinates: " << thermal.y_coordinates() << std::endl;
        std::cout << "    Force Applied: " << thermal.force_applied() << std::endl;
        std::cout << "    Life Stamp: " << thermal.life_stamp() << std::endl;
    }
}

int main(int argc, char **argv)
{
    // Initialize Gazebo Transport node
    gz::transport::Node node;

    // Subscribe to the thermal group message topic
    std::string thermalGroupTopic = "/thermal_msg_group";
    if (!node.Subscribe(thermalGroupTopic, ThermalGroupCallback))
    {
        std::cerr << "Error subscribing to topic [" << thermalGroupTopic << "]" << std::endl;
        return -1;
    }

    std::cout << "Subscribed to topic [" << thermalGroupTopic << "]" << std::endl;

    // Keep the node running to listen for messages
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}

