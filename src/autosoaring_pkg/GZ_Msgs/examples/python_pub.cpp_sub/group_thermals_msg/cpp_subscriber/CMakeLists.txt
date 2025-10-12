cmake_minimum_required(VERSION 3.10)
project(ThermalGroupSubscriber)

# Find required packages
find_package(gz-transport13 REQUIRED)
find_package(Protobuf REQUIRED)

# Include directories for generated Protobuf files
include_directories(
    /home/radhouene/custom_messages/build/msgs-nsgs_genmsg  # Path to generated Protobuf headers
)

# Add the executable
add_executable(thermal_sub 
    subscriber_thermal_group.cc 
    /home/radhouene/custom_messages/build/msgs-nsgs_genmsg/gz/msgs/thermal.pb.cc  # Generated Protobuf source file
)

# Link libraries
target_link_libraries(thermal_sub 
    gz-transport13::gz-transport13  # Gazebo Transport
    ${Protobuf_LIBRARIES}           # Protobuf library
)

# Include Protobuf headers
target_include_directories(thermal_sub PRIVATE ${Protobuf_INCLUDE_DIRS})

