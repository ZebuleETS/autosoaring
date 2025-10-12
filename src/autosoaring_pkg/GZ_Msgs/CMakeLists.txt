cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(msgs VERSION 1.0.0)

# Find dependencies
find_package(gz-cmake3 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Define the list of .proto files
set(MSGS_PROTOS
    ${CMAKE_CURRENT_SOURCE_DIR}/gz/msgs/thermal.proto
)

# Generate Protobuf messages
gz_msgs_generate_messages(
  TARGET nsgs
  PROTO_PACKAGE "gz.msgs"
  MSGS_PATH ${CMAKE_CURRENT_SOURCE_DIR}
  MSGS_PROTOS ${MSGS_PROTOS}
  DEPENDENCIES gz-msgs10::gz-msgs10
)


