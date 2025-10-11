# Autosoaring ROS 2 Package

This package provides autonomous soaring capabilities for UAVs with thermal detection, generation, mapping, and battery management.

## Features

- **Thermal Generator Node**: Generates thermal updrafts in simulation environment
- **Thermal Detection Node**: Detects and exploits thermal updrafts during flight
- **Thermal Mapping Node**: Maps and visualizes thermal data and flight paths
- **Battery Manager Node**: Monitors and manages battery consumption

## Installation

1. Clone this repository to your ROS 2 workspace:
```bash
cd ~/your_ros2_workspace/src
git clone <repository_url> autosoaring
```

2. Build the package:
```bash
cd ~/your_ros2_workspace
colcon build --packages-select autosoaring_pkg
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Launch all nodes:
```bash
ros2 launch autosoaring_pkg autosoaring_launch.py
```

### Launch individual nodes:
```bash
# Thermal generator (requires config file)
ros2 run autosoaring_pkg thermal_generator_node /path/to/config.yaml

# Thermal detection
ros2 run autosoaring_pkg thermal_detection_node

# Thermal mapping
ros2 run autosoaring_pkg thermal_mapping_node

# Battery manager
ros2 run autosoaring_pkg battery_manager_node
```

### Using the start script:
```bash
./start_autosoaring.sh
```

## Configuration

The thermal generator node requires a configuration file. An example configuration is provided in `config/thermal_config.yaml`.

## Dependencies

- ROS 2 (tested with Humble)
- Python 3.8+
- MAVSDK (for thermal detection node)
- Additional Python packages as specified in the nodes

## License

MIT License
