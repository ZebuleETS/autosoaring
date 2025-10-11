# AutoSoaring UAV System

An advanced autonomous soaring system for fixed-wing UAVs that enables energy-efficient flight through thermal updraft detection and exploitation.

## Features

- **Thermal Detection**: Real-time detection of thermal updrafts using MAVSDK
- **Thermal Mapping**: 3D mapping and visualization of thermal data
- **Thermal Generation**: Configurable thermal field simulation
- **Battery Management**: Intelligent power management for extended flight
- **Mission Planning**: Integration with QGroundControl mission files
- **3D Path Visualization**: Comprehensive flight path analysis

## System Requirements

### Software Dependencies
- **ROS2 Humble** - Robot Operating System 2
- **Python 3.8+** - Python runtime
- **MAVSDK** - MAVLink SDK for drone communication
- **Matplotlib** - Data visualization
- **NumPy** - Numerical computing
- **Shapely** - Geometric operations
- **Gazebo** - Simulation environment
- **Protobuf** - Message serialization

### Hardware Requirements
- **Fixed-wing UAV** with MAVLink-compatible autopilot
- **Ground Control Station** (QGroundControl recommended)
- **Telemetry link** (WiFi, 4G, or radio)

## Quick Start

### 1. Clone the Repository
```bash
git clone <your-repo-url>
cd autosoaring
```

### 2. Setup and Build
```bash
# Complete setup and build
./setup_autosoaring.sh install

# Or step by step:
./setup_autosoaring.sh setup    # Check dependencies
./setup_autosoaring.sh build    # Build ROS2 workspace
```

### 3. Start the System
```bash
# Start AutoSoaring system
./setup_autosoaring.sh start

# Or manually:
./start_autosoaring.sh
```

## Project Structure

```
autosoaring/
├── setup_autosoaring.sh          # Main setup script
├── start_autosoaring.sh          # System startup script
├── README_OPENSOURCE.md          # This file
├── src/
│   └── autosoaring_pkg/
│       ├── autosoaring_pkg/
│       │   ├── thermal_detection_node.py    # MAVSDK-based thermal detection
│       │   ├── thermal_mapping_node.py      # Thermal data mapping
│       │   ├── thermal_generator_node.py    # Thermal field generation
│       │   └── battery_manager_node.py      # Power management
│       ├── config/
│       │   ├── thermal_config.yaml          # Thermal generation config
│       │   ├── area2.plan                   # Mission waypoints
│       │   └── area3.plan                   # Alternative mission
│       ├── GZ_Msgs/                         # Gazebo protobuf messages
│       │   ├── gz/msgs/thermal.proto        # Thermal message definition
│       │   ├── python/thermal_msg_pb2.py    # Generated Python protobuf
│       │   └── build/                       # Built protobuf files
│       └── launch/
│           └── autosoaring_launch.py        # ROS2 launch file
└── autosoaring_app/                         # ROS2 workspace (separate directory)
    ├── src/
    ├── build/
    └── install/
```

## Configuration

### Thermal Configuration (`thermal_config.yaml`)
```yaml
qgc_plan_path: "area2.plan"              # Mission file (relative path)
num_thermals: 12                         # Number of thermals to generate
min_distance_between_thermals: 200.0     # Minimum separation (meters)
zi_range: [1250.0, 1250.0]              # Thermal height range (meters)
w_star_range: [5, 5]                    # Thermal strength range (m/s)
lifespan_range: [1200000.0, 1800000.0]  # Thermal lifetime range (seconds)
```

### Mission Files
- **`area2.plan`**: Default mission with waypoints for thermal exploration
- **`area3.plan`**: Alternative mission configuration

### Protobuf Messages
The system uses custom Gazebo protobuf messages for thermal communication:
- **`thermal.proto`**: Message definition for thermal data
- **`thermal_msg_pb2.py`**: Generated Python bindings
- **`thermal_pb2.py`**: Alternative Python bindings

The protobuf messages are automatically built during setup and provide efficient communication between the thermal generator and Gazebo simulation.

## Usage

### Starting the System
```bash
# From autosoaring directory
./setup_autosoaring.sh start
```

### Monitoring the System
```bash
# View active nodes
ros2 node list

# Monitor thermal data
ros2 topic echo /thermal_data

# Check system status
ps aux | grep python3
```

### Stopping the System
- **Ctrl+C** in the terminal running the system
- Or: `pkill -f 'python3.*thermal'`

## Output Files

The system generates several output files:

### Thermal Maps
- **`thermal_simple_plot_*.png`**: 2D thermal field visualizations
- Automatically opened when system stops

### Flight Data
- **`uav_path_3d_*.csv`**: 3D flight path data
- Contains: timestamp, latitude, longitude, altitude, climb_rate, energy

### Telemetry Data
- **`telemetry_data_*.csv`**: Complete flight telemetry
- Contains: time, altitude, climb_rate, energy, throttle, airspeed

## Advanced Usage

### Custom Mission Planning
1. Create mission in QGroundControl
2. Export as `.plan` file
3. Place in `src/autosoaring_pkg/config/`
4. Update `thermal_config.yaml` with new filename

### Thermal Detection Tuning
Edit `thermal_detection_node.py` to adjust:
- **Climb rate thresholds** for thermal detection
- **Orbit parameters** for thermal exploitation
- **Centering algorithms** for thermal navigation

### Battery Management
Configure in `battery_manager_node.py`:
- **Power thresholds** for different flight modes
- **Energy management** strategies
- **Emergency procedures** for low battery

## Troubleshooting

### Common Issues

1. **"ROS2 not found"**
   ```bash
   # Install ROS2 Humble
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

2. **"MAVSDK not found"**
   ```bash
   pip3 install mavsdk
   ```

3. **"Workspace build failed"**
   ```bash
   # Check dependencies
   ./setup_autosoaring.sh setup
   
   # Clean and rebuild
   cd autosoaring_app
   rm -rf build install
   colcon build
   ```

4. **"No thermal data"**
   - Check UAV connection
   - Verify telemetry link
   - Ensure mission is uploaded

### Debug Mode
```bash
# Run with verbose output
ros2 run autosoaring_pkg thermal_detection_node --ros-args --log-level debug
```

## Development

### Adding New Features
1. Create new Python node in `src/autosoaring_pkg/autosoaring_pkg/`
2. Update `package.xml` and `setup.py` if needed
3. Add launch configuration in `launch/`
4. Test with `colcon build` and `source install/setup.bash`

### Code Structure
- **Thermal Detection**: MAVSDK integration for real-time data
- **Thermal Mapping**: Data processing and visualization
- **Thermal Generation**: Configurable thermal field simulation
- **Battery Management**: Power monitoring and optimization

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## Support

For issues and questions:
- Check the troubleshooting section
- Review the code documentation
- Open an issue on the repository

## Acknowledgments

- **MAVSDK** for drone communication
- **ROS2** for distributed system architecture
- **QGroundControl** for mission planning
- **Open source community** for tools and libraries
