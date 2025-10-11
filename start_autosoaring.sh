#!/bin/bash

# AutoSoaring UAV System Startup Script
# Original script for launching the autosoaring system

echo " Starting AutoSoaring UAV System..."

# Set the workspace directory
WORKSPACE_DIR="/home/radhouene/autosoaring_app"
cd "$WORKSPACE_DIR"

# Source ROS2 environment
echo " Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash

# Source the workspace
echo " Sourcing workspace..."
source install/setup.bash

# Set the source directory for Python files
SRC_DIR="/home/radhouene/autosoaring_app/src/autosoaring_pkg/autosoaring_pkg"

# Get the package directory for config files
PKG_DIR=$(ros2 pkg prefix autosoaring_pkg)/share/autosoaring_pkg
CONFIG_FILE="$PKG_DIR/config/thermal_config.yaml"

echo " Starting AutoSoaring components..."

# Start thermal mapping node first (to receive generated thermals)
echo " Starting thermal_mapping_node..."
python3 "$SRC_DIR/thermal_mapping_node.py" &
THERMAL_MAPPING_PID=$!
echo " thermal_mapping_node started with PID: $THERMAL_MAPPING_PID"

# Wait a moment for mapping node to start
sleep 2

# Start thermal generator node (to publish thermals)
echo " Starting thermal_generator_node..."
python3 "$SRC_DIR/thermal_generator_node.py" "$CONFIG_FILE" &
THERMAL_GENERATOR_PID=$!
echo " thermal_generator_node started with PID: $THERMAL_GENERATOR_PID"

# Wait a moment for generator node to start
sleep 2

# Start thermal detection node (MAVSDK-based)
echo " Starting thermal_detection_node..."
python3 "$SRC_DIR/thermal_detection_node.py" &
THERMAL_DETECTION_PID=$!
echo " thermal_detection_node started with PID: $THERMAL_DETECTION_PID"

# Wait a moment for detection node to start
sleep 2

# Start battery manager node
echo " Starting battery_manager_node..."
python3 "$SRC_DIR/battery_manager_node.py" &
BATTERY_MANAGER_PID=$!
echo " battery_manager_node started with PID: $BATTERY_MANAGER_PID"

echo ""
echo " AutoSoaring system startup complete!"
echo ""
echo " System Status:"
echo "=================="
echo "• thermal_mapping_node: PID $THERMAL_MAPPING_PID"
echo "• thermal_generator_node: PID $THERMAL_GENERATOR_PID"
echo "• thermal_detection_node: PID $THERMAL_DETECTION_PID"
echo "• battery_manager_node: PID $BATTERY_MANAGER_PID"

echo ""
echo " Available topics:"
echo "==================="
ros2 topic list 2>/dev/null || echo "Topics will be available once nodes are fully started"

echo ""
echo " Available services:"
echo "====================="
ros2 service list 2>/dev/null || echo "Services will be available once nodes are fully started"

echo ""
echo " To monitor the system:"
echo "========================"
echo "• View node graph: ros2 node list"
echo "• Monitor topics: ros2 topic echo /thermal_data"
echo "• Check system status: ps aux | grep python3"
echo "• View logs: Check terminal output above"
echo ""
echo "To stop the system:"
echo "====================="
echo "• Press Ctrl+C in this terminal"
echo "• Or run: pkill -f 'python3.*thermal'"
echo "• Or run: pkill -f 'python3.*battery'"
echo ""

# Keep the script running to maintain the background processes
echo " System is running. Press Ctrl+C to stop all nodes..."

            # Function to cleanup on exit
            cleanup() {
                echo ""
                echo " Stopping AutoSoaring system..."

                # Send SIGINT to thermal mapping node first to allow it to save the thermal map
                echo " Saving thermal map and 3D path data..."
                kill -SIGINT $THERMAL_MAPPING_PID 2>/dev/null
                
                # Wait much longer for thermal mapping node to complete its shutdown and save data
                echo " Waiting for thermal mapping node to save data..."
                sleep 10  # Give it much more time to save both thermal map and 3D path data
                
                # Check if thermal mapping node is still running
                if kill -0 $THERMAL_MAPPING_PID 2>/dev/null; then
                    echo "  Thermal mapping node still running, waiting a bit more..."
                    sleep 5
                    # Force kill if still running
                    kill -9 $THERMAL_MAPPING_PID 2>/dev/null
                fi
                
                # Then stop all other nodes
                echo " Stopping other nodes..."
                kill $THERMAL_GENERATOR_PID $THERMAL_DETECTION_PID $BATTERY_MANAGER_PID 2>/dev/null
                
                # Wait a moment for all processes to stop
                sleep 2
                
                echo " All nodes stopped"
                
                # Check if thermal map was created
                echo " Checking for thermal map files..."
                if ls thermal_simple_plot_*.png 1> /dev/null 2>&1; then
                    echo " Thermal map found! Opening..."
                    latest_thermal_map=$(ls -t thermal_simple_plot_*.png | head -1)
                    echo " Latest thermal map: $latest_thermal_map"
                    xdg-open "$latest_thermal_map" 2>/dev/null || echo "📁 Please open manually: $latest_thermal_map"
                else
                    echo "  No thermal map found. Check if thermal mapping node had data to plot."
                fi
                
                # Check if 3D path data was created
                echo " Checking for 3D path data files..."
                if ls uav_path_3d_*.csv 1> /dev/null 2>&1; then
                    echo " 3D path data found!"
                    latest_3d_path=$(ls -t uav_path_3d_*.csv | head -1)
                    echo " Latest 3D path data: $latest_3d_path"
                    echo " You can now run: python3 src/autosoaring_pkg/autosoaring_pkg/uav_3d_path_visualizer.py"
                    echo "   And select option 0 to view the latest 3D path visualization"
                else
                    echo "  No 3D path data found. The thermal mapping node may not have received position data."
                fi
                
                exit 0
            }

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Wait for all background processes
wait
