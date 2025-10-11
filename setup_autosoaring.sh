#!/bin/bash

# AutoSoaring Project Setup Script
# This script sets up the AutoSoaring project for open source distribution

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTOSOARING_DIR="$SCRIPT_DIR"
AUTOSOARING_APP_DIR="$(dirname "$SCRIPT_DIR")/autosoaring_app"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  AutoSoaring Project Setup${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to show usage
show_usage() {
    echo -e "${BLUE}Usage: $0 [OPTION]${NC}"
    echo ""
    echo -e "${BLUE}Options:${NC}"
    echo -e "  ${YELLOW}setup${NC}     - Set up environment and check dependencies (default)"
    echo -e "  ${YELLOW}build${NC}     - Build GZ_Msgs and ROS2 workspace"
    echo -e "  ${YELLOW}install${NC}   - Complete setup and build"
    echo -e "  ${YELLOW}start${NC}     - Start the AutoSoaring system"
    echo -e "  ${YELLOW}help${NC}      - Show this help message"
    echo ""
    echo -e "${BLUE}Examples:${NC}"
    echo -e "  ${YELLOW}$0${NC}           # Set up environment and check dependencies"
    echo -e "  ${YELLOW}$0 build${NC}     # Build GZ_Msgs and ROS2 workspace"
    echo -e "  ${YELLOW}$0 install${NC}   # Complete setup and build"
    echo -e "  ${YELLOW}$0 start${NC}     # Start the AutoSoaring system"
    echo ""
}

# Function to setup environment
setup_environment() {
    echo -e "${BLUE}Setting up AutoSoaring environment...${NC}"
    
    # Check if autosoaring_app directory exists
    if [ ! -d "$AUTOSOARING_APP_DIR" ]; then
        echo -e "${RED}Error: autosoaring_app directory not found at: $AUTOSOARING_APP_DIR${NC}"
        echo -e "${YELLOW}Please ensure autosoaring_app is in the same parent directory as autosoaring${NC}"
        return 1
    fi
    
    # Check dependencies
    echo -e "${BLUE}Checking dependencies...${NC}"
    
    if ! command_exists ros2; then
        echo -e "${RED}Error: ROS2 not found. Please install ROS2 Humble.${NC}"
        echo -e "${YELLOW}Installation guide: https://docs.ros.org/en/humble/Installation.html${NC}"
        return 1
    fi
    
    if ! command_exists python3; then
        echo -e "${RED}Error: Python3 not found. Please install Python3.${NC}"
        return 1
    fi
    
    # Check Python packages
    echo -e "${BLUE}Checking Python packages...${NC}"
    
    python3 -c "import rclpy" 2>/dev/null || {
        echo -e "${RED}Error: rclpy not found. Please install ROS2 Python packages.${NC}"
        return 1
    }
    
    python3 -c "import mavsdk" 2>/dev/null || {
        echo -e "${YELLOW}Warning: mavsdk not found. Installing...${NC}"
        pip3 install mavsdk
    }
    
    python3 -c "import matplotlib" 2>/dev/null || {
        echo -e "${YELLOW}Warning: matplotlib not found. Installing...${NC}"
        pip3 install matplotlib
    }
    
    python3 -c "import shapely" 2>/dev/null || {
        echo -e "${YELLOW}Warning: shapely not found. Installing...${NC}"
        pip3 install shapely
    }
    
    # Check GZ_Msgs protobuf files
    echo -e "${BLUE}Checking GZ_Msgs protobuf files...${NC}"
    
    GZ_MSGS_DIR="$AUTOSOARING_DIR/src/autosoaring_pkg/GZ_Msgs"
    if [ ! -d "$GZ_MSGS_DIR" ]; then
        echo -e "${RED}Error: GZ_Msgs directory not found at: $GZ_MSGS_DIR${NC}"
        return 1
    fi
    
    if [ ! -f "$GZ_MSGS_DIR/python/thermal_msg_pb2.py" ]; then
        echo -e "${YELLOW}Warning: thermal_msg_pb2.py not found. Building GZ_Msgs...${NC}"
        cd "$GZ_MSGS_DIR"
        if [ -d "build" ]; then
            rm -rf build
        fi
        mkdir -p build && cd build
        cmake .. && make
        cd "$AUTOSOARING_DIR"
    fi
    
    echo -e "${GREEN}✓ Dependencies OK${NC}"
    echo ""
    
    # Set environment variables
    echo -e "${BLUE}Setting up environment variables...${NC}"
    
    export AUTOSOARING_DIR="$AUTOSOARING_DIR"
    export AUTOSOARING_APP_DIR="$AUTOSOARING_APP_DIR"
    
    echo -e "${GREEN}✓ Environment variables set${NC}"
    echo ""
    echo -e "${GREEN}Environment setup complete!${NC}"
    echo ""
    echo -e "${BLUE}To use these settings in your current shell, run:${NC}"
    echo -e "${YELLOW}  source $0 setup${NC}"
    echo ""
    echo -e "${BLUE}To make these settings permanent, add to your ~/.bashrc:${NC}"
    echo -e "${YELLOW}  echo 'source \$(pwd)/$0 setup' >> ~/.bashrc${NC}"
    echo ""
}

# Function to build GZ_Msgs
build_gz_msgs() {
    echo -e "${BLUE}Building GZ_Msgs protobuf files...${NC}"
    
    GZ_MSGS_DIR="$AUTOSOARING_DIR/src/autosoaring_pkg/GZ_Msgs"
    if [ ! -d "$GZ_MSGS_DIR" ]; then
        echo -e "${RED}Error: GZ_Msgs directory not found${NC}"
        return 1
    fi
    
    cd "$GZ_MSGS_DIR"
    
    # Clean previous build
    if [ -d "build" ]; then
        echo -e "${YELLOW}Cleaning previous build...${NC}"
        rm -rf build
    fi
    
    echo -e "${BLUE}Creating build directory...${NC}"
    mkdir -p build && cd build
    
    echo -e "${BLUE}Configuring with CMake...${NC}"
    if ! cmake ..; then
        echo -e "${RED}Error: CMake configuration failed for GZ_Msgs${NC}"
        return 1
    fi
    
    echo -e "${BLUE}Building GZ_Msgs...${NC}"
    if ! make; then
        echo -e "${RED}Error: Building GZ_Msgs failed${NC}"
        return 1
    fi
    
    echo -e "${GREEN}✓ GZ_Msgs built successfully${NC}"
    echo ""
}

# Function to build the workspace
build_workspace() {
    echo -e "${BLUE}Building ROS2 workspace...${NC}"
    
    if [ ! -d "$AUTOSOARING_APP_DIR" ]; then
        echo -e "${RED}Error: autosoaring_app directory not found${NC}"
        return 1
    fi
    
    cd "$AUTOSOARING_APP_DIR"
    
    echo -e "${BLUE}Sourcing ROS2 environment...${NC}"
    source /opt/ros/humble/setup.bash
    
    echo -e "${BLUE}Building workspace...${NC}"
    if colcon build; then
        echo -e "${GREEN}✓ Workspace built successfully${NC}"
    else
        echo -e "${RED}Error: Workspace build failed${NC}"
        return 1
    fi
    
    echo -e "${BLUE}Sourcing workspace...${NC}"
    source install/setup.bash
    
    echo -e "${GREEN}✓ Workspace ready${NC}"
    echo ""
}

# Function to install everything
install_autosoaring() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  Installing AutoSoaring System${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    
    # Setup environment
    setup_environment
    
    # Build GZ_Msgs
    build_gz_msgs
    
    # Build workspace
    build_workspace
    
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}  Installation Complete!${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo -e "${BLUE}AutoSoaring system is ready to use.${NC}"
    echo ""
    echo -e "${BLUE}To start the system:${NC}"
    echo -e "${YELLOW}  $0 start${NC}"
    echo ""
    echo -e "${BLUE}Or manually:${NC}"
    echo -e "${YELLOW}  cd $AUTOSOARING_DIR${NC}"
    echo -e "${YELLOW}  ./start_autosoaring.sh${NC}"
}

# Function to start the system
start_autosoaring() {
    echo -e "${BLUE}Starting AutoSoaring system...${NC}"
    
    if [ ! -f "$AUTOSOARING_DIR/start_autosoaring.sh" ]; then
        echo -e "${RED}Error: start_autosoaring.sh not found${NC}"
        return 1
    fi
    
    cd "$AUTOSOARING_DIR"
    chmod +x start_autosoaring.sh
    ./start_autosoaring.sh
}

# Main script logic
case "${1:-setup}" in
    "setup")
        setup_environment
        ;;
    "build")
        build_gz_msgs
        build_workspace
        ;;
    "install")
        install_autosoaring
        ;;
    "start")
        start_autosoaring
        ;;
    "help"|"-h"|"--help")
        show_usage
        ;;
    *)
        echo -e "${RED}Error: Unknown option '$1'${NC}"
        echo ""
        show_usage
        exit 1
        ;;
esac
