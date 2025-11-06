#!/bin/bash
# Quick Launch Script for Arm System
# Usage: ./quick_launch.sh [mode]
# Modes: sim, real, voice, minimal

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored messages
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if workspace is sourced
if [ -z "$ROS_DISTRO" ]; then
    print_error "ROS2 not sourced! Please run:"
    echo "  source /opt/ros/humble/setup.bash"
    echo "  source install/setup.bash"
    exit 1
fi

# Get mode from argument or show menu
MODE=$1

if [ -z "$MODE" ]; then
    echo "=========================================="
    echo "    Arm System Quick Launch Menu"
    echo "=========================================="
    echo "1) sim       - Full simulation system"
    echo "2) real      - Real robot system"
    echo "3) voice     - Headless voice control"
    echo "4) minimal   - Minimal system"
    echo "5) perception - Perception testing"
    echo "6) planning  - Planning testing"
    echo "=========================================="
    read -p "Select mode (1-6): " selection
    
    case $selection in
        1) MODE="sim";;
        2) MODE="real";;
        3) MODE="voice";;
        4) MODE="minimal";;
        5) MODE="perception";;
        6) MODE="planning";;
        *) print_error "Invalid selection"; exit 1;;
    esac
fi

# Launch based on mode
case $MODE in
    sim|simulation)
        print_info "Launching full simulation system..."
        print_warn "YOLO perception must be started separately!"
        echo "  In another terminal run: ros2 run arm_bringup start_yolo_node.sh"
        echo ""
        ros2 launch arm_bringup sim_bringup.launch.py
        ;;
    
    real|robot)
        print_info "Launching real robot system..."
        print_warn "Make sure robot is connected!"
        print_warn "YOLO perception must be started separately!"
        echo "  In another terminal run: ros2 run arm_bringup start_yolo_node.sh"
        echo ""
        
        # Check for serial port
        if [ ! -e "/dev/ttyUSB0" ] && [ ! -e "/dev/ttyACM0" ]; then
            print_warn "No serial device found at /dev/ttyUSB0 or /dev/ttyACM0"
            read -p "Enter serial port (or press Enter to continue): " SERIAL_PORT
            if [ ! -z "$SERIAL_PORT" ]; then
                ros2 launch arm_bringup real_bringup.launch.py serial_port:=$SERIAL_PORT
            else
                ros2 launch arm_bringup real_bringup.launch.py
            fi
        else
            ros2 launch arm_bringup real_bringup.launch.py
        fi
        ;;
    
    voice|headless)
        print_info "Launching headless voice control..."
        print_info "Say commands like: 'scan front', 'scan all', 'grasp'"
        ros2 launch arm_bringup voice_headless.launch.py
        ;;
    
    minimal|min)
        print_info "Launching minimal system..."
        ros2 launch arm_bringup minimal_bringup.launch.py
        ;;
    
    perception|percept|camera)
        print_info "Launching perception system..."
        ros2 launch arm_bringup perception_bringup.launch.py
        ;;
    
    planning|plan|moveit)
        print_info "Launching planning system..."
        ros2 launch arm_bringup planning_bringup.launch.py
        ;;
    
    *)
        print_error "Unknown mode: $MODE"
        echo "Available modes: sim, real, voice, minimal, perception, planning"
        exit 1
        ;;
esac
