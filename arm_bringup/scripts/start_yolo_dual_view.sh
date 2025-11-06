#!/bin/bash
# Wrapper script to launch YOLO with dual-view triangulation measurement
# This script activates the yolo_venv and launches the perception system with triangulation enabled

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  Dual-View Triangulation System Startup${NC}"
echo -e "${CYAN}========================================${NC}"

# Default virtual environment path
VENV_PATH="${HOME}/yolo_venv"

# Check if virtual environment exists
if [ ! -d "$VENV_PATH" ]; then
    echo -e "${YELLOW}Virtual environment does not exist, calling start_yolo_node.sh to create...${NC}"
    exec "$(dirname "$0")/start_yolo_node.sh" enable_triangulation:=true enable_coordinator:=true enable_projection:=false
    exit 0
fi

echo -e "${GREEN}Using virtual environment: $VENV_PATH${NC}"

# Activate virtual environment
source "$VENV_PATH/bin/activate"

# Source ROS2 workspace
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
else
    echo -e "${RED}ROS2 not found! Please install ROS2 first.${NC}"
    exit 1
fi

# Source the arm workspace
WORKSPACE_ROOT="${HOME}/lododo-arm"
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/install/setup.bash"
fi

echo ""
echo -e "${CYAN}Launch mode: Dual-view triangulation${NC}"
echo -e "${GREEN}Nodes:${NC}"
echo -e "  ✓ YoloDetector (virtual environment)"
echo -e "  ✓ TriangulationNode"
echo -e "  ✓ MeasurementCoordinator"
echo -e "  ✓ ScenePublisher"
echo -e "  ✗ ProjectionNode (disabled)"
echo ""
echo -e "${YELLOW}Description:${NC}"
echo -e "  • Traditional ProjectionNode disabled"
echo -e "  • Using triangulation to automatically calculate object size"
echo -e "  • Default manual mode (enable_arm_movement=false)"
echo -e "  • Need to manually move camera to second viewpoint"
echo ""
echo -e "${CYAN}Available services:${NC}"
echo -e "  ros2 service call /scan_and_measure arm_interfaces/srv/ScanAndMeasure"
echo ""
echo -e "${CYAN}Available actions:${NC}"
echo -e "  ros2 action send_goal /measure_object_action arm_interfaces/action/MeasureObject"
echo ""

# Force Python to use the venv's packages first
export PYTHONPATH="$VENV_PATH/lib/python3.10/site-packages:$PYTHONPATH"

# Launch with triangulation enabled, projection disabled
exec ros2 launch arm_perception_yolo yolo_perception_launch.py \
    enable_triangulation:=true \
    enable_coordinator:=true \
    enable_projection:=false \
    "$@"
