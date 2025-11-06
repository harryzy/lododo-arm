#!/bin/bash
# System Health Check Script for Arm System
# Checks if all required components are properly configured

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

PASS="${GREEN}✓${NC}"
FAIL="${RED}✗${NC}"
WARN="${YELLOW}⚠${NC}"

echo -e "${BLUE}=========================================="
echo "  Arm System Health Check"
echo -e "==========================================${NC}\n"

# Check ROS2 installation
echo -n "Checking ROS2 installation... "
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${FAIL} ROS2 not sourced"
    echo "  Run: source /opt/ros/humble/setup.bash"
    exit 1
else
    echo -e "${PASS} ROS2 $ROS_DISTRO"
fi

# Check workspace
echo -n "Checking workspace... "
if [ -d "install/arm_bringup" ]; then
    echo -e "${PASS} arm_bringup found"
else
    echo -e "${FAIL} arm_bringup not built"
    echo "  Run: colcon build --packages-select arm_bringup"
    exit 1
fi

# Check all required packages
echo -e "\n${BLUE}Checking required packages:${NC}"
PACKAGES=("arm_description" "arm_driver_node" "arm_moveit_config" "arm_perception_yolo" "arm_planning_py" "arm_voice_interface" "arm_rviz_plugin")
MISSING_PACKAGES=()

for pkg in "${PACKAGES[@]}"; do
    echo -n "  $pkg... "
    if [ -d "install/$pkg" ]; then
        echo -e "${PASS}"
    else
        echo -e "${FAIL}"
        MISSING_PACKAGES+=($pkg)
    fi
done

if [ ${#MISSING_PACKAGES[@]} -ne 0 ]; then
    echo -e "\n${WARN} Missing packages: ${MISSING_PACKAGES[*]}"
    echo "  Run: colcon build"
fi

# Check serial ports
echo -e "\n${BLUE}Checking hardware connections:${NC}"
echo -n "  Serial ports... "
SERIAL_PORTS=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true)
if [ -z "$SERIAL_PORTS" ]; then
    echo -e "${WARN} No serial devices found"
    echo "    Connect robot and check: ls /dev/tty*"
else
    echo -e "${PASS}"
    echo "$SERIAL_PORTS" | while read port; do
        echo "    Found: $port"
    done
fi

# Check camera
echo -n "  Camera devices... "
CAMERAS=$(ls /dev/video* 2>/dev/null || true)
if [ -z "$CAMERAS" ]; then
    echo -e "${WARN} No camera found"
    echo "    For perception features, connect a camera"
else
    echo -e "${PASS}"
    echo "$CAMERAS" | while read cam; do
        echo "    Found: $cam"
    done
fi

# Check microphone (for voice control)
echo -n "  Audio devices... "
if command -v arecord &> /dev/null; then
    MIC_COUNT=$(arecord -l 2>/dev/null | grep -c "card" || true)
    if [ $MIC_COUNT -gt 0 ]; then
        echo -e "${PASS} ($MIC_COUNT device(s))"
    else
        echo -e "${WARN} No microphone found"
        echo "    For voice control, connect a microphone"
    fi
else
    echo -e "${WARN} arecord not found"
fi

# Check dependencies
echo -e "\n${BLUE}Checking software dependencies:${NC}"

# Gazebo
echo -n "  Gazebo... "
if command -v gazebo &> /dev/null; then
    echo -e "${PASS}"
else
    echo -e "${FAIL} Not installed"
    echo "    Install: sudo apt install ros-humble-gazebo-ros-pkgs"
fi

# MoveIt
echo -n "  MoveIt... "
if [ -f "/opt/ros/$ROS_DISTRO/share/moveit_ros_move_group/package.xml" ]; then
    echo -e "${PASS}"
else
    echo -e "${FAIL} Not installed"
    echo "    Install: sudo apt install ros-humble-moveit"
fi

# RViz
echo -n "  RViz... "
if command -v rviz2 &> /dev/null; then
    echo -e "${PASS}"
else
    echo -e "${FAIL} Not installed"
    echo "    Install: sudo apt install ros-humble-rviz2"
fi

# Check Python packages
echo -e "\n${BLUE}Checking Python dependencies:${NC}"

PYTHON_PACKAGES=("cv2:opencv-python" "ultralytics:ultralytics" "vosk:vosk")
for pkg_info in "${PYTHON_PACKAGES[@]}"; do
    IFS=':' read -r import_name pip_name <<< "$pkg_info"
    echo -n "  $pip_name... "
    if python3 -c "import $import_name" 2>/dev/null; then
        echo -e "${PASS}"
    else
        echo -e "${WARN} Not installed"
        echo "    Install: pip3 install $pip_name"
    fi
done

# Check YOLO model
echo -e "\n${BLUE}Checking YOLO model:${NC}"
echo -n "  yolov8n.pt... "
if [ -f "yolov8n.pt" ]; then
    echo -e "${PASS} Found in workspace"
elif [ -f "$HOME/.cache/ultralytics/yolov8n.pt" ]; then
    echo -e "${PASS} Found in cache"
else
    echo -e "${WARN} Not found"
    echo "    Will be downloaded automatically on first run"
fi

# Check Vosk model
echo -n "  Vosk model... "
VOSK_MODEL=$(ls -d $HOME/vosk-model-* 2>/dev/null | head -n 1 || true)
if [ ! -z "$VOSK_MODEL" ]; then
    echo -e "${PASS} Found: $(basename $VOSK_MODEL)"
else
    echo -e "${WARN} Not found"
    echo "    Download from: https://alphacephei.com/vosk/models"
fi

# Summary
echo -e "\n${BLUE}=========================================="
echo "  Summary"
echo -e "==========================================${NC}"

if [ ${#MISSING_PACKAGES[@]} -eq 0 ]; then
    echo -e "${PASS} All packages installed"
else
    echo -e "${FAIL} Missing ${#MISSING_PACKAGES[@]} package(s)"
fi

if [ -z "$SERIAL_PORTS" ]; then
    echo -e "${WARN} Serial port not detected (needed for real robot)"
else
    echo -e "${PASS} Serial port available"
fi

if [ -z "$CAMERAS" ]; then
    echo -e "${WARN} Camera not detected (needed for perception)"
else
    echo -e "${PASS} Camera available"
fi

echo -e "\n${BLUE}Ready to launch!${NC}"
echo "  Simulation: ros2 launch arm_bringup sim_bringup.launch.py"
echo "  Real robot: ros2 launch arm_bringup real_bringup.launch.py"
echo "  Quick menu: ros2 run arm_bringup quick_launch.sh"
echo ""
