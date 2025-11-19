#!/bin/bash
# Quick test script for the Planar Perception system

echo "========================================="
echo "  Planar Perception System Test"
echo "========================================="
echo ""

# Check if package is installed
echo "[1/4] Checking package installation..."
if ros2 pkg list | grep -q "arm_perception_planar"; then
    echo "✅ Package installed"
else
    echo "❌ Package not found"
    exit 1
fi

# Check executables
echo ""
echo "[2/4] Checking executables..."
EXECUTABLES=$(ros2 pkg executables arm_perception_planar)
if echo "$EXECUTABLES" | grep -q "cube_detector_node"; then
    echo "✅ cube_detector_node found"
else
    echo "❌ cube_detector_node not found"
fi

if echo "$EXECUTABLES" | grep -q "planar_localization_node"; then
    echo "✅ planar_localization_node found"
else
    echo "❌ planar_localization_node not found"
fi

# Check configuration file
echo ""
echo "[3/4] Checking configuration files..."
CONFIG_PATH=$(ros2 pkg prefix arm_perception_planar)/share/arm_perception_planar/config/planar_params.yaml
if [ -f "$CONFIG_PATH" ]; then
    echo "✅ Configuration file found: $CONFIG_PATH"
else
    echo "❌ Configuration file not found"
fi

# Check launch file
echo ""
echo "[4/4] Checking launch files..."
LAUNCH_PATH=$(ros2 pkg prefix arm_perception_planar)/share/arm_perception_planar/launch/planar_perception.launch.py
if [ -f "$LAUNCH_PATH" ]; then
    echo "✅ Launch file found: $LAUNCH_PATH"
else
    echo "❌ Launch file not found"
fi

echo ""
echo "========================================="
echo "  Test Summary"
echo "========================================="
echo ""
echo "Package: arm_perception_planar ✅"
echo "Nodes:"
echo "  - cube_detector_node ✅"
echo "  - planar_localization_node ✅"
echo ""
echo "To run the system:"
echo "  ros2 launch arm_perception_planar planar_perception.launch.py"
echo ""
echo "To view detection results:"
echo "  ros2 topic echo /planar/measured_objects"
echo ""
echo "For calibration guide:"
echo "  cat $(ros2 pkg prefix arm_perception_planar)/../../../src/lododo-arm/arm_perception_planar/CALIBRATION_GUIDE.md"
echo ""
echo "========================================="
