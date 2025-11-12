#!/bin/bash
# Cube detection specialized startup script
# Simplified: Only tells launch file to enable cube detection
# All parameters are managed in measurement_params.yaml and yolo_perception_launch.py

echo "========================================"
echo "  🎲 Cube Detection Mode Startup"
echo "========================================"
echo ""
echo "📋 Configuration:"
echo "  • Mode: Cube detection enabled"
echo "  • Parameters: from measurement_params.yaml"
echo "  • Launch file: yolo_perception_launch.py"
echo ""
echo "🎯 Will only return 1 most cube-like object"
echo ""
echo "💡 To override parameters, pass launch arguments:"
echo "   Example: baseline:=0.20 min_conf:=0.15"
echo "========================================"
echo ""

# Source ROS2 workspace
source install/setup.bash

# Launch YOLO perception with cube detection mode
# All parameters are configured in measurement_params.yaml
# Pass any additional arguments to the launch file (e.g., baseline:=0.20)
ros2 launch arm_perception_yolo yolo_perception_launch.py \
  detect_cube_only:=True \
  "$@"
