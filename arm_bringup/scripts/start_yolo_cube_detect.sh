#!/bin/bash
# Cube detection specialized startup script
# Enable cube detection mode, only returns the most cube-like object

echo "========================================"
echo "  🎲 Cube Detection Mode Startup"
echo "========================================"
echo ""
echo "📋 Configuration:"
echo "  • Cube detection: enabled"
echo "  • thickness_ratio: 1.0 (cube)"
echo "  • bbox aspect ratio tolerance: 0.5"
echo "  • 3D dimension tolerance: 0.4"
echo ""
echo "🎯 Will only return 1 most cube-like object"
echo "========================================"
echo ""

# Source ROS2 workspace
source install/setup.bash

# Launch YOLO tri-view detection (with cube detection mode)
ros2 launch arm_perception_yolo yolo_perception_launch.py \
  model_path:=yolov8m.pt \
  min_conf:=0.15 \
  detection_mode:=triggered \
  baseline:=0.20 \
  detect_cube_only:=True \
  cube_aspect_ratio_tolerance:=0.5 \
  cube_dimension_tolerance:=0.4
