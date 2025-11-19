#!/usr/bin/env bash
# Wrapper script to run camera calibration with proper Python environment
# This ensures numpy compatibility between ROS2 and OpenCV

set -e

VENV="/home/hurry/planar_venv"

echo "=========================================="
echo "  Camera Calibration Launcher"
echo "=========================================="
echo ""

# Check if venv exists
if [ ! -d "$VENV" ]; then
  echo "❌ Virtual environment not found at $VENV"
  echo "Please run: bash scripts/setup_planar_venv.sh"
  exit 1
fi

# Activate venv and source ROS2
source "$VENV/bin/activate"
source ~/lododo/install/setup.bash

# Link system OpenCV if not already done
if ! python -c "import cv2; assert 'GTK' in cv2.getBuildInformation()" 2>/dev/null; then
  echo "⚠️  GTK support not detected. Linking system OpenCV..."
  bash "$VENV/use_system_opencv.sh"
fi

echo "✓ Virtual environment: $VENV"
echo "✓ Python: $(which python)"
echo "✓ NumPy version: $(python -c 'import numpy; print(numpy.__version__)')"
echo "✓ OpenCV version: $(python -c 'import cv2; print(cv2.__version__)')"
echo ""

# Check if camera topic is available
echo "Checking camera topics..."
if ! ros2 topic list 2>/dev/null | grep -q "/camera/image_raw"; then
  echo "⚠️  Warning: /camera/image_raw topic not found"
  echo "   Please start camera node first:"
  echo "   ros2 launch arm_bringup pc_side.launch.py"
  echo ""
  read -p "Continue anyway? (y/N) " -n 1 -r
  echo
  if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit 1
  fi
fi

# Default parameters
SIZE="${SIZE:-6x8}"
SQUARE="${SQUARE:-0.025}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/camera/image_raw}"

echo "=========================================="
echo "  Starting Camera Calibration"
echo "=========================================="
echo "Calibration board: $SIZE inner corners"
echo "Square size: $SQUARE meters"
echo "Image topic: $IMAGE_TOPIC"
echo ""
echo "Instructions:"
echo "1. Show the calibration board to the camera"
echo "2. Move it slowly in X, Y, Size, and Skew directions"
echo "3. Wait for all 4 progress bars to turn GREEN"
echo "4. Click CALIBRATE button"
echo "5. Click SAVE button after calibration completes"
echo ""
echo "Launching calibration tool..."
echo ""

# Use ROS2's cameracalibrator but with venv's Python environment
# The key is to ensure PYTHONPATH doesn't include conflicting numpy
export PYTHONPATH="$VENV/lib/python3.10/site-packages:/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH"

exec ros2 run camera_calibration cameracalibrator \
  --size "$SIZE" \
  --square "$SQUARE" \
  --ros-args -r image:="$IMAGE_TOPIC" -r camera:=/camera
