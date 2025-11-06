#!/bin/bash
# Camera parameter optimization script - For top-down shooting overexposure issue
# Usage scenario: Robotic arm gripper shoots desktop from top-down angle, light causes object top surface overexposure

echo "========================================"
echo "  Camera Parameter Optimization - Top View Overexposure Prevention"
echo "========================================"

# Detect camera device
CAMERA_DEV=$(v4l2-ctl --list-devices | grep -A1 "usb" | tail -1 | xargs)
if [ -z "$CAMERA_DEV" ]; then
    CAMERA_DEV="/dev/video0"
fi

echo "Using camera device: $CAMERA_DEV"
echo ""

# Get current settings
echo "📊 Current camera parameters:"
v4l2-ctl -d $CAMERA_DEV --get-ctrl=brightness
v4l2-ctl -d $CAMERA_DEV --get-ctrl=contrast
v4l2-ctl -d $CAMERA_DEV --get-ctrl=hue
echo ""

# Optimization parameters for top-down overexposure
echo "🔧 Applying optimization parameters (prevent overexposure + enhance red detection)..."

# 1. Significantly reduce brightness to prevent top surface overexposure
echo "1. ⬇️  Reducing brightness to 10 (prevent overexposure, original value may be too high)"
v4l2-ctl -d $CAMERA_DEV --set-ctrl=brightness=10

# 2. Increase contrast to enhance object edges and texture
echo "2. ⬆️  Increasing contrast to 120 (enhance edge detection)"
v4l2-ctl -d $CAMERA_DEV --set-ctrl=contrast=120

# 3. Adjust hue to optimize red object recognition
echo "3. 🎨 Adjusting hue to 0 (optimize red detection)"
v4l2-ctl -d $CAMERA_DEV --set-ctrl=hue=0

echo ""
echo "✅ Camera parameters optimized (for top-down shooting)"
echo ""
echo "📊 Optimized parameters:"
v4l2-ctl -d $CAMERA_DEV --get-ctrl=brightness
v4l2-ctl -d $CAMERA_DEV --get-ctrl=contrast
v4l2-ctl -d $CAMERA_DEV --get-ctrl=hue
echo ""
echo "💡 Fine-tuning suggestions (if effect still not ideal):"
echo "  [Darker] Reduce brightness: v4l2-ctl --set-ctrl=brightness=20"
echo "  [Very Dark] Very low brightness: v4l2-ctl --set-ctrl=brightness=10"
echo "  [Stronger Contrast] Increase contrast: v4l2-ctl --set-ctrl=contrast=220"
echo "  [Restore Default] v4l2-ctl --set-ctrl=brightness=16,contrast=16,hue=16"
echo ""
echo "🎯 Best practices for red 5cm cube:"
echo "  1. First test current setting (brightness=30)"
echo "  2. If still overexposed, gradually reduce to 20 → 15 → 10"
echo "  3. If too dark and unclear, increase to 40 → 50"
echo "  4. Ensure ambient light not too strong, avoid top surface reflection"
echo ""
echo "📹 Verification method:"
echo "  ros2 topic echo /camera/image_raw --once  # Check if image is normal"
echo "  or use rqt_image_view to view live image"
