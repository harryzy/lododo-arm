#!/usr/bin/env bash
# Setup script for planar perception virtual environment
# This creates an isolated Python environment with OpenCV GUI support

set -e

VENV_PATH="/home/hurry/planar_venv"

echo "========================================"
echo "  Planar Perception Environment Setup"
echo "========================================"
echo ""

# Check if venv already exists
if [ -d "$VENV_PATH" ]; then
  read -p "Virtual environment already exists at $VENV_PATH. Recreate? (y/N) " -n 1 -r
  echo
  if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Removing existing virtual environment..."
    rm -rf "$VENV_PATH"
  else
    echo "Using existing virtual environment."
    exit 0
  fi
fi

# Create virtual environment
echo "[1/5] Creating virtual environment at $VENV_PATH..."
python3 -m venv "$VENV_PATH"

# Activate virtual environment
echo "[2/5] Activating virtual environment..."
source "$VENV_PATH/bin/activate"

# Upgrade pip
echo "[3/5] Upgrading pip..."
pip install --upgrade pip

# Install dependencies
echo "[4/5] Installing Python packages..."
echo "  - Installing numpy (1.x for compatibility)..."
pip install "numpy<2.0"

echo "  - Installing OpenCV with GUI support (from system)..."
# Use system OpenCV which has GTK support compiled in
pip install opencv-python==4.5.4.60 || {
  echo "Warning: Failed to install specific opencv-python version."
  echo "Installing system python3-opencv is recommended:"
  echo "  sudo apt install python3-opencv"
  echo "Then create a symlink in the venv:"
  SYSTEM_CV2=$(python3 -c "import cv2, os; print(os.path.dirname(cv2.__file__))" 2>/dev/null || echo "")
  if [ -n "$SYSTEM_CV2" ]; then
    echo "  ln -sf $SYSTEM_CV2 $VENV_PATH/lib/python*/site-packages/"
  fi
}

echo "  - Installing ROS2 dependencies..."
pip install pyyaml

echo "  - Installing camera_calibration in venv..."
pip install camera-calibration

# Create a wrapper to use system OpenCV if pip version fails
echo "[5/5] Configuring environment..."
cat > "$VENV_PATH/use_system_opencv.sh" << 'EOF'
#!/bin/bash
# This script links system OpenCV into the virtual environment
SYSTEM_CV2_PATH=$(python3 -c "import sys; sys.path.insert(0, '/usr/lib/python3/dist-packages'); import cv2, os; print(os.path.dirname(cv2.__file__))" 2>/dev/null)
VENV_SITE_PACKAGES=$(python -c "import site; print(site.getsitepackages()[0])")

if [ -n "$SYSTEM_CV2_PATH" ] && [ -d "$SYSTEM_CV2_PATH" ]; then
  echo "Linking system OpenCV from $SYSTEM_CV2_PATH"
  ln -sf "$SYSTEM_CV2_PATH" "$VENV_SITE_PACKAGES/cv2"
  echo "Done! System OpenCV (with GTK support) is now available in the venv."
else
  echo "Error: Could not find system OpenCV installation."
  echo "Please install: sudo apt install python3-opencv"
  exit 1
fi
EOF
chmod +x "$VENV_PATH/use_system_opencv.sh"

# Test OpenCV installation
echo ""
echo "Testing OpenCV installation..."
if python -c "import cv2; print(f'OpenCV {cv2.__version__} loaded successfully')" 2>/dev/null; then
  echo "✓ OpenCV is working"
  
  # Check GTK support
  HAS_GTK=$(python -c "import cv2; info = cv2.getBuildInformation(); print('YES' if 'GTK' in info and 'YES' in info else 'NO')" 2>/dev/null || echo "NO")
  if [ "$HAS_GTK" = "YES" ]; then
    echo "✓ GTK support detected (GUI windows will work)"
  else
    echo "⚠ GTK support not detected in pip OpenCV"
    echo "  Run the following to use system OpenCV with GTK:"
    echo "    source $VENV_PATH/bin/activate"
    echo "    $VENV_PATH/use_system_opencv.sh"
  fi
else
  echo "⚠ OpenCV not working. Trying system OpenCV fallback..."
  "$VENV_PATH/use_system_opencv.sh"
fi

echo ""
echo "========================================"
echo "  Setup Complete!"
echo "========================================"
echo ""
echo "Virtual environment created at: $VENV_PATH"
echo ""
echo "To activate manually:"
echo "  source $VENV_PATH/bin/activate"
echo ""
echo "To run camera calibration:"
echo "  source $VENV_PATH/bin/activate"
echo "  ros2 run camera_calibration cameracalibrator \\"
echo "    --size 6x8 --square 0.025 \\"
echo "    --ros-args -r image:=/camera/image_raw"
echo ""
