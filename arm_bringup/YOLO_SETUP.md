# YOLO Perception Setup Guide

> **💡 Important Update (2025-10-21)**  
> The setup script now installs **CPU-only** versions of PyTorch and dependencies to save disk space.
> - **Old (GPU)**: ~3.5 GB with CUDA libraries  
> - **New (CPU)**: ~500 MB without GPU libraries  
> - **No performance impact** for CPU inference!  
> 
> See [FIX_YOLO_CPU_ONLY.md](FIX_YOLO_CPU_ONLY.md) for details.

## Why Separate Virtual Environment?

The YOLO perception node uses Ultralytics YOLOv8, which has dependency conflicts with MoveIt and other ROS2 packages. To resolve this, we run the YOLO node in an isolated Python virtual environment.

## Quick Setup

### Automatic Setup (Recommended)

The easiest way is to let the launch script create the virtual environment automatically:

```bash
# First time: This will create the venv and install dependencies
ros2 run arm_bringup start_yolo_node.sh

# The script will:
# 1. Create virtual environment at ~/yolo_venv
# 2. Install opencv-python, ultralytics, numpy
# 3. Install ROS2 Python packages (rclpy, cv_bridge, etc.)
# 4. Launch the YOLO detection node
```

### Manual Setup

If you prefer to set up manually:

```bash
# 1. Create virtual environment
python3 -m venv ~/yolo_venv

# 2. Activate it
source ~/yolo_venv/bin/activate

# 3. Install dependencies (CPU-only version to save space)
pip install --upgrade pip

# Install PyTorch CPU version (smaller, no CUDA libraries)
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu

# Install YOLO and dependencies
pip install opencv-python-headless numpy
pip install ultralytics --no-deps
pip install PyYAML pillow scipy matplotlib requests psutil pandas

# Install ROS2 packages
pip install rclpy std_msgs sensor_msgs cv_bridge

# 4. Verify installation
python3 -c "import ultralytics; print('YOLO OK')"
python3 -c "import cv2; print('OpenCV OK')"
python3 -c "import torch; print(f'PyTorch: {torch.__version__} (CUDA: {torch.cuda.is_available()})')"
```

## Usage

### Starting with Main System

When you launch the main system, YOLO is NOT started automatically. You need to start it separately:

**Terminal 1: Main System**
```bash
cd ~/lododo-arm
source install/setup.bash
ros2 launch arm_bringup sim_bringup.launch.py
```

**Terminal 2: YOLO Perception**
```bash
cd ~/lododo-arm
source install/setup.bash
ros2 run arm_bringup start_yolo_node.sh
```

### Alternative: Manual Launch in Venv

```bash
cd ~/lododo-arm
source ~/yolo_venv/bin/activate
source install/setup.bash
ros2 run arm_perception_yolo yolo_detection_node
```

### With Parameters

```bash
# Custom camera device
ros2 run arm_bringup start_yolo_node.sh --ros-args -p camera_device:=/dev/video2

# Disable visualization (headless)
ros2 run arm_bringup start_yolo_node.sh --ros-args -p visualize:=false

# Custom confidence threshold
ros2 run arm_bringup start_yolo_node.sh --ros-args -p confidence_threshold:=0.7

# Multiple parameters
ros2 run arm_bringup start_yolo_node.sh --ros-args \
  -p camera_device:=/dev/video2 \
  -p confidence_threshold:=0.7 \
  -p visualize:=false
```

## Complete Workflow

### Simulation with YOLO

```bash
# Terminal 1: Main simulation system
ros2 launch arm_bringup sim_bringup.launch.py

# Terminal 2: YOLO perception
ros2 run arm_bringup start_yolo_node.sh
```

### Real Robot with YOLO

```bash
# Terminal 1: Real robot system
ros2 launch arm_bringup real_bringup.launch.py

# Terminal 2: YOLO perception with camera
ros2 run arm_bringup start_yolo_node.sh --ros-args -p camera_device:=/dev/video0
```

### Headless Voice Control with YOLO

```bash
# Terminal 1: Headless system
ros2 launch arm_bringup voice_headless.launch.py

# Terminal 2: YOLO perception (no visualization)
ros2 run arm_bringup start_yolo_node.sh --ros-args -p visualize:=false
```

## Verification

Check if YOLO node is running:
```bash
# List nodes
ros2 node list | grep yolo

# Check topics
ros2 topic list | grep detect

# See detections
ros2 topic echo /detected_objects
```

## Troubleshooting

### Virtual Environment Issues

**Problem**: Virtual environment creation fails
```bash
# Install venv if missing
sudo apt install python3-venv

# Recreate virtual environment
rm -rf ~/yolo_venv
ros2 run arm_bringup start_yolo_node.sh
```

**Problem**: Import errors in virtual environment
```bash
# Reinstall packages
source ~/yolo_venv/bin/activate
pip install --force-reinstall opencv-python ultralytics numpy rclpy
```

### Node Communication Issues

**Problem**: YOLO node can't communicate with other nodes
```bash
# Make sure both nodes are on the same ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# If different, set the same ID in both terminals
export ROS_DOMAIN_ID=0
```

**Problem**: Topics not visible
```bash
# Check if node is running
ros2 node list

# Check topic info
ros2 topic info /detected_objects
```

### Dependency Conflicts

**Problem**: Still getting import errors
```bash
# Use completely isolated environment
source ~/yolo_venv/bin/activate

# Do NOT source any other workspace that might have conflicting packages
# Only source ROS2 base:
source /opt/ros/humble/setup.bash
source ~/lododo-arm/install/setup.bash

# Launch node
ros2 run arm_perception_yolo yolo_detection_node
```

## Custom Virtual Environment Location

If you want to use a different location:

```bash
# Create venv at custom location
python3 -m venv /path/to/my/yolo_env

# Use with launch script
ros2 run arm_bringup start_yolo_node.sh /path/to/my/yolo_env
```

Or edit the script:
```bash
# In scripts/start_yolo_node.sh, change this line:
VENV_PATH="${HOME}/yolo_venv"
# To your preferred path:
VENV_PATH="/path/to/my/yolo_env"
```

## Testing Without Other Nodes

To test YOLO independently:

```bash
# Just start the YOLO node
source ~/yolo_venv/bin/activate
source ~/lododo-arm/install/setup.bash
ros2 run arm_perception_yolo yolo_detection_node

# In another terminal, check detection
ros2 topic echo /detected_objects
```

## Integration with Quick Launch

The quick launch script will prompt you to start YOLO separately:

```bash
ros2 run arm_bringup quick_launch.sh
# Select simulation or real robot
# Then in another terminal:
ros2 run arm_bringup start_yolo_node.sh
```

## Performance Notes

- Virtual environment adds minimal overhead
- YOLO node runs at same speed as without venv
- Memory isolation prevents dependency conflicts
- Can run on same or different machine (via ROS2 DDS)

## Summary

✅ **DO**: Start YOLO node separately in its own terminal with the provided script
✅ **DO**: Use `ros2 run arm_bringup start_yolo_node.sh` for easy launching
✅ **DO**: Keep the virtual environment for future use (don't delete it)

❌ **DON'T**: Try to launch YOLO in the same process as MoveIt
❌ **DON'T**: Install YOLO packages in the system Python or main ROS workspace
❌ **DON'T**: Source conflicting workspaces together

---

**Quick Reference:**
```bash
# Setup (first time only)
ros2 run arm_bringup start_yolo_node.sh

# Normal usage (Terminal 1)
ros2 launch arm_bringup sim_bringup.launch.py

# Normal usage (Terminal 2)
ros2 run arm_bringup start_yolo_node.sh
```
