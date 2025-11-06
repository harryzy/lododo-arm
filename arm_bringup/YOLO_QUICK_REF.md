# YOLO Perception - Quick Reference

## The Problem
YOLO (Ultralytics) has dependency conflicts with MoveIt packages. They cannot run in the same Python environment.

## The Solution
Run YOLO node in a separate virtual environment using the provided wrapper script.

## Quick Commands

### First Time Setup
```bash
# Automatic (recommended) - script will create venv
ros2 run arm_bringup start_yolo_node.sh

# Manual
python3 -m venv ~/yolo_venv
source ~/yolo_venv/bin/activate
pip install opencv-python ultralytics numpy rclpy std_msgs sensor_msgs cv_bridge
```

### Daily Usage

**Two Terminal Workflow:**

Terminal 1 - Main System:
```bash
cd ~/lododo-arm
source install/setup.bash
ros2 launch arm_bringup sim_bringup.launch.py
```

Terminal 2 - YOLO Perception:
```bash
cd ~/lododo-arm
source install/setup.bash
ros2 run arm_bringup start_yolo_node.sh
```

### Common Parameters

```bash
# Custom camera
ros2 run arm_bringup start_yolo_node.sh --ros-args -p camera_device:=/dev/video2

# No visualization (headless)
ros2 run arm_bringup start_yolo_node.sh --ros-args -p visualize:=false

# Higher confidence threshold
ros2 run arm_bringup start_yolo_node.sh --ros-args -p confidence_threshold:=0.7

# Combined
ros2 run arm_bringup start_yolo_node.sh --ros-args \
  -p camera_device:=/dev/video2 \
  -p visualize:=false \
  -p confidence_threshold:=0.7
```

## Verification

```bash
# Check if running
ros2 node list | grep yolo

# See detections
ros2 topic echo /detected_objects

# Check topic info
ros2 topic info /detected_objects
```

## Troubleshooting

```bash
# Recreate virtual environment
rm -rf ~/yolo_venv
ros2 run arm_bringup start_yolo_node.sh

# Manual launch (for debugging)
source ~/yolo_venv/bin/activate
source ~/lododo-arm/install/setup.bash
ros2 run arm_perception_yolo yolo_detection_node --ros-args --log-level debug
```

## Why This Approach?

✅ **Pros:**
- Isolates conflicting dependencies
- No modifications to system packages
- Easy to maintain and update
- Works reliably across different systems

❌ **Alternative approaches (not recommended):**
- Installing YOLO system-wide → conflicts with ROS packages
- Using Docker → adds complexity, networking issues
- Modifying package dependencies → breaks reproducibility

## Integration Pattern

This is a common pattern in ROS2 for handling Python package conflicts:
1. Create isolated virtual environment
2. Install conflicting packages there
3. Launch node through wrapper script
4. Communicate via ROS2 topics (DDS)

Similar pattern used for:
- TensorFlow/PyTorch nodes
- Custom ML models
- Legacy Python 2 nodes

---

**Remember:** Always start YOLO node separately, never try to include it in main launch files!
