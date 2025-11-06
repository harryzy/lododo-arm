# Quick Reference Guide

## Common Launch Commands

### 🎮 Full Systems

```bash
# Simulation with all features
ros2 launch arm_bringup sim_bringup.launch.py

# Real robot with all features  
ros2 launch arm_bringup real_bringup.launch.py

# Voice-only headless mode
ros2 launch arm_bringup voice_headless.launch.py
```

### 🔧 Subsystems

```bash
# Perception testing only
ros2 launch arm_bringup perception_bringup.launch.py

# Motion planning only
ros2 launch arm_bringup planning_bringup.launch.py

# Minimal hardware testing
ros2 launch arm_bringup minimal_bringup.launch.py
```

### 🛠️ Utility Scripts

```bash
# Interactive launch menu
ros2 run arm_bringup quick_launch.sh

# System health check
ros2 run arm_bringup health_check.sh
```

## Common Launch Arguments

### Serial Communication
```bash
serial_port:=/dev/ttyUSB0    # Robot serial port
baud_rate:=115200             # Communication speed
```

### Features Toggle
```bash
use_voice_control:=true/false  # Enable voice control
use_perception:=true/false     # Enable vision system
use_rviz:=true/false          # Show RViz GUI
```

### Camera Configuration
```bash
camera_device:=/dev/video0         # Camera device
confidence_threshold:=0.5          # YOLO confidence
visualize:=true/false             # Show detection window
```

### Logging
```bash
log_level:=info      # Options: debug, info, warn, error, fatal
```

## Quick Start Examples

### Example 1: Development in Simulation
```bash
# Start simulation with debug logging
ros2 launch arm_bringup sim_bringup.launch.py log_level:=debug

# In another terminal, send test command
ros2 topic pub /arm_command std_msgs/String "data: 'scan_front'" --once
```

### Example 2: Real Robot Setup
```bash
# Check system health first
ros2 run arm_bringup health_check.sh

# Start with custom serial port
ros2 launch arm_bringup real_bringup.launch.py serial_port:=/dev/ttyACM0

# Or use quick launch menu
ros2 run arm_bringup quick_launch.sh
```

### Example 3: Headless Deployment
```bash
# Start voice control without GUI
ros2 launch arm_bringup voice_headless.launch.py

# Test voice commands:
# - "scan front"
# - "scan all"  
# - "grasp"
```

### Example 4: Perception Testing
```bash
# Start perception with custom camera
ros2 launch arm_bringup perception_bringup.launch.py \
  camera_device:=/dev/video2 \
  confidence_threshold:=0.7 \
  model_path:=yolov8s.pt

# Check detected objects
ros2 topic echo /detected_objects
```

## Monitoring Commands

```bash
# List all active nodes
ros2 node list

# Check topic data flow
ros2 topic list
ros2 topic hz /arm_command
ros2 topic echo /arm_command_result

# Monitor TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Check service availability  
ros2 service list | grep moveit
```

## Troubleshooting Commands

```bash
# Kill all Gazebo processes
killall gzserver gzclient

# Check serial port permissions
ls -l /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0   # Temporary fix
sudo usermod -a -G dialout $USER  # Permanent fix (logout required)

# List cameras
v4l2-ctl --list-devices

# Test microphone
arecord -l
arecord -d 3 test.wav
aplay test.wav
```

## Configuration Files

Default parameters: `install/arm_bringup/share/arm_bringup/config/default_params.yaml`

Override in launch:
```bash
ros2 launch arm_bringup real_bringup.launch.py \
  serial_port:=/dev/ttyUSB0 \
  camera_device:=/dev/video0
```

## Topics Reference

| Topic | Type | Description |
|-------|------|-------------|
| `/arm_command` | std_msgs/String | Send commands (scan_front, scan_all, scan_and_grasp) |
| `/arm_command_result` | std_msgs/String | Receive execution results |
| `/detected_objects` | vision_msgs/Detection2DArray | Object detection results |
| `/joint_states` | sensor_msgs/JointState | Robot joint positions |
| `/camera/image_raw` | sensor_msgs/Image | Camera feed |

## Package Structure

```
arm_bringup/
├── launch/                    # Launch files
│   ├── sim_bringup.launch.py           # Full simulation
│   ├── real_bringup.launch.py          # Real robot
│   ├── voice_headless.launch.py        # Headless voice
│   ├── perception_bringup.launch.py    # Perception only
│   ├── planning_bringup.launch.py      # Planning only
│   └── minimal_bringup.launch.py       # Minimal system
├── config/                    # Configuration files
│   └── default_params.yaml
├── scripts/                   # Utility scripts
│   ├── quick_launch.sh       # Interactive menu
│   └── health_check.sh       # System diagnostics
├── README.md                  # Full documentation
├── ARCHITECTURE.md            # System architecture
└── QUICK_REFERENCE.md         # This file
```

## Build & Install

```bash
# Build this package only
colcon build --packages-select arm_bringup

# Build all packages
colcon build

# Source workspace
source install/setup.bash
```

## Getting Help

- Full documentation: See [README.md](README.md)
- System architecture: See [ARCHITECTURE.md](ARCHITECTURE.md)
- Issues: Check `/arm_command_result` topic for error messages
- Health check: Run `ros2 run arm_bringup health_check.sh`

---

**Pro Tip**: Use tab completion! Type `ros2 launch arm_bringup <TAB>` to see all available launch files.
