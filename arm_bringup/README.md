# Arm Bringup Package

The `arm_bringup` package provides a collection of launch files to easily start the robotic arm system in various configurations. This package serves as the central entry point for launching the complete arm system or individual subsystems.

## 📦 系统依赖

### 必须安装的包

**1. v4l2_camera (摄像头驱动)**
```bash
sudo apt install ros-humble-v4l2-camera
```
> ⚠️ **重要:** 用于真实摄像头。默认的 usb_cam 0.8.1 版本在 MJPEG 解码时存在严重 bug，会导致 RViz 显示异常或 segfault。v4l2_camera 是稳定的替代方案。
>
> 详细配置请参考: [CAMERA_SUCCESS.md](../../CAMERA_SUCCESS.md)

**2. YOLO 虚拟环境**
```bash
# 参见 YOLO_SETUP.md 获取详细安装步骤
ros2 run arm_bringup verify_yolo_install.sh
```

## ⚠️ Important: YOLO Perception Node

**The YOLO perception node MUST be started separately** due to dependency conflicts between Ultralytics YOLO and MoveIt packages. 

**Quick Start:**
```bash
# Terminal 1: Main system
ros2 launch arm_bringup sim_bringup.launch.py

# Terminal 2: YOLO perception (in separate virtual environment)
ros2 run arm_bringup start_yolo_node.sh
```

See [YOLO_SETUP.md](YOLO_SETUP.md) for detailed setup instructions.

## 📦 Package Overview

This bringup package integrates all the components of the arm system:
- **arm_description**: Robot URDF models
- **arm_driver_node**: Hardware driver for real robot
- **arm_moveit_config**: MoveIt motion planning configuration
- **arm_perception_yolo**: YOLO-based object detection
- **arm_planning_py**: High-level planning and grasping logic
- **arm_voice_interface**: Voice control interface
- **arm_rviz_plugin**: Custom RViz control panel

## 🚀 Launch Files

### 1. **sim_bringup.launch.py** - Full Simulation System
Launches the complete arm system in Gazebo simulation with all features enabled.

**What it includes:**
- Gazebo simulation environment
- Robot state publisher
- MoveIt move_group
- RViz with custom control panel
- Arm planning interface
- Voice control (optional)

**Note:** YOLO perception node must be started separately (see above)

**Usage:**
```bash
# Full simulation with all features
ros2 launch arm_bringup sim_bringup.launch.py

# Without voice control
ros2 launch arm_bringup sim_bringup.launch.py use_voice_control:=false

# Without perception
ros2 launch arm_bringup sim_bringup.launch.py use_perception:=false

# Headless (no RViz)
ros2 launch arm_bringup sim_bringup.launch.py use_rviz:=false

# Custom log level
ros2 launch arm_bringup sim_bringup.launch.py log_level:=debug
```

**Launch Arguments:**
- `use_voice_control` (default: `true`) - Enable voice control interface
- `use_perception` (default: `true`) - Enable YOLO perception system
- `use_rviz` (default: `true`) - Launch RViz visualization
- `log_level` (default: `info`) - Logging level (debug, info, warn, error, fatal)

---

### 2. **real_bringup.launch.py** - Real Robot System
Launches the complete arm system with physical robot hardware.

**What it includes:**
- Real robot driver (serial communication)
- Robot state publisher
- MoveIt move_group
- RViz with custom control panel
- YOLO perception node with camera
- Arm planning interface
- Voice control (optional)

**Usage:**
```bash
# Full system with default serial port
ros2 launch arm_bringup real_bringup.launch.py

# Custom serial port and baud rate
ros2 launch arm_bringup real_bringup.launch.py serial_port:=/dev/ttyACM0 baud_rate:=115200

# With custom camera device
ros2 launch arm_bringup real_bringup.launch.py camera_device:=/dev/video2

# Without voice control
ros2 launch arm_bringup real_bringup.launch.py use_voice_control:=false
```

**Launch Arguments:**
- `serial_port` (default: `/dev/ttyUSB0`) - Serial port for robot communication
- `baud_rate` (default: `115200`) - Baud rate for serial communication
- `use_voice_control` (default: `true`) - Enable voice control interface
- `use_perception` (default: `true`) - Enable YOLO perception system
- `use_rviz` (default: `true`) - Launch RViz visualization
- `camera_device` (default: `/dev/video0`) - Camera device for perception
- `log_level` (default: `info`) - Logging level

---

### 3. **voice_headless.launch.py** - Headless Voice Control
Launches the arm system for voice-only control without any GUI (ideal for embedded systems).

**What it includes:**
- Real robot driver or simulation
- Robot state publisher
- MoveIt move_group (no RViz)
- Arm planning interface
- Voice control interface
- Perception (optional, no visualization)

**Usage:**
```bash
# Headless mode with voice control
ros2 launch arm_bringup voice_headless.launch.py

# With simulation instead of real hardware
ros2 launch arm_bringup voice_headless.launch.py use_sim:=true

# Without perception
ros2 launch arm_bringup voice_headless.launch.py use_perception:=false
```

---

### 6. **robot_side.launch.py** - Distributed Robot Side (Raspberry Pi)
Launches hardware drivers on Raspberry Pi for distributed deployment.

**What it includes:**
- Arm driver node (servo control)
- Robot state publisher
- RealSense camera driver

**Typical Deployment:** Runs on Raspberry Pi connected to robot hardware

**Usage:**
```bash
# Standard startup on Raspberry Pi
ros2 launch arm_bringup robot_side.launch.py

# Custom serial port
ros2 launch arm_bringup robot_side.launch.py port:=/dev/ttyUSB0

# Without camera
ros2 launch arm_bringup robot_side.launch.py enable_camera:=false

# Specific camera serial number
ros2 launch arm_bringup robot_side.launch.py camera_serial_no:="XXXXX"

# Custom ROS domain (must match PC side)
ros2 launch arm_bringup robot_side.launch.py ros_domain_id:=42
```

**Launch Arguments:**
- `port` (default: `/dev/ttyUSB0`) - Serial port for servo communication
- `enable_camera` (default: `true`) - Enable RealSense camera
- `camera_serial_no` (default: `""`) - Camera serial number (empty = first available)
- `ros_domain_id` (default: `0`) - ROS domain ID for DDS communication

---

### 7. **pc_side.launch.py** - Distributed Control Side (PC)
Launches planning and perception on PC for distributed deployment.

**What it includes:**
- MoveIt move_group (motion planning)
- YOLO perception node
- RViz visualization (optional)
- Arm planning interface
- Voice control (optional)

**Typical Deployment:** Runs on PC workstation

**Usage:**
```bash
# Standard startup with RViz
ros2 launch arm_bringup pc_side.launch.py

# Without RViz (headless)
ros2 launch arm_bringup pc_side.launch.py rviz:=false

# With voice control
ros2 launch arm_bringup pc_side.launch.py voice_control:=true

# Custom ROS domain (must match robot side)
ros2 launch arm_bringup pc_side.launch.py ros_domain_id:=42
```

**Launch Arguments:**
- `rviz` (default: `true`) - Launch RViz visualization
- `voice_control` (default: `false`) - Enable voice control interface
- `ros_domain_id` (default: `0`) - ROS domain ID for DDS communication

**⚠️ Important: YOLO Perception Node**

The YOLO perception node must be started **separately** in another terminal due to its isolated virtual environment requirement:

```bash
# Terminal 2: Start YOLO perception (after pc_side.launch.py is running)

# Option 1 (Recommended): Cube detection for reliable grasping
ros2 run arm_bringup start_yolo_cube_detect.sh

# Option 2 (Advanced): Dual-view triangulation measurement
# ros2 run arm_bringup start_yolo_dual_view.sh

# Option 3 (Basic): Single-view projection (legacy)
# ros2 run arm_bringup start_yolo_node.sh
```

**Recommendation Hierarchy:**
1. **start_yolo_cube_detect.sh** - Best for consistent grasping (optimized for cubes)
2. **start_yolo_dual_view.sh** - Advanced multi-view measurement system
3. **start_yolo_node.sh** - Basic single-view (requires camera tuning)

See [YOLO_SETUP.md](YOLO_SETUP.md) for detailed YOLO configuration.

See [YOLO_SETUP.md](YOLO_SETUP.md) for detailed YOLO configuration.

**🌐 Distributed Deployment Guide:**

For complete setup instructions, see:
- [Distributed Deployment Guide](docs/distributed_deployment_guide.md) - Full setup guide
- [Quick Reference](docs/distributed_deployment_quick_ref.md) - Command cheat sheet

Quick setup scripts:
```bash
# On Raspberry Pi
ros2 run arm_bringup setup_robot_side.sh

# On PC
ros2 run arm_bringup setup_pc_side.sh
```

**Launch Arguments:**
- `serial_port` (default: `/dev/ttyUSB0`) - Serial port for robot communication
- `baud_rate` (default: `115200`) - Baud rate for serial communication
- `use_perception` (default: `true`) - Enable perception (no visualization)
- `camera_device` (default: `/dev/video0`) - Camera device for perception
- `log_level` (default: `info`) - Logging level
- `use_sim` (default: `false`) - Use simulation instead of real hardware

**Voice Commands:**
- "scan front" - Scan the front area
- "scan all" - Perform a full scan
- "grasp" or "scan and grasp" - Scan and grasp objects

---

### 4. **perception_bringup.launch.py** - Perception System Only
Launches only the perception system for testing and development.

**What it includes:**
- YOLO detection node (via virtual environment script)
- Projection node for 3D pose estimation
- Automatic virtual environment setup

**Important:** This launch file automatically handles the virtual environment setup. The YOLO perception system runs in an isolated Python environment to avoid dependency conflicts with MoveIt.

**Usage:**
```bash
# Start perception with default settings (first time will setup venv)
ros2 launch arm_bringup perception_bringup.launch.py

# The script automatically:
# - Creates ~/yolo_venv if it doesn't exist
# - Installs CPU-only PyTorch and Ultralytics
# - Launches both yolo_detector and projection_node
```

**Note:** Parameters like `camera_device`, `model_path`, `confidence_threshold` are configured in the underlying `yolo_perception_launch.py`. If you need to customize these, edit:
```bash
src/arm_perception_yolo/launch/yolo_perception_launch.py
```

**Default Configuration:**
- Camera: `/dev/video0`
- Model: `yolov8m.pt` (medium model, good balance)
- Detection mode: `triggered` (waits for commands)
- Confidence threshold: `0.5`

---

### 5. **minimal_bringup.launch.py** - Minimal System (No GUI)
Launches only the essential components for basic arm control without visualization.

**What it includes:**
- Robot driver or simulation
- Robot state publisher
- Joint state publisher
- MoveIt move_group (background only)

**Features:**
- 🚀 Lightweight - no RViz by default
- � Ideal for headless operation or manual RViz launch
- ⚡ Fast startup time
- � Perfect for testing and development

**Usage:**
```bash
# Start with real hardware (no GUI)
ros2 launch arm_bringup minimal_bringup.launch.py

# Start with simulation (no GUI)
ros2 launch arm_bringup minimal_bringup.launch.py use_sim:=true

# Custom serial port
ros2 launch arm_bringup minimal_bringup.launch.py serial_port:=/dev/ttyACM0
```

**Note:** This launch file does NOT start RViz by default. For visualization, you can:
1. Use `sim_bringup.launch.py` or `real_bringup.launch.py` for full system with GUI
2. Manually launch RViz in a separate terminal:
   ```bash
   ros2 launch arm_moveit_config moveit_rviz.launch.py
   ```

**Launch Arguments:**
- `use_sim` (default: `false`) - Use simulation instead of real hardware
- `serial_port` (default: `/dev/ttyUSB0`) - Serial port for robot communication
- `baud_rate` (default: `115200`) - Baud rate for serial communication

---

## 🛠️ Installation

```bash
cd ~/lododo-arm
colcon build --packages-select arm_bringup
source install/setup.bash
```

## 📋 Quick Start Guide

### For Development & Testing (Simulation)
```bash
# Full simulation system
ros2 launch arm_bringup sim_bringup.launch.py
```

### For Real Robot Operation
```bash
# Make sure the robot is connected
ls /dev/ttyUSB*

# Launch the system
ros2 launch arm_bringup real_bringup.launch.py
```

### For Embedded/Headless Deployment
```bash
# Start voice control without GUI
ros2 launch arm_bringup voice_headless.launch.py
```

### For Testing Individual Subsystems
```bash
# Test perception only
ros2 launch arm_bringup perception_bringup.launch.py

# Test with simulation (for planning testing)
ros2 launch arm_bringup sim_bringup.launch.py
```

## 🔧 Common Use Cases

### 1. Development Workflow
Start with simulation for safe testing:
```bash
ros2 launch arm_bringup sim_bringup.launch.py
```

### 2. Hardware Testing
Use minimal bringup to verify hardware:
```bash
ros2 launch arm_bringup minimal_bringup.launch.py
```

### 3. Perception Calibration
Test camera and YOLO detection:
```bash
ros2 launch arm_bringup perception_bringup.launch.py
```

### 4. Planning Testing
Use simulation mode for motion planning tests:
```bash
ros2 launch arm_bringup sim_bringup.launch.py
```

### 5. Production Deployment
Full system on real robot:
```bash
ros2 launch arm_bringup real_bringup.launch.py
```

### 6. Embedded System
Headless voice control:
```bash
ros2 launch arm_bringup voice_headless.launch.py
```

## 📊 System Architecture

```
arm_bringup
├── sim_bringup.launch.py ──────► Gazebo + RViz + All Features (Planning Testing)
├── real_bringup.launch.py ─────► Hardware + RViz + All Features
├── voice_headless.launch.py ───► Hardware + Voice Only (No GUI)
├── perception_bringup.launch.py ► Perception Testing
└── minimal_bringup.launch.py ──► Basic Hardware Control
```

## 🐛 Troubleshooting

### Serial Port Permission Denied
```bash
sudo usermod -a -G dialout $USER
# Log out and log back in
```

### Camera Not Found
```bash
# List available cameras
v4l2-ctl --list-devices

# Use the correct device
ros2 launch arm_bringup real_bringup.launch.py camera_device:=/dev/video2
```

### Gazebo Not Starting
```bash
# Kill existing Gazebo processes
killall gzserver gzclient

# Restart
ros2 launch arm_bringup sim_bringup.launch.py
```

### Voice Recognition Not Working
```bash
# Test microphone
arecord -l

# Check Vosk model installation
ls ~/vosk-model-*
```

## 📚 Related Documentation

- [arm_description](../arm_description/README.md) - Robot models and URDF
- [arm_moveit_config](../arm_moveit_config/README.md) - MoveIt configuration
- [arm_perception_yolo](../arm_perception_yolo/README.md) - YOLO perception
- [arm_planning_py](../arm_planning_py/README.md) - Planning interface
- [arm_voice_interface](../arm_voice_interface/README.md) - Voice control
- [arm_rviz_plugin](../arm_rviz_plugin/README.md) - RViz control panel

## 📄 License

Apache-2.0 License

## 👥 Maintainers

- lododo <contect@lododo.org>

---

**Note**: This package follows ROS2 best practices for bringup packages, making it easy to launch complex robotic systems with a single command while maintaining flexibility through launch arguments.
