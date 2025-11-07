<div align="center">

# 🦾 Lododo Arm

**Complete ROS2 Humble Robotic Arm Control System**

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-green.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-blue.svg)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C++-17-orange.svg)](https://isocpp.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)](https://ubuntu.com/)

[![MoveIt2](https://img.shields.io/badge/MoveIt2-Enabled-purple.svg)](https://moveit.ros.org/)
[![YOLO](https://img.shields.io/badge/YOLO-v8-yellow.svg)](https://github.com/ultralytics/ultralytics)
[![Distributed](https://img.shields.io/badge/Deployment-Distributed-red.svg)](docs/distributed_deployment.md)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)]()

[Features](#-features) • [Quick Start](#-quick-start) • [Documentation](#-documentation) • [Contributing](#-contributing) • [License](#-license)

---

</div>

An intelligent robotic arm system integrating computer vision, motion planning, and voice control based on ROS2 Humble. Supports distributed deployment with Raspberry Pi for hardware control and PC for planning/perception.

## ✨ Features

### 🤖 Hardware Control
- **6-DOF Robotic Arm** - Full motion control using Feetech ST3215 servos
- **Gripper Control** - Precision grasping with rotation support
- **RealSense D435i** - Depth camera for 3D perception
- **Distributed Deployment** - Raspberry Pi + PC architecture

### 🎯 Motion Planning
- **MoveIt2 Integration** - Professional motion planning framework
- **Collision Avoidance** - Safe trajectory generation
- **Inverse Kinematics** - Automatic joint angle calculation
- **Custom Move Groups** - Flexible control configurations

### 👁️ Computer Vision
- **YOLOv8 Detection** - Real-time object detection
- **Cube Detection Mode** - Optimized for cube grasping tasks
- **Dual-View Triangulation** - Enhanced 3D position estimation
- **Depth Integration** - Precise spatial localization

### 🗣️ Voice Control (Optional)
- **Natural Language Commands** - Vosk-based speech recognition
- **Chinese/English Support** - Multi-language interface
- **Custom Command Mapping** - Configurable voice actions

### 📊 Visualization & Control
- **Custom RViz Plugin** - Interactive control panel
- **Real-time Monitoring** - System status display
- **Debug Tools** - Comprehensive logging and diagnostics

## 🏗️ System Architecture

```
┌─────────────────────────────────┐         ┌──────────────────────────────────┐
│   Raspberry Pi (Robot Side)     │         │    PC (Control Side)             │
│  ───────────────────────────────  │         │  ────────────────────────────────  │
│  • arm_driver_node              │         │  • move_group (MoveIt2)          │
│  • robot_state_publisher        │ <──DDS──> │  • arm_planning_py_node          │
│  • realsense2_camera            │  Network  │  • yolo_perception_node          │
│  • joint_state_publisher        │         │  • rviz2                         │
│                                 │         │  • arm_voice_node (optional)     │
└─────────────────────────────────┘         └──────────────────────────────────┘
```

### 📦 Package Structure

```
├── arm_bringup          # System integration & launch files
├── arm_description      # URDF robot models & kinematics
├── arm_driver_node      # Feetech servo hardware driver (C++)
├── arm_interfaces       # Custom ROS2 messages & services
├── arm_moveit_config    # MoveIt2 motion planning config
├── arm_perception_yolo  # YOLOv8 vision perception system
├── arm_planning_py      # Motion planning & grasping logic (Python)
├── arm_rviz_plugin      # Custom RViz control panel (C++)
└── arm_voice_interface  # Voice command interface (Python)
```

## 📋 Prerequisites

### 🔧 Hardware Requirements

**Minimum Configuration:**
- 6-DOF robotic arm with Feetech ST3215 servos
- Intel RealSense D435i depth camera (or compatible USB camera)
- Ubuntu 22.04 LTS
- 8GB RAM, 4-core CPU

**Recommended Configuration (Distributed):**
- Raspberry Pi 4 (4GB+) - Robot side hardware control
- PC with Ubuntu 22.04 - Planning and perception
- Same LAN network connection

### 💻 Software Requirements

- **ROS2 Humble** - Robot Operating System 2
- **Python 3.10+** - Main programming language
- **MoveIt2** - Motion planning framework
- **PyTorch** - Deep learning (YOLO perception)
- **OpenCV** - Computer vision
- **Ultralytics** - YOLOv8 implementation

## 🚀 Quick Start

### Step 1: Clone Repository

```bash
# Create ROS2 workspace
mkdir -p ~/lododo-arm/src
cd ~/lododo-arm/src

# Clone repository (note the . at the end)
git clone https://github.com/harryzy/lododo-arm.git .
cd ~/lododo-arm
```

### Step 2: Install Dependencies
- [Install ROS2 humble](https://docs.ros.org/en/humble/#)
```bash
# Install ROS2 packages
sudo apt update
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    python3-pip
sudo apt install python3-colcon-common-extensions -y
# Install Python dependencies
pip3 install pyserial numpy scipy ultralytics opencv-python
```

### Step 3: Build Workspace

```bash
cd ~/lododo-arm
colcon build

# Source the workspace
source install/setup.bash

# Source ros2 humble
source /opt/ros/humble/setup.bash
```

### Step 4: Launch System

**Single Machine (All-in-One):**
```bash
# Launch complete system
ros2 launch arm_bringup real_bringup.launch.py

# Terminal 2 (YOLO perception)
ros2 run arm_bringup start_yolo_cube_detect.sh
```

**Distributed Deployment (Recommended):**

```bash
# On Raspberry Pi (robot side)
ros2 launch arm_bringup robot_side.launch.py

# On PC (control side) - Terminal 1
ros2 launch arm_bringup pc_side.launch.py

# On PC - Terminal 2 (YOLO perception)
ros2 run arm_bringup start_yolo_cube_detect.sh
```

**Simulation:**
```bash
ros2 launch arm_bringup sim_bringup.launch.py
```

**For Cube Detection Mode:**
```bash
ros2 run arm_bringup start_yolo_cube_detect.sh
```

## 🎮 Usage

### Control Panel
The custom RViz plugin provides buttons for:
- **Scan Front** - Scan objects from front view
- **Scan All** - Multi-view scanning (15° intervals)
- **Grasp Lift** - Pick up detected object
- **Deliver to Pose** - Place object at target location

### Voice Commands
With voice control enabled:
- "扫描正面" (Scan front)
- "扫描全部" (Scan all)
- "抓取提升" (Grasp and lift)
- "放到姿态" (Deliver to pose)

### Command Line
```bash
# Publish detection command
ros2 topic pub /detection_command arm_interfaces/msg/DetectionCommand "{command: 'scan_front'}"

# Publish grasp command
ros2 topic pub /grasp_command arm_interfaces/msg/GraspCommand "{...}"
```

## 📐 Stereo Vision & Triangulation

The system uses a novel **rotational stereo vision** approach:
- Camera mounted on robotic arm
- Joint1 rotation creates baseline (typically 15°)
- Triangulation calculates object depth and dimensions
- Calibration factor for depth accuracy: 0.584

## 🎲 Cube Detection Mode

Specialized mode for detecting 5cm cubes:
- Edge length filtering (3-8cm)
- Position validation (within workspace)
- Shape scoring (bbox aspect ratio + 3D similarity)
- Automatic best candidate selection

Configuration: `config/measurement_params.yaml`

## 📚 Documentation

- [Architecture Overview](docs/ARCHITECTURE.md)
- [Calibration Guide](docs/depth_calibration_issue.md)
- [Cube Detection](docs/cube_detection_guide.md)
- [Troubleshooting](docs/)

## 🛠️ Configuration

Key configuration files:
- `config/measurement_params.yaml` - Vision and triangulation parameters
- `config/perception_params.yaml` - YOLO detection settings
- `config/ompl_planning.yaml` - MoveIt2 planning parameters

## 🤝 Contributing

We welcome contributions! Please read our [Contributing Guidelines](CONTRIBUTING.md) before submitting PRs.

### How to Contribute

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📝 License

This project is licensed under the **Apache License 2.0** - see the [LICENSE](LICENSE) file for details.

```
Copyright 2024 lododo

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
```

## � Authors & Contributors

**Main Developer:**
- **lododo** - *Initial work and maintenance*
  - GitHub: [@harryzy](https://github.com/harryzy)
  - Email: contect@lododo.org

See also the list of [contributors](https://github.com/harryzy/lododo-arm/contributors) who participated in this project.

## 🙏 Acknowledgments

- **ROS2 & MoveIt2** - Robot Operating System and motion planning
- **Ultralytics** - YOLOv8 object detection framework
- **Intel RealSense** - Depth camera SDK and ROS2 wrapper
- **Vosk** - Offline speech recognition
- **Feetech** - Servo motor SDK
- **Open Robotics** - ROS2 ecosystem and tools

## 📞 Support & Contact

- **Issues**: [GitHub Issues](https://github.com/harryzy/lododo-arm/issues)
- **Discussions**: [GitHub Discussions](https://github.com/harryzy/lododo-arm/discussions)
- **Email**: contect@lododo.org

## ⭐ Star History

If you find this project useful, please consider giving it a star ⭐️

## 📊 Project Status

**Active Development** | Last Updated: November 2024

Current Version: v0.972

- ✅ Basic motion control
- ✅ Object detection and tracking
- ✅ Stereo vision triangulation
- ✅ Grasp planning and execution
- ✅ Voice control interface
- ✅ Cube detection mode
- 🚧 Multi-object manipulation
- 🚧 Deep learning grasp pose estimation
- 🚧 Adaptive gripper control

## 🐛 Known Issues

See [Issues](https://github.com/harryzy/lododo-arm/issues) for a list of known issues and feature requests.

## 📞 Support

If you encounter any problems or have questions, please:
1. Check the [documentation](docs/)
2. Search [existing issues](https://github.com/harryzy/lododo-arm/issues)
3. Create a new issue with detailed information

---

⭐ If you find this project helpful, please consider giving it a star!
