# Lododo Arm - Intelligent Robotic Arm System

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-green.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-blue.svg)](https://www.python.org/)

An intelligent robotic arm system integrating computer vision, motion planning, and voice control based on ROS2.

## ✨ Features

- 🤖 **6-DOF Robotic Arm** - Full motion control using Feetech servos
- 👁️ **Computer Vision** - YOLOv8-based object detection with stereo vision
- 🎯 **Intelligent Grasping** - Automatic grasp planning with MoveIt2
- 🗣️ **Voice Control** - Natural language commands using Vosk
- 📊 **Real-time Monitoring** - Custom RViz plugin for system control
- 🎲 **Cube Detection** - Specialized mode for detecting and picking cubes

## 🏗️ System Architecture

```
├── arm_bringup          # Launch files and startup scripts
├── arm_description      # URDF robot models
├── arm_driver_node      # Hardware driver for Feetech servos
├── arm_interfaces       # Custom ROS2 message definitions
├── arm_moveit_config    # MoveIt2 configuration
├── arm_perception_yolo  # YOLOv8 object detection
├── arm_perception_rtdetr # RT-DETR detection (experimental)
├── arm_planning_py      # Motion planning and grasping logic
├── arm_rviz_plugin      # Custom RViz control panel
└── arm_voice_interface  # Voice command interface
```

## 📋 Prerequisites

### Hardware Requirements
- 6-DOF robotic arm with Feetech servos (ST3215 or compatible)
- USB camera (640x480 or higher)
- Ubuntu 22.04 (recommended)

### Software Requirements
- ROS2 Humble
- Python 3.10+
- PyTorch (for YOLO)
- MoveIt2
- OpenCV

## 🚀 Quick Start

### 1. Installation

```bash
# Clone repository (ROS2 workspace structure)
mkdir -p ~/lododo-arm/src
cd ~/lododo-arm/src
git clone https://github.com/harryzy/lododo-arm.git .
cd ~/lododo-arm

# Install dependencies
sudo apt update
sudo apt install ros-humble-desktop ros-humble-moveit ros-humble-gazebo-ros2-control

# Install Python dependencies
pip install ultralytics opencv-python numpy vosk

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### 2. Launch the System

**For Real Hardware:**
```bash
ros2 launch arm_bringup real_bringup.launch.py
```

**For Simulation:**
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

Contributions are welcome! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for details.

## 📝 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## 👨‍💻 Author

**lododo**
- GitHub: [@harryzy](https://github.com/harryzy)
- Email: contect@lododo.org

## 🙏 Acknowledgments

- ROS2 and MoveIt2 communities
- Ultralytics YOLOv8
- Vosk speech recognition
- Feetech servo SDK

## 📊 Project Status

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
