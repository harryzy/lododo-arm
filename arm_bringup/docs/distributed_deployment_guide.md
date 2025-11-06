# ROS2 Distributed Deployment Guide

This guide explains how to deploy the robotic arm system across a Raspberry Pi (robot side) and PC (control side) using ROS2's distributed architecture.

## 📋 System Architecture

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

## 🔧 Hardware Requirements

### Robot Side (Raspberry Pi)
- **Model**: Raspberry Pi 4 (4GB+ RAM recommended)
- **OS**: Ubuntu 22.04 Server 64-bit
- **Storage**: 32GB+ microSD card
- **Connections**: 
  - USB-to-Serial adapter → Servo bus
  - USB 3.0 → RealSense D435i camera
  - Ethernet → Router/Switch

### Control Side (PC)
- **OS**: Ubuntu 22.04 Desktop
- **GPU**: NVIDIA GPU (recommended for YOLO acceleration)
- **RAM**: 8GB+
- **Connection**: Ethernet → Router/Switch

## 🌐 Network Configuration

### 1. Ensure Both Devices Are on Same Network

```bash
# On Raspberry Pi - check IP
ip addr show

# On PC - check IP
ip addr show

# Test connectivity (from PC)
ping <raspberry_pi_ip>

# Test connectivity (from Raspberry Pi)
ping <pc_ip>
```

### 2. Configure ROS2 DDS

Create DDS configuration file (on both sides):

```bash
# Create config directory
mkdir -p ~/.ros

# Create DDS configuration file
cat > ~/.ros/fastdds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>SIMPLE</discoveryProtocol>
                    <leaseDuration>
                        <sec>30</sec>
                    </leaseDuration>
                </discovery_config>
            </builtin>
        </rtps>
    </participant>
</profiles>
EOF
```

### 3. Set Environment Variables

**Raspberry Pi** (`~/.bashrc`):
```bash
# ROS2 environment
source /opt/ros/humble/setup.bash
source ~/lododo-arm/install/setup.bash

# DDS configuration
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

**PC** (`~/.bashrc`):
```bash
# ROS2 environment
source /opt/ros/humble/setup.bash
source ~/lododo-arm/install/setup.bash

# DDS configuration
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

**Important**: Reload configuration
```bash
source ~/.bashrc
```

## 🚀 Deployment Steps

### Step 1: Install and Configure on Raspberry Pi

```bash
# 1. Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# 2. Clone project
mkdir -p ~/lododo-arm/src
cd ~/lododo-arm/src
git clone https://github.com/harryzy/lododo-arm.git .
cd ~/lododo-arm

# 3. Install dependencies
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    python3-pip

pip3 install pyserial numpy scipy

# 4. Build project (only necessary packages)
colcon build --packages-select \
    arm_description \
    arm_interfaces \
    arm_driver_node \
    arm_bringup

# 5. Set serial port permissions
sudo usermod -a -G dialout $USER
# Log out and log back in for changes to take effect

# 6. Test device connections
ls -l /dev/ttyUSB*  # Check serial port
rs-enumerate-devices  # Check camera
```

### Step 2: Install and Configure on PC

```bash
# 1. Clone project
mkdir -p ~/lododo-arm/src
cd ~/lododo-arm/src
git clone https://github.com/harryzy/lododo-arm.git .
cd ~/lododo-arm

# 2. Install dependencies
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-rviz2 \
    python3-pip

pip3 install torch torchvision ultralytics scipy

# 3. Build project
colcon build

# 4. Download YOLO model (if not present)
cd ~/lododo-arm
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8m.pt
```

### Step 3: Start System

#### On Raspberry Pi (via SSH)

```bash
# Terminal 1: Start robot side
cd ~/lododo-arm
source install/setup.bash
ros2 launch arm_bringup robot_side.launch.py port:=/dev/ttyUSB0

# Optional parameters:
# port:=/dev/ttyUSB0              # Serial port device
# enable_camera:=true             # Enable camera
# camera_serial_no:=""            # Camera serial number (empty=first available)
# ros_domain_id:=0                # ROS domain ID
```

#### On PC

```bash
# Terminal 1: Start PC side
cd ~/lododo-arm
source install/setup.bash
ros2 launch arm_bringup pc_side.launch.py

# Optional parameters:
# rviz:=true                      # Launch RViz
# voice_control:=false            # Enable voice control
# ros_domain_id:=0                # ROS domain ID

# Terminal 2: Start YOLO perception (in separate virtual environment)
# Recommended: Cube detection
ros2 run arm_bringup start_yolo_cube_detect.sh

# Alternative 1: Dual-view triangulation (advanced)
# ros2 run arm_bringup start_yolo_dual_view.sh

# Alternative 2: Single-view projection (basic/legacy)
# ros2 run arm_bringup start_yolo_node.sh
```

**Note:** YOLO perception requires a separate terminal with isolated virtual environment. It cannot be launched together with other ROS2 nodes due to dependency conflicts.

**Script Selection Guide:**
- **start_yolo_cube_detect.sh**: Optimized for cube grasping (recommended)
- **start_yolo_dual_view.sh**: Advanced dual-view triangulation measurement
- **start_yolo_node.sh**: Basic single-view projection (requires camera tuning)

## 🔍 Verify Deployment

### 1. Check Node Communication

On PC:
```bash
# View all nodes
ros2 node list

# Should see nodes from both sides:
# /arm_driver_node           (Raspberry Pi)
# /robot_state_publisher     (Raspberry Pi)
# /camera/realsense2_camera_node (Raspberry Pi)
# /move_group                (PC)
# /arm_planning_py_node      (PC)
# /yolo_perception_node      (PC)
# /rviz2                     (PC)

# View topics
ros2 topic list

# Check key topic frequencies
ros2 topic hz /joint_states
ros2 topic hz /camera/color/image_raw
```

### 2. Check TF Tree

```bash
# On PC
ros2 run tf2_tools view_frames

# Open generated frames.pdf to view TF tree
```

### 3. Test Arm Control

```bash
# On PC, test via RViz MotionPlanning plugin
# Or use command line to test grasp service
ros2 service call /grasp_object arm_interfaces/srv/GraspObject "{target_name: 'cup'}"
```

## 🐛 Troubleshooting

### Issue 1: Nodes Cannot Discover Each Other

**Symptom**: `ros2 node list` only shows local nodes

**Solution**:
```bash
# 1. Check ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID  # Should be same on both sides

# 2. Check ROS_LOCALHOST_ONLY
echo $ROS_LOCALHOST_ONLY  # Should be 0

# 3. Check firewall
sudo ufw status
# If enabled, allow ROS2 ports
sudo ufw allow 7400:7500/udp
sudo ufw allow 7400:7500/tcp

# 4. Test multicast
# On Raspberry Pi:
ros2 multicast receive
# On PC:
ros2 multicast send
```

### Issue 2: High Camera Data Latency

**Solution**:
```bash
# On Raspberry Pi, reduce camera resolution and frame rate
ros2 launch arm_bringup robot_side.launch.py \
    camera_resolution:=640x480 \
    camera_fps:=15
```

### Issue 3: Planning Failures

**Symptom**: MoveIt2 planning fails or times out

**Solution**:
```bash
# 1. Check joint_states are being published
ros2 topic echo /joint_states

# 2. Check TF tree is complete
ros2 run tf2_ros tf2_echo base_link ee_link

# 3. Increase planning time
ros2 launch arm_bringup pc_side.launch.py planning_time:=10.0
```

### Issue 4: Raspberry Pi Performance

**Optimization**:
```bash
# 1. Disable unnecessary services
sudo systemctl disable bluetooth
sudo systemctl disable avahi-daemon

# 2. Overclock Raspberry Pi (use with caution)
# Edit /boot/firmware/config.txt
over_voltage=6
arm_freq=2000

# 3. Use lightweight camera configuration
# Only enable necessary streams (depth + color, no infrared)
```

## 📊 Performance Monitoring

### Raspberry Pi Monitoring

```bash
# CPU and memory usage
htop

# Network traffic
iftop

# ROS2 topic bandwidth
ros2 topic bw /camera/color/image_raw
ros2 topic bw /joint_states
```

### PC Monitoring

```bash
# GPU usage (YOLO)
nvidia-smi

# ROS2 node performance
ros2 run rqt_top rqt_top
```

## 🔐 Security Recommendations

1. **Use VPN**: If communicating over public networks, use VPN (e.g., WireGuard)
2. **Non-default ROS_DOMAIN_ID**: Use non-default value to avoid conflicts with other robots
3. **Firewall Rules**: Only open necessary ports
4. **SSH Keys**: Use key-based authentication instead of passwords

## 📚 References

- [ROS2 Network Configuration](https://docs.ros.org/en/humble/Concepts/About-Different-Middleware-Vendors.html)
- [FastDDS Configuration](https://fast-dds.docs.eprosima.com/en/latest/)
- [MoveIt2 Documentation](https://moveit.picknik.ai/main/index.html)
- [RealSense ROS2 Package](https://github.com/IntelRealSense/realsense-ros)

## 🆘 Getting Help

If you encounter issues, check:
1. ROS2 logs on both sides: `~/.ros/log/`
2. Network connectivity: `ping` and `ros2 multicast`
3. GitHub Issues: https://github.com/harryzy/lododo-arm/issues
