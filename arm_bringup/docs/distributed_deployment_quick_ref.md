# ROS2 Distributed Deployment - Quick Reference

## 🚀 Quick Start

### First Time Setup (Once Only)

#### On Raspberry Pi
```bash
cd ~/lododo-arm/src/arm_bringup
./scripts/setup_robot_side.sh
source ~/.bashrc
```

#### On PC
```bash
cd ~/lododo-arm/src/arm_bringup
./scripts/setup_pc_side.sh
source ~/.bashrc
```

### Daily Usage

#### Startup Sequence

**1. Start Robot Side First** (SSH to Raspberry Pi)
```bash
~/start_robot.sh
# Or
robot-start
# Or manually
ros2 launch arm_bringup robot_side.launch.py
```

**2. Then Start PC Side (Two Terminals Required)**

Terminal 1 - Main control:
```bash
# Test connection first
~/test_robot_connection.sh

# Start PC side (with RViz)
~/start_pc.sh
# Or
pc-start-rviz
# Or manually
ros2 launch arm_bringup pc_side.launch.py rviz:=true
```

Terminal 2 - YOLO perception:
```bash
# Start YOLO node (requires isolated environment)
# Option 1 (Recommended): Cube detection
ros2 run arm_bringup start_yolo_cube_detect.sh

# Option 2 (Advanced): Dual-view triangulation
# ros2 run arm_bringup start_yolo_dual_view.sh

# Option 3 (Basic): Single-view projection
# ros2 run arm_bringup start_yolo_node.sh
```

## 📝 Common Commands

### Check Status
```bash
# View all nodes
ros2 node list

# View all topics
ros2 topic list

# View topic frequencies
ros2 topic hz /joint_states
ros2 topic hz /camera/color/image_raw

# View TF tree
ros2 run tf2_tools view_frames
```

### Test Grasping
```bash
# Grasp cup
ros2 service call /grasp_object arm_interfaces/srv/GraspObject "{target_name: 'cup'}"

# Grasp bottle
ros2 service call /grasp_object arm_interfaces/srv/GraspObject "{target_name: 'bottle'}"
```

### Detection Mode Switching
```bash
# Switch to triggered mode
ros2 param set /yolo_perception_node detection_mode triggered

# Switch to continuous mode
ros2 param set /yolo_perception_node detection_mode continuous

# Manually trigger detection
ros2 service call /trigger_detection std_srvs/srv/Trigger
```

## 🔧 Launch File Parameters

### robot_side.launch.py (Raspberry Pi)
```bash
ros2 launch arm_bringup robot_side.launch.py \
    port:=/dev/ttyUSB0 \
    enable_camera:=true \
    camera_serial_no:="" \
    ros_domain_id:=0
```

**Parameters:**
- `port`: Serial port device path (default: /dev/ttyUSB0)
- `enable_camera`: Enable camera (default: true)
- `camera_serial_no`: Camera serial number, empty=first available (default: "")
- `ros_domain_id`: ROS domain ID, must match PC side (default: 0)

### pc_side.launch.py (PC)
```bash
ros2 launch arm_bringup pc_side.launch.py \
    rviz:=true \
    voice_control:=false \
    ros_domain_id:=0
```

**Parameters:**
- `rviz`: Launch RViz (default: true)
- `voice_control`: Enable voice control (default: false)
- `ros_domain_id`: ROS domain ID, must match robot side (default: 0)

**Note:** YOLO perception must be started separately:
```bash
# Option 1 (Recommended): Cube detection
ros2 run arm_bringup start_yolo_cube_detect.sh

# Option 2 (Advanced): Dual-view triangulation
# ros2 run arm_bringup start_yolo_dual_view.sh

# Option 3 (Basic): Single-view projection
# ros2 run arm_bringup start_yolo_node.sh
```

## 🐛 Troubleshooting

### Problem: PC Cannot See Robot Nodes

**Checklist:**
```bash
# 1. Network connectivity
ping <raspberry_pi_ip>

# 2. ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID  # Run on both sides

# 3. ROS_LOCALHOST_ONLY must be 0
echo $ROS_LOCALHOST_ONLY  # Run on both sides

# 4. Test multicast
# On Raspberry Pi:
ros2 multicast receive
# On PC:
ros2 multicast send

# 5. Check firewall
sudo ufw status
```

**Solution:**
```bash
# Ensure environment variables are correct
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Reload environment
source ~/.bashrc

# Restart nodes on both sides
```

### Problem: High Camera Data Latency

**Optimization:**
```bash
# Reduce camera resolution and frame rate (modify on Raspberry Pi side)
depth_module.profile: "640x480x15"
rgb_camera.profile: "640x480x15"
```

### Problem: MoveIt Planning Failures

**Check:**
```bash
# 1. Check joint_states
ros2 topic echo /joint_states

# 2. Check TF
ros2 run tf2_ros tf2_echo base_link ee_link

# 3. Check move_group is running
ros2 node info /move_group
```

## 📊 Performance Monitoring

### Raspberry Pi
```bash
# CPU/Memory
htop

# Network traffic
iftop

# Topic bandwidth
ros2 topic bw /camera/color/image_raw
ros2 topic bw /joint_states
```

### PC
```bash
# GPU usage
nvidia-smi

# ROS2 node resources
ros2 run rqt_top rqt_top
```

## 🔐 Security Tips

1. **Change default ROS_DOMAIN_ID**: Avoid conflicts with other robots
   ```bash
   export ROS_DOMAIN_ID=42  # Choose a unique number
   ```

2. **Use static IP**: Set static IP for Raspberry Pi

3. **SSH key login**: Set up passwordless login
   ```bash
   ssh-copy-id ubuntu@<raspberry_pi_ip>
   ```

## 📚 Related Documentation

- [Full Deployment Guide](distributed_deployment_guide.md)
- [System Architecture](../ARCHITECTURE.md)
- [Getting Started Guide](../GETTING_STARTED.md)

## 🆘 Getting Help

1. Check logs: `~/.ros/log/`
2. GitHub Issues: https://github.com/harryzy/lododo-arm/issues
3. ROS2 Community: https://discourse.ros.org/
