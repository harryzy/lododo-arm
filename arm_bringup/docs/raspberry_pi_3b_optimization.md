# Raspberry Pi 3B+ Optimization Guide

## Hardware Specifications

**Raspberry Pi 3B+:**
- CPU: 4-core ARM Cortex-A53 @ 1.4GHz
- RAM: 1GB LPDDR2
- Storage: 32GB SD card
- USB: USB 2.0 (shared bandwidth)
- Network: 300Mbps Ethernet

**Comparison with RPi 4B (recommended):**
| Feature | RPi 3B+ | RPi 4B |
|---------|---------|--------|
| RAM | 1GB | 2GB/4GB/8GB |
| CPU | 1.4GHz | 1.5-1.8GHz |
| USB | 2.0 | 3.0 |
| Network | 300Mbps | 1Gbps |

## Resource Requirements

### Standard robot_side.launch.py:
- **Memory:** ~500-700 MB
- **CPU:** 15-30% average
- **Bottleneck:** RealSense camera (200-300 MB RAM)

### Lite robot_side_lite.launch.py:
- **Memory:** ~300-450 MB
- **CPU:** 10-20% average
- **Optimizations:** Lower resolution, disabled RGB, reduced framerate

## ⚠️ Known Issues on RPi 3B+

### 1. **Memory Pressure**
**Symptom:** System lag, node crashes, OOM killer
**Cause:** 1GB RAM insufficient for full-resolution camera + ROS2
**Solution:** Use `robot_side_lite.launch.py` with reduced camera settings

### 2. **USB Bandwidth**
**Symptom:** Frame drops, delayed camera data
**Cause:** USB 2.0 shared between camera and serial port
**Solution:** Use lower resolution (424x240) and disable RGB

### 3. **Swap Performance**
**Symptom:** Severe lag when memory fills up
**Cause:** SD card swap is very slow
**Solution:** Minimize swap usage, enable zram

## 🚀 Optimization Steps

### Step 1: Enable ZRAM (Compressed RAM)

ZRAM creates compressed swap in RAM, faster than SD card:

```bash
# Install zram-config
sudo apt update
sudo apt install zram-config

# Enable and start
sudo systemctl enable zram-config
sudo systemctl start zram-config

# Verify
sudo zramctl
# Expected output: ~250MB zram device
```

### Step 2: Reduce Swap on SD Card

```bash
# Reduce swappiness (prefer ZRAM over SD swap)
sudo sysctl vm.swappiness=10
echo "vm.swappiness=10" | sudo tee -a /etc/sysctl.conf

# Optionally disable SD swap entirely (risky, only if ZRAM working)
# sudo dphys-swapfile swapoff
# sudo systemctl disable dphys-swapfile
```

### Step 3: Disable Unnecessary Services

```bash
# Check running services
systemctl list-units --type=service --state=running

# Disable GUI (if running headless)
sudo systemctl set-default multi-user.target

# Disable bluetooth (if not needed)
sudo systemctl disable bluetooth.service

# Free up memory
free -h  # Check available memory
```

### Step 4: Optimize ROS2 Settings

Create `~/.bashrc` additions:

```bash
# ROS2 memory optimization
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_lowmem.xml

# Reduce DDS discovery traffic
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

Create `~/.ros/fastdds_lowmem.xml`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<dds>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>udp_transport</transport_id>
                <type>UDPv4</type>
                <sendBufferSize>65536</sendBufferSize>
                <receiveBufferSize>65536</receiveBufferSize>
            </transport_descriptor>
        </transport_descriptors>
        
        <participant profile_name="participant_profile" is_default_profile="true">
            <rtps>
                <builtin>
                    <discovery_config>
                        <leaseDuration>
                            <sec>30</sec>
                        </leaseDuration>
                    </discovery_config>
                </builtin>
                <useBuiltinTransports>false</useBuiltinTransports>
                <userTransports>
                    <transport_id>udp_transport</transport_id>
                </userTransports>
            </rtps>
        </participant>
    </profiles>
</dds>
```

### Step 5: Use Lite Launch File

```bash
# Standard version (may struggle on 3B+)
ros2 launch arm_bringup robot_side.launch.py

# Optimized lite version (recommended for 3B+)
ros2 launch arm_bringup robot_side_lite.launch.py
```

## 📊 Performance Comparison

### Standard robot_side.launch.py:
- Depth: 640x480 @ 30fps
- RGB: 640x480 @ 30fps (aligned)
- Memory: ~600 MB
- CPU: ~25%
- Status: **May experience OOM on 3B+**

### Lite robot_side_lite.launch.py:
- Depth: 424x240 @ 15fps
- RGB: Disabled
- Memory: ~350 MB
- CPU: ~15%
- Status: **Should work on 3B+**

## 🧪 Testing Procedure

### 1. Monitor System Resources

```bash
# Terminal 1: Monitor memory
watch -n 1 free -h

# Terminal 2: Monitor CPU
htop

# Terminal 3: Monitor temperatures
watch -n 2 'vcgencmd measure_temp'
```

### 2. Launch Robot Side

```bash
# Source ROS2
source /opt/ros/humble/setup.bash
source ~/lododo-arm/install/setup.bash

# Launch lite version
ros2 launch arm_bringup robot_side_lite.launch.py
```

### 3. Check Node Status

```bash
# Verify all nodes running
ros2 node list

# Expected output:
# /arm_driver_node
# /robot_state_publisher
# /realsense2_camera (if enable_camera:=true)

# Check topics
ros2 topic list
ros2 topic hz /camera/depth/image_rect_raw
```

### 4. Stress Test

```bash
# On PC side, start full system
ros2 launch arm_bringup pc_side.launch.py

# Monitor RPi memory during operation
ssh pi@<raspberry_pi_ip>
watch -n 1 free -h
```

## ⚙️ Advanced Tuning

### If Still Running Out of Memory:

1. **Disable point cloud generation:**
   ```python
   # In robot_side_lite.launch.py
   'pointcloud.enable': False,  # Change to False
   ```

2. **Further reduce resolution:**
   ```python
   'depth_module.profile': '320x180x15',  # Even lower
   ```

3. **Reduce robot_state_publisher frequency:**
   ```python
   'publish_frequency': 15.0,  # Down from 30Hz
   ```

### If Camera Keeps Disconnecting:

USB power issue on RPi 3B+:

```bash
# Add to /boot/config.txt
sudo nano /boot/config.txt

# Add these lines:
max_usb_current=1
usb_max_current_enable=1

# Reboot
sudo reboot
```

### If CPU Throttling:

```bash
# Check throttling status
vcgencmd get_throttled
# 0x0 = no throttling
# 0x50000 = under-voltage detected

# Ensure good power supply (5V 2.5A minimum)
# Add heatsink to RPi CPU
```

## 🔍 Troubleshooting

### Symptom: Node crashes with "Killed"
**Cause:** OOM killer terminated process
**Fix:**
1. Enable ZRAM (see Step 1)
2. Use `robot_side_lite.launch.py`
3. Disable point cloud generation

### Symptom: Camera not detected
**Cause:** Insufficient USB power
**Fix:**
1. Use powered USB hub
2. Enable max_usb_current in /boot/config.txt
3. Use better power supply

### Symptom: System very slow
**Cause:** SD card swap thrashing
**Fix:**
1. Enable ZRAM
2. Reduce swappiness to 10
3. Monitor with `sudo iotop`

### Symptom: Frame drops on camera
**Cause:** USB 2.0 bandwidth limitation
**Fix:**
1. Lower resolution to 424x240
2. Disable RGB camera
3. Reduce framerate to 15fps

## ✅ Recommended Configuration

**For RPi 3B+ (1GB RAM):**
```bash
# Use lite launch file
ros2 launch arm_bringup robot_side_lite.launch.py \
    port:=/dev/ttyUSB0 \
    enable_camera:=true \
    ros_domain_id:=0

# Expected memory usage: ~350 MB
# Expected CPU usage: ~15%
```

**System optimizations applied:**
- ✅ ZRAM enabled
- ✅ Swappiness reduced to 10
- ✅ Unnecessary services disabled
- ✅ Camera resolution: 424x240 @ 15fps
- ✅ RGB camera disabled
- ✅ FastDDS low-memory profile

## 📈 Upgrade Path

**If experiencing issues, consider upgrading to:**
- **Raspberry Pi 4B (2GB):** ~$45, 2x memory
- **Raspberry Pi 4B (4GB):** ~$55, 4x memory, recommended
- **Raspberry Pi 5 (4GB):** ~$60, better CPU, but check ROS2 compatibility

**Benefits of upgrade:**
- Run full-resolution camera (640x480)
- Enable RGB alignment for better YOLO detection
- More headroom for additional nodes
- USB 3.0 for higher bandwidth

## 📝 Summary

**RPi 3B+ (1GB) CAN run robot_side with these conditions:**
- ✅ Use `robot_side_lite.launch.py`
- ✅ Enable ZRAM compression
- ✅ Accept lower camera resolution
- ✅ Monitor memory usage regularly

**But consider RPi 4B (2GB+) for:**
- Better performance margin
- Full camera resolution
- Future expandability
- Less troubleshooting

The lite configuration should work reliably on your 3B+, but you'll need to accept reduced camera quality. For production use, RPi 4B 4GB is highly recommended (~$55 investment for much better experience).
