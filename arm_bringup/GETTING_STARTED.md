# Getting Started with Arm Bringup

Welcome! This guide will help you get started with the arm system in just a few minutes.

## 🚀 Quick Start (3 Steps)

### Step 1: Build the Package
```bash
cd ~/lododo-arm
colcon build --packages-select arm_bringup
source install/setup.bash
```

### Step 2: Check System Health
```bash
ros2 run arm_bringup health_check.sh
```

### Step 3: Launch!

**For first-time users** (Interactive menu):
```bash
ros2 run arm_bringup quick_launch.sh
```

**For simulation**:
```bash
ros2 launch arm_bringup sim_bringup.launch.py
```

That's it! 🎉

---

## 📖 Detailed Guide

### What is arm_bringup?

`arm_bringup` is the "ignition key" for your robotic arm system. Instead of starting multiple nodes manually, you can launch everything with one command.

Think of it like:
- **Traditional way**: Start engine, turn on lights, adjust mirrors, engage gear...
- **With bringup**: Turn the key and go! 🚗

### What Can You Do?

1. **🎮 Full Simulation** - Test everything safely in Gazebo
2. **🤖 Real Robot** - Control physical hardware
3. **🎤 Voice Control** - Control arm with voice commands (no screen needed)
4. **👁️ Perception Testing** - Test camera and object detection
5. **🧠 Planning Testing** - Test motion planning algorithms
6. **🔧 Minimal Mode** - Basic hardware testing

### Understanding Launch Files

Each launch file is for a specific scenario:

| Launch File | When to Use | What It Does |
|------------|-------------|--------------|
| `sim_bringup` | Learning, development | Everything in simulation |
| `real_bringup` | Production, real robot | Everything on hardware |
| `voice_headless` | Embedded systems | Voice control only, no GUI |
| `perception_bringup` | Camera testing | Just the vision system |
| `planning_bringup` | Motion testing | Just motion planning |
| `minimal_bringup` | Troubleshooting | Just basic control |

### Common Workflows

#### 🎓 Learning Workflow
```bash
# 1. Start in simulation (safe!)
ros2 launch arm_bringup sim_bringup.launch.py

# 2. In another terminal, try commands
ros2 topic pub /arm_command std_msgs/String "data: 'scan_front'" --once

# 3. Watch what happens in RViz and Gazebo
```

#### 🔬 Development Workflow
```bash
# 1. Test individual subsystems first
ros2 launch arm_bringup perception_bringup.launch.py
# Check if camera works

ros2 launch arm_bringup planning_bringup.launch.py  
# Check if planning works

# 2. Then test full simulation
ros2 launch arm_bringup sim_bringup.launch.py

# 3. Finally test on real robot
ros2 launch arm_bringup real_bringup.launch.py
```

#### 🏭 Production Workflow
```bash
# 1. Health check
ros2 run arm_bringup health_check.sh

# 2. Launch system
ros2 launch arm_bringup real_bringup.launch.py

# 3. Monitor
ros2 topic echo /arm_command_result
```

### Using the Quick Launch Menu

The easiest way to start:

```bash
ros2 run arm_bringup quick_launch.sh
```

You'll see:
```
==========================================
    Arm System Quick Launch Menu
==========================================
1) sim       - Full simulation system
2) real      - Real robot system
3) voice     - Headless voice control
4) minimal   - Minimal system
5) perception - Perception testing
6) planning  - Planning testing
==========================================
Select mode (1-6):
```

Just type the number and press Enter!

### Customizing Launch

All launch files support arguments. For example:

```bash
# Simulation without voice control
ros2 launch arm_bringup sim_bringup.launch.py use_voice_control:=false

# Real robot with custom serial port
ros2 launch arm_bringup real_bringup.launch.py serial_port:=/dev/ttyACM0

# Perception with better YOLO model
ros2 launch arm_bringup perception_bringup.launch.py model_path:=yolov8m.pt
```

To see all available arguments:
```bash
ros2 launch arm_bringup sim_bringup.launch.py --show-args
```

### Sending Commands

There are three ways to send commands:

#### 1. Using RViz Plugin (GUI)
- Click the buttons in the control panel
- Check the "Enable Voice Control" box for voice

#### 2. Using Command Line
```bash
ros2 topic pub /arm_command std_msgs/String "data: 'scan_front'" --once
ros2 topic pub /arm_command std_msgs/String "data: 'scan_all'" --once
ros2 topic pub /arm_command std_msgs/String "data: 'scan_and_grasp'" --once
```

#### 3. Using Voice (if enabled)
Just say:
- "scan front"
- "scan all"
- "grasp"

### Monitoring Results

See what the arm is doing:
```bash
# Watch results in real-time
ros2 topic echo /arm_command_result

# Or check RViz plugin's result panel (shows last 10)
```

### Troubleshooting

#### "Package not found"
```bash
# Make sure you built it
colcon build --packages-select arm_bringup
source install/setup.bash
```

#### "Serial port permission denied"
```bash
# Add yourself to dialout group
sudo usermod -a -G dialout $USER
# Then logout and login again
```

#### "Gazebo won't start"
```bash
# Kill existing Gazebo processes
killall gzserver gzclient
# Then try again
```

#### "Can't find camera"
```bash
# List available cameras
v4l2-ctl --list-devices
# Use the correct device
ros2 launch arm_bringup real_bringup.launch.py camera_device:=/dev/video2
```

### What's Next?

- 📚 Read [README.md](README.md) for complete documentation
- 🏗️ See [ARCHITECTURE.md](ARCHITECTURE.md) for system design
- ⚡ Check [QUICK_REFERENCE.md](QUICK_REFERENCE.md) for command cheatsheet
- 🔍 Run health check regularly: `ros2 run arm_bringup health_check.sh`

### Need Help?

1. **Check system health**: `ros2 run arm_bringup health_check.sh`
2. **Read error messages**: They usually tell you what's wrong
3. **Check topics**: `ros2 topic list` and `ros2 topic echo /arm_command_result`
4. **Enable debug logging**: Add `log_level:=debug` to launch command

### Tips for Success

✅ **DO:**
- Start with simulation before using real robot
- Run health check before launching
- Read the startup messages (they contain useful info)
- Use tab completion: `ros2 launch arm_bringup <TAB>`

❌ **DON'T:**
- Don't skip the health check
- Don't run multiple launch files at once (they conflict)
- Don't ignore warning messages
- Don't forget to source the workspace!

### Example Session

Here's a complete example session:

```bash
# Terminal 1: Setup
cd ~/lododo-arm
source install/setup.bash

# Check everything is OK
ros2 run arm_bringup health_check.sh

# Launch simulation
ros2 launch arm_bringup sim_bringup.launch.py

# Terminal 2: Control
source install/setup.bash

# Send a command
ros2 topic pub /arm_command std_msgs/String "data: 'scan_front'" --once

# Watch the result
ros2 topic echo /arm_command_result

# Or just use the RViz GUI buttons! 🖱️
```

---

**Ready to start? Run this now:**
```bash
ros2 run arm_bringup quick_launch.sh
```

Happy robotics! 🤖✨
