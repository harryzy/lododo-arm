# Arm RViz Plugin

An RViz plugin for robotic arm control, providing a visual interface to send commands and monitor execution results.

## Features

### 1. Command Buttons
- **Scan Front**: Sends `scan_front` command to `/arm_command` topic
- **Scan All**: Sends `scan_all` command to `/arm_command` topic
- **Scan & Grasp**: Sends `scan_and_grasp` command to `/arm_command` topic

### 2. Voice Control Toggle
- Checkbox to enable/disable voice control
- When enabled, you can control the arm with voice commands (requires `arm_voice_node` running)
- Supported voice commands:
  - "scan front" → scan_front
  - "scan all" → scan_all
  - "grasp" / "scan and grasp" → scan_and_grasp

### 3. Result Display Panel
- Real-time display of execution results from `/arm_command_result` topic
- Shows up to 10 most recent messages
- Each message includes timestamp
- Auto-scrolls to show latest messages

## Build & Install


```bash
cd ~/lododo-arm
colcon build --packages-select arm_rviz_plugin
source install/setup.bash
```

## Usage

### Method 1: Add Panel to Existing RViz

1. Launch RViz2:
```bash
rviz2
```

2. In RViz menu bar, select: `Panels` → `Add New Panel`

3. In the popup dialog, select: `arm_rviz_plugin/ArmControlPanel`

4. Click `OK`, and the panel will appear in the RViz window

### Method 2: Use Launch File

```bash
ros2 launch arm_rviz_plugin arm_control_rviz.launch.py
```

## Integration with Other Nodes

### Launch Voice Control Node (Optional)
If you want to use voice control:

```bash
ros2 run arm_voice_interface arm_voice_node
```

### Launch Command Handler Node
A node must subscribe to `/arm_command` topic and publish results to `/arm_command_result`:

```bash
ros2 run arm_planning_py arm_command_interface
```

## Topic Interfaces

### Published Topics
- `/arm_command` (std_msgs/String): Sends control commands

### Subscribed Topics
- `/arm_command_result` (std_msgs/String): Receives execution results

## UI Description

The plugin interface contains the following components:

1. **Title Bar**: Shows "Arm Control Panel"
2. **Command Control Area**: Three colored buttons for sending different commands
   - Green: Scan Front
   - Blue: Scan All
   - Orange: Scan & Grasp
3. **Voice Control Area**: Checkbox to enable/disable voice control
4. **Status Label**: Shows current operation status
5. **Execution Results Area**: Text box displaying the last 10 operation records

## Troubleshooting

### Plugin Cannot Load
Make sure you have built and sourced the workspace:
```bash
colcon build --packages-select arm_rviz_plugin
source install/setup.bash
```

### Commands Not Responding
Check if `/arm_command` topic has subscribers:
```bash
ros2 topic info /arm_command
```

### No Execution Results Shown
Check if `/arm_command_result` topic has publishers:

