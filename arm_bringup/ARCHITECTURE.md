# Arm System Architecture

This document provides an overview of the complete arm system architecture and how different packages interact.

## System Components

### 1. Hardware Layer
- **arm_driver_node**: Communicates with physical robot via serial port
- **Camera**: Captures images for perception system

### 2. Perception Layer
- **arm_perception_yolo**: YOLOv8-based object detection
  - Subscribes to camera feed
  - Publishes detected objects and bounding boxes

### 3. Planning Layer
- **arm_moveit_config**: MoveIt configuration
  - Motion planning
  - Collision checking
  - Trajectory execution
- **arm_planning_py**: High-level planning logic
  - **arm_command_interface**: Processes commands and coordinates actions
  - **arm_grasper**: Implements grasping strategies

### 4. Interface Layer
- **arm_voice_interface**: Voice command recognition using Vosk
- **arm_rviz_plugin**: Custom RViz control panel with buttons and status

### 5. Simulation Layer
- **Gazebo**: Physics simulation
- **arm_description**: URDF robot models

## Data Flow

```
┌─────────────────────────────────────────────────────────────┐
│                      User Interfaces                         │
├─────────────────┬───────────────────┬──────────────────────┤
│ Voice Commands  │  RViz Plugin      │  Command Line       │
│ (arm_voice_     │  (arm_rviz_       │  (ros2 topic pub)   │
│  interface)     │   plugin)         │                      │
└────────┬────────┴──────────┬────────┴──────────┬───────────┘
         │                   │                   │
         └───────────────────┼───────────────────┘
                            │
                            ▼
                  /arm_command (String)
                            │
                            ▼
         ┌──────────────────────────────────┐
         │   arm_command_interface          │
         │   (arm_planning_py)              │
         └──────────┬───────────────────────┘
                    │
         ┌──────────┼──────────┐
         │          │          │
         ▼          ▼          ▼
    scan_front  scan_all  scan_and_grasp
         │          │          │
         └──────────┼──────────┘
                    │
         ┌──────────┴──────────┐
         │                     │
         ▼                     ▼
  ┌─────────────┐      ┌─────────────┐
  │   MoveIt    │      │  Perception │
  │ (Planning & │◄─────┤   (YOLO)    │
  │ Execution)  │      │             │
  └──────┬──────┘      └─────────────┘
         │
         ▼
  ┌─────────────────┐
  │ arm_grasper     │
  │ (Grasping Logic)│
  └──────┬──────────┘
         │
         ▼
  /arm_command_result (String)
         │
         └─────────► RViz Plugin Display
```

## Topic Communication

### Published Topics
| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/arm_command` | std_msgs/String | Voice/RViz Plugin | Control commands |
| `/arm_command_result` | std_msgs/String | Planning Interface | Execution results |
| `/detected_objects` | vision_msgs/Detection2DArray | YOLO Node | Detected objects |
| `/camera/image_raw` | sensor_msgs/Image | Camera/Gazebo | Camera feed |
| `/joint_states` | sensor_msgs/JointState | Driver/Gazebo | Joint positions |
| `/move_group/*` | various | MoveIt | Planning/execution |

### Service Interfaces
| Service | Type | Server | Description |
|---------|------|--------|-------------|
| `/compute_ik` | moveit_msgs/GetPositionIK | MoveIt | Inverse kinematics |
| `/plan_kinematic_path` | moveit_msgs/GetMotionPlan | MoveIt | Motion planning |

## Launch File Dependencies

```
sim_bringup.launch.py
├── gazebo_launch.py (arm_moveit_config)
├── move_group.launch.py (arm_moveit_config)
├── arm_control_rviz.launch.py (arm_rviz_plugin)
├── yolo_detection_node (arm_perception_yolo)
├── arm_command_interface (arm_planning_py)
├── arm_grasper (arm_planning_py)
└── arm_voice_node (arm_voice_interface)

real_bringup.launch.py
├── driver_view_launch.py (arm_moveit_config)
├── move_group.launch.py (arm_moveit_config)
├── arm_control_rviz.launch.py (arm_rviz_plugin)
├── yolo_detection_node (arm_perception_yolo)
├── arm_command_interface (arm_planning_py)
├── arm_grasper (arm_planning_py)
└── arm_voice_node (arm_voice_interface)

voice_headless.launch.py
├── driver_view_launch.py (arm_moveit_config)
├── move_group_simple_launch.py (arm_moveit_config)
├── yolo_detection_node (arm_perception_yolo) [optional]
├── arm_command_interface (arm_planning_py)
├── arm_grasper (arm_planning_py)
└── arm_voice_node (arm_voice_interface)
```

## Command Flow Example

### Example: Voice Command "scan and grasp"

1. **User speaks**: "scan and grasp"
2. **arm_voice_node** recognizes and publishes to `/arm_command`: "scan_and_grasp"
3. **arm_command_interface** receives command:
   - Sends scan request to perception
4. **YOLO perception** detects objects:
   - Publishes to `/detected_objects`
5. **arm_grasper** plans grasp:
   - Computes grasp pose from object position
   - Requests motion plan from MoveIt
6. **MoveIt** plans and executes:
   - Plans collision-free trajectory
   - Executes on robot/simulation
7. **arm_command_interface** publishes result to `/arm_command_result`:
   - "Successfully grasped object at position (x, y, z)"
8. **arm_rviz_plugin** displays result in UI

## Package Dependencies

```
arm_bringup (launch coordinator)
    │
    ├─► arm_description (URDF models)
    │       └─► Used by: driver, MoveIt, Gazebo
    │
    ├─► arm_driver_node (hardware interface)
    │       └─► Depends on: serial communication
    │
    ├─► arm_moveit_config (motion planning)
    │       └─► Depends on: arm_description, MoveIt
    │
    ├─► arm_perception_yolo (object detection)
    │       └─► Depends on: YOLOv8, OpenCV, camera
    │
    ├─► arm_planning_py (high-level logic)
    │       └─► Depends on: arm_moveit_config, arm_perception_yolo
    │
    ├─► arm_voice_interface (voice control)
    │       └─► Depends on: Vosk, microphone
    │
    └─► arm_rviz_plugin (GUI control)
            └─► Depends on: RViz, Qt5
```

## Resource Requirements

### Simulation (sim_bringup)
- **CPU**: 4+ cores recommended
- **RAM**: 4-8 GB
- **GPU**: Optional (for Gazebo rendering)
- **Disk**: ~2 GB (models and dependencies)

### Real Robot (real_bringup)
- **CPU**: 2+ cores
- **RAM**: 2-4 GB
- **Camera**: USB camera or CSI camera
- **Serial**: USB-to-serial adapter

### Headless (voice_headless)
- **CPU**: 2 cores
- **RAM**: 1-2 GB
- **Microphone**: USB microphone
- **Storage**: ~500 MB (Vosk model)

## Configuration Priority

Launch arguments override config files:

```
1. Command line launch arguments (highest priority)
2. Launch file defaults
3. Config file values
4. Package defaults (lowest priority)
```

Example:
```bash
# Uses config file default
ros2 launch arm_bringup real_bringup.launch.py

# Overrides config file
ros2 launch arm_bringup real_bringup.launch.py serial_port:=/dev/ttyACM0
```

## Development Tips

### Adding New Commands
1. Add command handler in `arm_command_interface.py`
2. Update voice recognition keywords in `arm_voice_node.py`
3. Add button in RViz plugin if needed

### Debugging
```bash
# Check all active nodes
ros2 node list

# Check topic flow
ros2 topic list
ros2 topic echo /arm_command

# Check TF tree
ros2 run tf2_tools view_frames
```

### Performance Monitoring
```bash
# CPU usage per node
ros2 run demo_nodes_cpp cpu_usage

# Memory usage
htop
```

## Safety Considerations

1. **Emergency Stop**: Always have physical e-stop button
2. **Workspace Limits**: Configure MoveIt collision checking
3. **Speed Limits**: Set appropriate velocity/acceleration limits
4. **Permissions**: Limit serial port access to authorized users
5. **Simulation First**: Always test in simulation before real robot

---

Last Updated: 2025-10-21
