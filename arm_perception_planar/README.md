# Planar Perception System

High-precision cube detection and localization system using planar constraints and geometric analysis.

## Features

- ✅ **YOLO-free detection**: Uses OpenCV geometric analysis for improved robustness
- ✅ **High localization accuracy**: XY plane ±3-5 mm, Z axis ±2-3 mm
- ✅ **Real-time processing**: <100 ms per frame
- ✅ **ROS2-standard tools**: Built on top of `camera_calibration` and `image_geometry`
- ✅ **Self-contained package**: Designed to integrate without affecting existing systems

````markdown
# Planar Perception System

High-precision cube detection and localization system using planar constraints and geometric analysis.

## Features

- ✅ **YOLO-free detection**: Uses OpenCV geometric analysis for improved robustness
- ✅ **High localization accuracy**: XY plane ±3-5 mm, Z axis ±2-3 mm
- ✅ **Real-time processing**: <100 ms per frame
- ✅ **ROS2-standard tools**: Built on top of `camera_calibration` and `image_geometry`
- ✅ **Self-contained package**: Designed to integrate without affecting existing systems

## Quick Start

### 1. Build

```bash
cd ~/lododo
colcon build --packages-select arm_perception_planar
source install/setup.bash
```

### 2. Camera Calibration (first-time setup)

See: [CALIBRATION_GUIDE.md](CALIBRATION_GUIDE.md) for full instructions.

```bash
# Run the calibration tool
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  --ros-args -r image:=/camera/image_raw
```

### 3. Run the system

```bash
# Launch the planar perception stack
ros2 launch arm_perception_planar planar_perception.launch.py

# View measured results
ros2 topic echo /planar/measured_objects

# Visualize in RViz2
rviz2
# Add topic: /planar/detection_markers
```

## System Architecture

```
Camera input → Geometric detection → Planar projection localization → 3D coordinates
  ↓              ↓                        ↓                 ↓
image_raw   cube_detector (OpenCV)  planar_localization (image_geometry)  measured_objects
```

## Configuration

Primary configuration: `config/planar_params.yaml`

Key parameters:
- `table_height`: Table height (in `base_link` frame)
- `cube_detection.expected_size`: Expected cube side length
- `camera_pose.*`: Camera extrinsic parameters
- `cube_detection.geometry_thresholds.*`: Geometry detection thresholds

## ROS2 Topics

| Topic | Type | Description |
|------:|:-----|:------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Input image stream |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |
| `/planar/cube_detections` | `vision_msgs/Detection2DArray` | 2D detection results |
| `/planar/measured_objects` | `arm_interfaces/MeasuredObject` | 3D localization results |
| `/planar/detection_markers` | `visualization_msgs/MarkerArray` | RViz markers |
| `/planar/debug_image` | `sensor_msgs/Image` | Debug visualization image |
| `/collision_object` | `moveit_msgs/CollisionObject` | Collision objects for MoveIt |
| `/detection_request` | `std_msgs/String` | Trigger detection request |
| `/detection_result` | `std_msgs/String` | JSON-formatted detection result |

## MoveIt Integration

Detected cubes are published as `CollisionObject` messages for MoveIt planning. Example configuration in `config/planar_params.yaml`:

```yaml
# configuration (config/planar_params.yaml)
scene:
  object_id: detected_cube    # Object ID prefix
  publish_box: true           # Enable/disable publishing
```

See: [MOVEIT_INTEGRATION.md](MOVEIT_INTEGRATION.md) for integration details.

Test script:
```bash
./test_collision_object.sh
```

## YOLO Compatibility

This package supports the same trigger-style interface used by existing YOLO-based systems:

```bash
# Trigger detection
ros2 topic pub --once /detection_request std_msgs/String "data: 'site_001'"

# Inspect JSON output
ros2 topic echo /detection_result
```

See: [INTERFACE_COMPATIBILITY.md](INTERFACE_COMPATIBILITY.md) for details.

## Comparison with Other Approaches

| Metric | Stereo/YOLO System | Planar Perception System |
|------:|:------------------|:------------------------|
| Detection method | YOLO classification | Geometric shape analysis |
| XY accuracy | ±5-10 mm | ±3-5 mm |
| Z accuracy | ±10-20 mm | ±2-3 mm |
| Measurement time | 2-3 s | <0.1 s |
| Small object detection | Poor | Good |

## Documentation

- [Design Document](../../PLANAR_PERCEPTION_DESIGN.md) - Full technical design
- [Calibration Guide](CALIBRATION_GUIDE.md) - Camera calibration steps
- [MoveIt Integration](MOVEIT_INTEGRATION.md) - Scene object integration
- [Interface Compatibility](INTERFACE_COMPATIBILITY.md) - YOLO-compatible interface

## License

Apache-2.0

## Maintainers

lododo-arm team

````
