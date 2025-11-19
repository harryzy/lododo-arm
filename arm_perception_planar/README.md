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

Build this package in your ROS2 workspace:

```bash
colcon build --packages-select arm_perception_planar
source install/setup.bash
```

### 2. Run

Launch the perception system:

```bash
ros2 launch arm_perception_planar planar_perception.launch.py
```

Visualize detections in RViz:

```bash
rviz2
# Add topic: /planar/detection_markers
```

## System Architecture

```
Camera input → Geometric detection → Planar projection → 3D localization
     ↓                  ↓                    ↓                 ↓
image_raw      cube_detector (OpenCV)   planar_localization   measured_objects
```

## Configuration

Primary configuration file: `config/planar_params.yaml`

### Key Parameters

- **`table_height`**: Table height in `base_link` frame (meters)
- **`cube_detection.expected_size`**: Expected cube side length (meters)
- **`camera_pose.*`**: Camera extrinsic parameters (position and orientation)
- **`cube_detection.geometry_thresholds.*`**: Geometry detection thresholds

## ROS2 Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Input camera image stream |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsic parameters |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/planar/cube_detections` | `vision_msgs/Detection2DArray` | 2D detection results |
| `/planar/measured_objects` | `arm_interfaces/MeasuredObject` | 3D localization results |
| `/planar/detection_markers` | `visualization_msgs/MarkerArray` | RViz visualization markers |
| `/planar/debug_image` | `sensor_msgs/Image` | Debug visualization image |
| `/collision_object` | `moveit_msgs/CollisionObject` | MoveIt collision objects |

### Service Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/detection_request` | `std_msgs/String` | Trigger detection request |
| `/detection_result` | `std_msgs/String` | JSON-formatted detection result |

## MoveIt Integration

Detected cubes are automatically published as `CollisionObject` messages for MoveIt planning.

Configuration in `config/planar_params.yaml`:

```yaml
scene:
  object_id: detected_cube    # Object ID prefix
  publish_box: true           # Enable/disable publishing
```

For detailed integration steps, see [MOVEIT_INTEGRATION.md](MOVEIT_INTEGRATION.md)

Test collision object integration:

```bash
./test_collision_object.sh
```

## YOLO Compatibility

This package provides a compatible interface with existing YOLO-based detection systems.

### Trigger Detection

```bash
ros2 topic pub --once /detection_request std_msgs/String "data: 'site_001'"
```

### Check Results

```bash
ros2 topic echo /detection_result
```

For details, see [INTERFACE_COMPATIBILITY.md](INTERFACE_COMPATIBILITY.md)

## Performance Comparison

| Metric | Stereo/YOLO System | Planar Perception System |
|--------|-------------------|--------------------------|
| Detection method | YOLO classification | Geometric shape analysis |
| XY accuracy | ±5-10 mm | ±3-5 mm |
| Z accuracy | ±10-20 mm | ±2-3 mm |
| Measurement time | 2-3 s | <0.1 s |
| Small object detection | Poor | Good |

## Documentation

- [Design Document](../../PLANAR_PERCEPTION_DESIGN.md) - Complete technical design
- [Calibration Guide](CALIBRATION_GUIDE.md) - Camera calibration procedures
- [MoveIt Integration](MOVEIT_INTEGRATION.md) - Scene object integration details
- [Interface Compatibility](INTERFACE_COMPATIBILITY.md) - YOLO-compatible interface

## License

Apache-2.0

## Maintainers

lododo-arm team
