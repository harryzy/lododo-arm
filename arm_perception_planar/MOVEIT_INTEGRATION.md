# MoveIt Scene Integration

## Overview
The `arm_perception_planar` package automatically publishes detected cubes as `CollisionObject` messages to the MoveIt planning scene, enabling collision-aware motion planning.

## How It Works

### 1. Detection Flow
```
Camera Image → Cube Detection → 3D Localization → CollisionObject Publishing
```

When a cube is detected and localized:
1. Geometry-based detection identifies cube contours
2. Planar projection computes 3D position
3. CollisionObject is created with BOX primitive
4. Published to `/collision_object` topic
5. MoveIt automatically adds it to planning scene

### 2. CollisionObject Structure

Each detected cube is published as:
```yaml
id: "detected_cube|class=cube|idx=0"
header:
  frame_id: "base_link"
primitives:
  - type: BOX
    dimensions: [0.05, 0.05, 0.05]  # 5cm cube
primitive_poses:
  - position:
      x: 0.300
      y: 0.000
      z: 0.030
    orientation:
      w: 1.0
operation: ADD
```

### 3. Object Identification

Object ID format: `{object_id}|class=cube|idx={N}`

- `object_id`: Configurable base name (default: `detected_cube`)
- `class`: Object class (always `cube` for this package)
- `idx`: Sequential index (0, 1, 2, ...) for multiple cubes

**Example IDs:**
- Single cube: `detected_cube|class=cube|idx=0`
- Multiple cubes: `detected_cube|class=cube|idx=0`, `detected_cube|class=cube|idx=1`, ...

## Configuration

### Enable/Disable Scene Publishing

Edit `config/planar_params.yaml`:

```yaml
scene:
  object_id: detected_cube    # Base ID for collision objects
  publish_box: true           # Set to false to disable
```

### Disable at Runtime

```python
# Python node
self.set_parameters([Parameter('scene.publish_box', Parameter.Type.BOOL, False)])
```

```bash
# Command line
ros2 param set /planar_localization_node scene.publish_box false
```

## Usage with MoveIt

### 1. Launch Perception and MoveIt

```bash
# Terminal 1: Launch planar perception
ros2 launch arm_perception_planar planar_perception.launch.py

# Terminal 2: Launch MoveIt
ros2 launch arm_moveit_config demo.launch.py
```

### 2. Trigger Detection

```bash
# Send detection request
ros2 topic pub --once /detection_request std_msgs/String "data: 'grasp_site'"
```

### 3. Verify Scene Objects

```bash
# Check collision objects in scene
ros2 topic echo /collision_object

# Monitor planning scene
ros2 topic echo /planning_scene
```

### 4. Plan with Collision Avoidance

The detected cubes are now part of the planning scene. MoveIt will automatically avoid collisions with them.

```python
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import GetPlanningScene

# Get current scene (cubes are already included)
scene_client = node.create_client(GetPlanningScene, 'get_planning_scene')
request = GetPlanningScene.Request()
request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES

response = scene_client.call(request)
print(f"Scene objects: {response.scene.world.collision_objects}")
```

## Comparison with YOLO Package

| Feature | YOLO Package | Planar Perception Package |
|---------|--------------|---------------------------|
| CollisionObject Topic | `/collision_object` | `/collision_object` ✅ |
| Object ID Format | `detected_object\|class={N}\|label={name}` | `detected_cube\|class=cube\|idx={N}` |
| Primitive Type | BOX | BOX ✅ |
| Frame | `base_link` | `base_link` ✅ |
| Operation | ADD | ADD ✅ |
| Multi-object Support | ✅ | ✅ |

Both packages are fully compatible with MoveIt planning scene.

## Advanced Usage

### Remove Objects from Scene

To remove detected objects from the scene:

```python
from moveit_msgs.msg import CollisionObject

def remove_object(object_id: str):
    co = CollisionObject()
    co.id = object_id
    co.operation = CollisionObject.REMOVE
    collision_object_pub.publish(co)

# Remove specific cube
remove_object("detected_cube|class=cube|idx=0")
```

### Clear All Objects

```python
def clear_all_objects():
    co = CollisionObject()
    co.id = "detected_cube"  # Base ID
    co.operation = CollisionObject.REMOVE
    collision_object_pub.publish(co)
```

### Update Object Position

Objects are automatically updated when re-detected. If you need manual updates:

```python
# Re-publish with same ID but new position
co = CollisionObject()
co.id = "detected_cube|class=cube|idx=0"
co.operation = CollisionObject.MOVE  # or ADD
# ... set new position ...
collision_object_pub.publish(co)
```

## Troubleshooting

### Objects Not Appearing in RViz

1. Check if CollisionObject publishing is enabled:
   ```bash
   ros2 param get /planar_localization_node scene.publish_box
   ```

2. Verify messages are being published:
   ```bash
   ros2 topic hz /collision_object
   ```

3. In RViz, enable "Planning Scene" display:
   - Add display: `PlanningScene`
   - Topic: `/move_group/monitored_planning_scene`
   - Scene Display: Check "Show Scene Geometry"

### Objects Not Used in Planning

1. Ensure MoveIt node is subscribed to `/collision_object`:
   ```bash
   ros2 topic info /collision_object
   ```

2. Check planning scene contains objects:
   ```bash
   ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{}"
   ```

3. Verify object frame matches planning frame (should be `base_link`)

### Object IDs Conflicting

If you're running both YOLO and Planar packages simultaneously:

1. Change object_id in one package:
   ```yaml
   # planar_params.yaml
   scene:
     object_id: planar_cube  # Different from YOLO's "detected_object"
   ```

2. Or disable CollisionObject in one package:
   ```yaml
   scene:
     publish_box: false
   ```

## Integration Example

Complete example of using detected cubes in motion planning:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from moveit_msgs.msg import CollisionObject
import time

class GraspPlanner(Node):
    def __init__(self):
        super().__init__('grasp_planner')
        
        # Subscribe to collision objects
        self.sub_collision = self.create_subscription(
            CollisionObject,
            '/collision_object',
            self.collision_callback,
            10
        )
        
        # Publisher to trigger detection
        self.pub_request = self.create_publisher(
            String,
            '/detection_request',
            10
        )
        
        self.detected_objects = {}
    
    def collision_callback(self, msg):
        """Store detected objects"""
        self.get_logger().info(f'Received object: {msg.id}')
        if msg.operation == CollisionObject.ADD:
            self.detected_objects[msg.id] = msg
    
    def trigger_detection_and_plan(self, site_id='grasp_site'):
        """Trigger detection and plan grasp"""
        # Step 1: Clear previous objects
        self.detected_objects.clear()
        
        # Step 2: Trigger detection
        request = String()
        request.data = site_id
        self.pub_request.publish(request)
        self.get_logger().info('Detection triggered, waiting for objects...')
        
        # Step 3: Wait for objects to be added to scene
        time.sleep(1.0)
        
        # Step 4: Plan grasp based on detected objects
        if self.detected_objects:
            self.get_logger().info(f'Found {len(self.detected_objects)} objects')
            for obj_id, obj in self.detected_objects.items():
                pos = obj.primitive_poses[0].position
                self.get_logger().info(
                    f'Object {obj_id} at ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})'
                )
                # Now plan grasp to this position
                # MoveIt will automatically avoid collisions with this object
        else:
            self.get_logger().warn('No objects detected')

def main():
    rclpy.init()
    node = GraspPlanner()
    
    # Wait for initialization
    time.sleep(2.0)
    
    # Trigger detection and planning
    node.trigger_detection_and_plan()
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

The `arm_perception_planar` package provides seamless MoveIt integration through `CollisionObject` publishing:

- ✅ **Automatic**: Objects automatically added to scene upon detection
- ✅ **Compatible**: Same interface as YOLO package
- ✅ **Configurable**: Enable/disable and customize object IDs
- ✅ **Multi-object**: Supports multiple cubes with unique IDs
- ✅ **Real-time**: Objects updated as they're detected

This enables collision-aware motion planning without any additional code.
