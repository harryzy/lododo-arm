#!/bin/bash
# Test script for CollisionObject publishing

echo "=========================================="
echo "Testing CollisionObject Publishing"
echo "=========================================="
echo ""

# Check if node is running
echo "1. Checking if planar_localization_node is running..."
if ros2 node list | grep -q planar_localization_node; then
    echo "   ✓ Node is running"
else
    echo "   ✗ Node not found. Please launch planar_perception first:"
    echo "     ros2 launch arm_perception_planar planar_perception.launch.py"
    exit 1
fi
echo ""

# Check CollisionObject publisher
echo "2. Checking /collision_object topic..."
if ros2 topic list | grep -q "/collision_object"; then
    echo "   ✓ Topic exists"
    echo "   Publishers:"
    ros2 topic info /collision_object | grep "Publisher count"
else
    echo "   ✗ Topic not found"
    exit 1
fi
echo ""

# Check configuration
echo "3. Checking scene configuration..."
PUBLISH_BOX=$(ros2 param get /planar_localization_node scene.publish_box 2>/dev/null)
OBJECT_ID=$(ros2 param get /planar_localization_node scene.object_id 2>/dev/null)

if [[ $PUBLISH_BOX == *"true"* ]]; then
    echo "   ✓ CollisionObject publishing enabled"
else
    echo "   ✗ CollisionObject publishing disabled"
    echo "     Enable with: ros2 param set /planar_localization_node scene.publish_box true"
fi

echo "   Object ID: $OBJECT_ID"
echo ""

# Trigger detection and monitor
echo "4. Triggering detection..."
echo "   Sending request to /detection_request..."
ros2 topic pub --once /detection_request std_msgs/String "data: 'test_collision'" &
PID=$!

echo "   Monitoring /collision_object for 5 seconds..."
echo "   (Place a cube in camera view if not already present)"
echo ""

timeout 5 ros2 topic echo /collision_object --once 2>/dev/null

if [ $? -eq 0 ]; then
    echo ""
    echo "   ✓ CollisionObject received!"
    echo ""
    echo "=========================================="
    echo "Test PASSED ✓"
    echo "=========================================="
    echo ""
    echo "Next steps:"
    echo "  1. Launch RViz: ros2 launch arm_moveit_config demo.launch.py"
    echo "  2. Add 'PlanningScene' display in RViz"
    echo "  3. Enable 'Show Scene Geometry' to see detected cubes"
else
    echo ""
    echo "   ⚠ No CollisionObject received within timeout"
    echo "   This could mean:"
    echo "     - No cube detected in camera view"
    echo "     - Camera not publishing images"
    echo "     - Detection node not running"
    echo ""
    echo "   Try:"
    echo "     - Check camera: ros2 topic hz /camera/image_raw"
    echo "     - Check detections: ros2 topic echo /planar/cube_detections"
    echo "     - Place cube in camera field of view"
fi

wait $PID 2>/dev/null

echo ""
echo "For continuous monitoring:"
echo "  ros2 topic echo /collision_object"
echo ""
