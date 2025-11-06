#!/bin/bash
# Quick diagnosis script for simulation status

echo "========================================="
echo "Simulation System Status Check"
echo "========================================="
echo ""

echo "1. Checking ROS nodes..."
ros2 node list | sort
echo ""

echo "2. Checking joint_states topic..."
ros2 topic info /joint_states
echo""

echo "3. Checking if joint_states are being published..."
timeout 2 ros2 topic echo /joint_states --once || echo "  ❌ No joint_states being published!"
echo ""

echo "4. Checking controllers..."
ros2 control list_controllers 2>/dev/null || echo "  ❌ Controller manager not available!"
echo ""

echo "5. Checking TF transforms..."
ros2 run tf2_ros tf2_echo base_link link5 2>/dev/null | head -n 5 || echo "  ❌ TF tree incomplete!"
echo ""

echo "6. Checking robot_description parameter..."
ros2 param get /robot_state_publisher robot_description | head -n 5
echo ""

echo "========================================="
echo "Status check complete!"
echo "========================================="
