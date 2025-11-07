# Distributed Deployment Configuration Notes

## Network Latency Optimizations

The configuration files have been optimized for distributed deployment where:
- Robot side runs on Raspberry Pi (with controllers)
- PC side runs MoveIt move_group
- Network latency exists between the two systems

## Key Parameter Changes

### moveit_controllers.yaml

1. **action_monitor_rate**: 50 → 20 Hz
   - Reduced frequency of action status checks
   - Tolerates network delays better

2. **wait_served_timeout**: 10.0 → 30.0 seconds
   - Allows more time for action server discovery over network
   - Critical for initial connection establishment

3. **allowed_execution_duration_scaling**: 10.0 → 15.0
   - Allows 15x longer execution time than nominal
   - Accommodates network delays during trajectory execution

4. **allowed_goal_duration_margin**: 5.0 → 10.0 seconds
   - Extra time margin for reaching goal position
   - Prevents premature timeout due to network latency

5. **allowed_start_tolerance**: 0.5 → 1.0
   - More tolerant of position differences at trajectory start
   - Accounts for state update delays

6. **publish_planning_scene_hz**: 4.0 → 2.0 Hz
   - Reduces network traffic for planning scene updates
   - Saves bandwidth on slower networks

### ros2_controllers.yaml

1. **action_monitor_rate**: 20.0 → 10.0 Hz
   - Less frequent monitoring on robot side
   - Reduces computational load on Raspberry Pi

2. **stopped_velocity_tolerance**: 0.1 → 0.2
   - More tolerant velocity check when stopping
   - Accounts for delayed state feedback

3. **Joint constraints** (trajectory/goal):
   - trajectory: 0.1 → 0.2
   - goal: 0.05 → 0.1
   - Increased tolerances for distributed execution

4. **goal_tolerance** (hand_controller): 0.01 → 0.02
   - More tolerant gripper goal position

## Troubleshooting

### "unknown result response, ignoring" Error

**Symptom**: Robot moves successfully but MoveIt reports failure

**Cause**: Network latency causes action result to arrive late or be dropped

**Solution**: Already applied in config files above. If still occurs:
- Check network quality (ping between PC and Raspberry Pi)
- Increase `wait_served_timeout` further
- Increase `allowed_execution_duration_scaling`

### "controller failed during execution" Warning

**Symptom**: Appears with above error but robot completes motion

**Cause**: MoveIt times out waiting for trajectory completion

**Solution**: 
- Increase `allowed_goal_duration_margin`
- Check if joint state feedback is publishing at good rate
- Verify ROS_DOMAIN_ID matches on both systems

### Performance Tips

1. **Reduce planning scene updates**: Lower `publish_planning_scene_hz`
2. **Simplify trajectories**: Use fewer waypoints
3. **Network optimization**: Use wired Ethernet instead of WiFi if possible
4. **DDS tuning**: Consider configuring DDS QoS for reliable communication

## Testing Distributed Setup

```bash
# On Raspberry Pi
ros2 launch arm_bringup robot_side.launch.py

# On PC (wait for robot side to start)
ros2 launch arm_bringup pc_side.launch.py

# Check action server availability
ros2 action list

# Monitor network latency
ping <raspberry-pi-ip>

# Check topic hz
ros2 topic hz /joint_states
```

## Reverting for Local Deployment

For local (non-distributed) deployment, you may want tighter tolerances:
- Restore original `action_monitor_rate` values (50, 20)
- Reduce timeouts back to original values
- Decrease trajectory/goal tolerances

Keep a backup of original config files if needed.
