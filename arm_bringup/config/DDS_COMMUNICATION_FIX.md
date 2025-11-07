# DDS Communication Fix for Distributed Deployment

## Problem

When running distributed deployment (PC + Raspberry Pi):
- **Symptom**: Robot executes trajectory successfully, but PC side reports errors:
  ```
  [ERROR] unknown goal response, ignoring...
  [ERROR] unknown result response, ignoring...
  [WARN] Controller failed during execution
  ```
- **Root Cause**: DDS (FastDDS) default QoS settings are not optimized for cross-machine Action communication
- **Result**: Action Goal/Result responses are lost or malformed during network transmission

## Solution

Added DDS QoS profile configuration (`dds_qos_profile.xml`) with:

### Key Improvements:

1. **Increased Socket Buffers**:
   - Send buffer: 1MB (prevents message dropping during bursts)
   - Receive buffer: 4MB (accommodates large trajectory messages)

2. **Reliable Communication**:
   - Reliability kind: RELIABLE (ensures delivery)
   - Durability: TRANSIENT_LOCAL (late joiners get recent messages)
   - History depth: 100 (increased from default 10)

3. **Increased Timeouts**:
   - Max blocking time: 5 seconds
   - Liveliness lease duration: 10 seconds

4. **Optimized UDP Transport**:
   - Max message size: 64KB
   - TTL: 16 hops (sufficient for LAN)

## Implementation

Both `pc_side.launch.py` and `robot_side.launch.py` now set:
```python
SetEnvironmentVariable(
    name='FASTRTPS_DEFAULT_PROFILES_FILE',
    value='<path>/dds_qos_profile.xml'
)
```

This environment variable tells FastDDS to use our custom QoS profile.

## Testing

After applying this fix:

```bash
# On Raspberry Pi
cd ~/lododo-arm
source install/setup.bash
ros2 launch arm_bringup robot_side.launch.py

# On PC
cd ~/lododo-arm
source install/setup.bash
ros2 launch arm_bringup pc_side.launch.py
```

Expected result:
- ✅ No "unknown goal/result response" errors
- ✅ MoveIt correctly reports trajectory execution success
- ✅ Smooth distributed operation

## Verification Commands

```bash
# Check if DDS profile is loaded
env | grep FASTRTPS

# Monitor Action communication
ros2 action list
ros2 action info /arm_controller/follow_joint_trajectory

# Test network latency
ping <raspberry-pi-ip>
# Should be < 50ms for good performance

# Check message flow
ros2 topic hz /joint_states
ros2 topic hz /trajectory_execution_event
```

## Troubleshooting

If errors still occur:

1. **Verify both sides use the profile**:
   ```bash
   # Should output the XML file path
   echo $FASTRTPS_DEFAULT_PROFILES_FILE
   ```

2. **Check network quality**:
   - Ping latency should be < 50ms
   - No packet loss
   - Use wired Ethernet if possible

3. **Increase buffer sizes** in `dds_qos_profile.xml`:
   ```xml
   <sendSocketBufferSize>2097152</sendSocketBufferSize>  <!-- 2MB -->
   <receiveSocketBufferSize>8388608</receiveSocketBufferSize>  <!-- 8MB -->
   ```

4. **Check ROS_DOMAIN_ID matches** on both machines:
   ```bash
   echo $ROS_DOMAIN_ID
   ```

## Technical Details

### Why Actions are Affected

ROS2 Actions consist of 3 message types:
1. **Goal** (PC → Robot): Request to execute trajectory
2. **Feedback** (Robot → PC): Progress updates during execution
3. **Result** (Robot → PC): Final execution status

With default DDS settings:
- Goal/Result may timeout or be dropped on slower networks
- Feedback messages can overflow small buffers
- Late-joining subscribers (like MoveIt) miss initial messages

### QoS Profile Hierarchy

```
FASTRTPS_DEFAULT_PROFILES_FILE (highest priority)
  ↓
DDS default QoS
  ↓
ROS2 QoS hints (in code)
```

Our XML file overrides defaults for all DDS communication.

## References

- [FastDDS QoS Documentation](https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/policy.html)
- [ROS2 DDS Tuning Guide](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html)
- [MoveIt2 Controller Manager](https://moveit.picknik.ai/main/doc/examples/controller_configuration/controller_configuration_tutorial.html)

## Related Configuration

This DDS fix complements the controller timeout adjustments in:
- `moveit_controllers.yaml` - MoveIt side timeouts
- `ros2_controllers.yaml` - Robot controller tolerances
- `DISTRIBUTED_DEPLOYMENT_NOTES.md` - Complete distributed deployment guide

All three must be configured for optimal distributed deployment performance.
