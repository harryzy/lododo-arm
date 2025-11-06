#!/bin/bash
# PC Side Quick Configuration Script
# For configuring ROS2 distributed environment on PC workstation

set -e

echo "=========================================="
echo "  ROS2 Distributed Setup - PC Side"
echo "=========================================="
echo ""

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get local IP
PC_IP=$(hostname -I | awk '{print $1}')
echo -e "${GREEN}Detected local IP: ${PC_IP}${NC}"
echo ""

# 1. Set ROS_DOMAIN_ID
echo "================================================"
echo "1. Configure ROS_DOMAIN_ID"
echo "================================================"
read -p "Enter ROS_DOMAIN_ID (default: 0, must match robot side): " DOMAIN_ID
DOMAIN_ID=${DOMAIN_ID:-0}
echo -e "${GREEN}ROS_DOMAIN_ID set to: ${DOMAIN_ID}${NC}"
echo ""

# 2. Input robot IP
echo "================================================"
echo "2. Configure Robot Connection"
echo "================================================"
read -p "Enter Raspberry Pi IP address: " ROBOT_IP
if [ -z "$ROBOT_IP" ]; then
    echo -e "${RED}Error: Robot IP cannot be empty${NC}"
    exit 1
fi

# Test connectivity
echo "Testing connection to robot..."
if ping -c 2 -W 2 $ROBOT_IP > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Can reach robot at ${ROBOT_IP}${NC}"
else
    echo -e "${YELLOW}⚠ Cannot ping robot. Please check network connection${NC}"
fi
echo ""

# 3. Create DDS configuration file
echo "================================================"
echo "3. Create FastDDS Configuration"
echo "================================================"
mkdir -p ~/.ros

cat > ~/.ros/fastdds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>SIMPLE</discoveryProtocol>
                    <leaseDuration>
                        <sec>30</sec>
                    </leaseDuration>
                </discovery_config>
            </builtin>
        </rtps>
    </participant>
</profiles>
EOF

echo -e "${GREEN}✓ FastDDS configuration file created${NC}"
echo ""

# 4. Configure environment variables
echo "================================================"
echo "4. Configure Environment Variables"
echo "================================================"

# Backup original .bashrc
cp ~/.bashrc ~/.bashrc.backup_$(date +%Y%m%d_%H%M%S)

# Remove old ROS2 configuration (if exists)
sed -i '/# ROS2 Distributed Setup - PC Side/,/# End of ROS2 Distributed Setup/d' ~/.bashrc

# Add new configuration
cat >> ~/.bashrc << ENVEOF

# ROS2 Distributed Setup - PC Side (added on $(date))
source /opt/ros/humble/setup.bash
[ -f ~/lododo-arm/install/setup.bash ] && source ~/lododo-arm/install/setup.bash

# DDS Configuration
export ROS_DOMAIN_ID=${DOMAIN_ID}
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Robot connection
export ROBOT_IP=${ROBOT_IP}
export PC_IP=${PC_IP}

# Aliases
alias pc-start='ros2 launch arm_bringup pc_side.launch.py'
alias pc-start-rviz='ros2 launch arm_bringup pc_side.launch.py rviz:=true'
alias pc-start-voice='ros2 launch arm_bringup pc_side.launch.py voice_control:=true'
alias pc-check='ros2 topic list && ros2 node list'
alias robot-ssh='ssh ubuntu@${ROBOT_IP}'
# End of ROS2 Distributed Setup
ENVEOF

echo -e "${GREEN}✓ Environment variables configured${NC}"
echo ""

# 5. Firewall configuration
echo "================================================"
echo "5. Configure Firewall"
echo "================================================"
if command -v ufw &> /dev/null; then
    UFW_STATUS=$(sudo ufw status | head -1)
    if echo "$UFW_STATUS" | grep -q "inactive"; then
        echo -e "${GREEN}✓ Firewall is inactive, no configuration needed${NC}"
    else
        echo "Firewall is active, configuring ROS2 ports..."
        sudo ufw allow 7400:7500/udp comment "ROS2 DDS"
        sudo ufw allow 7400:7500/tcp comment "ROS2 DDS"
        echo -e "${GREEN}✓ Firewall rules added${NC}"
    fi
else
    echo -e "${GREEN}✓ UFW not installed, skipping firewall configuration${NC}"
fi
echo ""

# 6. Check GPU (for YOLO)
echo "================================================"
echo "6. Check GPU Status"
echo "================================================"
if command -v nvidia-smi &> /dev/null; then
    echo "NVIDIA GPU Information:"
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
    echo -e "${GREEN}✓ NVIDIA GPU detected, YOLO will use GPU acceleration${NC}"
else
    echo -e "${YELLOW}⚠ NVIDIA GPU not detected, YOLO will use CPU${NC}"
fi
echo ""

# 7. Generate startup scripts
echo "================================================"
echo "7. Generate Quick Start Scripts"
echo "================================================"

# PC side startup script
cat > ~/start_pc.sh << 'SCRIPT'
#!/bin/bash
# PC side quick start script

source ~/.bashrc
cd ~/lododo-arm

# Check robot connection
echo "Checking connection to robot..."
if ! ping -c 1 -W 2 $ROBOT_IP > /dev/null 2>&1; then
    echo "Warning: Cannot ping robot ($ROBOT_IP)"
    echo "Please ensure robot is started and on the same network"
    read -p "Continue anyway? (y/N): " CONTINUE
    if [ "$CONTINUE" != "y" ] && [ "$CONTINUE" != "Y" ]; then
        exit 1
    fi
fi

# Important reminder
echo ""
echo "=========================================="
echo "⚠️  IMPORTANT: Two Terminals Required"
echo "=========================================="
echo ""
echo "Terminal 1 (this one): PC control side"
echo "Terminal 2 (separate): YOLO perception"
echo ""
echo "After this terminal starts, open another terminal and run:"
echo "  Option 1 (Recommended): ros2 run arm_bringup start_yolo_cube_detect.sh"
echo "  Option 2 (Advanced):    ros2 run arm_bringup start_yolo_dual_view.sh"
echo "  Option 3 (Basic):       ros2 run arm_bringup start_yolo_node.sh"
echo ""
echo "Note: Cube detection (Option 1) is recommended for reliable grasping."
echo "      Dual-view (Option 2) provides advanced triangulation measurement."
echo "      Single-view (Option 3) is basic mode requiring camera tuning."
echo ""
echo "Press Enter to continue..."
read

# Start PC side
echo "Starting PC side..."
ros2 launch arm_bringup pc_side.launch.py rviz:=true
SCRIPT

chmod +x ~/start_pc.sh
echo -e "${GREEN}✓ Startup script created: ~/start_pc.sh${NC}"

# Create connection test script
cat > ~/test_robot_connection.sh << 'SCRIPT'
#!/bin/bash
# Test ROS2 connection to robot

source ~/.bashrc

echo "=========================================="
echo "  ROS2 Distributed Connection Test"
echo "=========================================="
echo ""

# 1. Network connectivity
echo "1. Testing network connectivity..."
if ping -c 2 -W 2 $ROBOT_IP > /dev/null 2>&1; then
    echo "   ✓ Network connection OK"
else
    echo "   ✗ Cannot reach robot"
    exit 1
fi
echo ""

# 2. ROS2 configuration
echo "2. Checking ROS2 configuration..."
echo "   ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "   ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
echo "   RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo ""

# 3. Node discovery
echo "3. Checking ROS2 nodes..."
NODES=$(ros2 node list 2>/dev/null | wc -l)
echo "   Found $NODES nodes"
if [ $NODES -gt 0 ]; then
    ros2 node list
else
    echo "   ✗ No nodes found. Please check:"
    echo "     - Robot side is started"
    echo "     - ROS_DOMAIN_ID matches"
    echo "     - Firewall settings"
fi
echo ""

# 4. Topic check
echo "4. Checking key topics..."
TOPICS=("/joint_states" "/camera/color/image_raw" "/tf")
for TOPIC in "${TOPICS[@]}"; do
    if ros2 topic list 2>/dev/null | grep -q "^${TOPIC}$"; then
        HZ=$(timeout 3 ros2 topic hz $TOPIC 2>&1 | grep "average rate" | awk '{print $3}')
        if [ -n "$HZ" ]; then
            echo "   ✓ $TOPIC (${HZ} Hz)"
        else
            echo "   ⚠ $TOPIC (exists but no data)"
        fi
    else
        echo "   ✗ $TOPIC (not found)"
    fi
done
echo ""

echo "=========================================="
echo "Test Complete"
echo "=========================================="
SCRIPT

chmod +x ~/test_robot_connection.sh
echo -e "${GREEN}✓ Test script created: ~/test_robot_connection.sh${NC}"
echo ""

# Completion
echo "=========================================="
echo -e "${GREEN}Configuration Complete!${NC}"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Reload environment: source ~/.bashrc"
echo "2. Test connection: ~/test_robot_connection.sh"
echo "3. Start PC side: ~/start_pc.sh"
echo "   Or use alias: pc-start"
echo ""
echo "Available aliases:"
echo "  - pc-start       : Start PC side (no RViz)"
echo "  - pc-start-rviz  : Start PC side (with RViz)"
echo "  - pc-start-voice : Start PC side (with voice control)"
echo "  - pc-check       : Check nodes and topics"
echo "  - robot-ssh      : SSH to robot"
echo ""
echo -e "${YELLOW}Important Notes:${NC}"
echo "1. Ensure robot side has the same ROS_DOMAIN_ID (${DOMAIN_ID})"
echo "2. Start robot side before starting PC side"
echo "3. Use ~/test_robot_connection.sh to verify connection"
echo ""
echo -e "${YELLOW}⚠️  TWO TERMINALS REQUIRED:${NC}"
echo "  Terminal 1: ros2 launch arm_bringup pc_side.launch.py"
echo "  Terminal 2 (choose one):"
echo "    - start_yolo_cube_detect.sh  (recommended for grasping)"
echo "    - start_yolo_dual_view.sh    (advanced triangulation)"
echo "    - start_yolo_node.sh         (basic single-view)"
echo ""
echo "YOLO perception must run in separate terminal with isolated environment!"
echo ""
