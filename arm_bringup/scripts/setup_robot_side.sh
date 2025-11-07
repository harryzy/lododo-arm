#!/bin/bash#!/bin/bash#!/bin/bash

# Raspberry Pi Robot Side Configuration Script

# Author: lododo# Robot Side Quick Configuration Script

# Contact: contect@lododo.org

# # For configuring ROS2 distributed environment on Raspberry Pi

# This script configures a Raspberry Pi for running robot-side nodes

# in a distributed ROS2 deployment.



set -eset -eset -e



# Color codes for output

RED='\033[0;31m'

GREEN='\033[0;32m'echo "=========================================="echo "=========================================="

YELLOW='\033[1;33m'

BLUE='\033[0;34m'echo "  ROS2 Distributed Setup - Robot Side"

NC='\033[0m' # No Color

echo "=========================================="echo "=========================================="

echo -e "${BLUE}╔══════════════════════════════════════════════════════════╗${NC}"

echo -e "${BLUE}║  Lododo Arm - Robot Side Configuration Script           ║${NC}"echo ""echo ""

echo -e "${BLUE}║  For Raspberry Pi (Hardware Controller)                  ║${NC}"

echo -e "${BLUE}╚══════════════════════════════════════════════════════════╝${NC}"

echo ""

# Color definitions

# Get Raspberry Pi IP

RPI_IP=$(hostname -I | awk '{print $1}')RED='\033[0;31m'RED='\033[0;31m'

if [ -z "$RPI_IP" ]; then

    echo -e "${RED}Error: Could not detect Raspberry Pi IP address${NC}"GREEN='\033[0;32m'GREEN='\033[0;32m'

    exit 1

fiYELLOW='\033[1;33m'YELLOW='\033[1;33m'



echo -e "${GREEN}Detected Raspberry Pi IP: $RPI_IP${NC}"NC='\033[0m' # No ColorNC='\033[0m' # No Color

echo ""



# Prompt for PC IP

read -p "Enter PC IP address: " PC_IP# Get local IP

if [ -z "$PC_IP" ]; then

    echo -e "${RED}Error: PC IP address is required${NC}"ROBOT_IP=$(hostname -I | awk '{print $1}')ROBOT_IP=$(hostname -I | awk '{print $1}')

    exit 1

fiecho -e "${GREEN}Detected local IP: ${ROBOT_IP}${NC}"



# Prompt for ROS Domain IDecho ""echo ""

read -p "Enter ROS Domain ID (default: 0): " ROS_DOMAIN

ROS_DOMAIN=${ROS_DOMAIN:-0}



# Prompt for serial port# 1. Set ROS_DOMAIN_ID# 

read -p "Enter serial port for arm driver (default: /dev/ttyUSB0): " SERIAL_PORT

SERIAL_PORT=${SERIAL_PORT:-/dev/ttyUSB0}echo "================================================"echo "================================================"



# Prompt for camera serial numberecho "1. Configure ROS_DOMAIN_ID"echo "

read -p "Enter RealSense camera serial number (press Enter to auto-detect): " CAMERA_SERIAL

echo "================================================"echo "================================================"

echo ""

echo -e "${YELLOW}Configuration Summary:${NC}"read -p "Enter ROS_DOMAIN_ID (default: 0): " DOMAIN_ID

echo "  Raspberry Pi IP: $RPI_IP"

echo "  PC IP: $PC_IP"DOMAIN_ID=${DOMAIN_ID:-0}DOMAIN_ID=${DOMAIN_ID:-0}

echo "  ROS Domain ID: $ROS_DOMAIN"

echo "  Serial Port: $SERIAL_PORT"echo -e "${GREEN}ROS_DOMAIN_ID set to: ${DOMAIN_ID}${NC}"

echo "  Camera Serial: ${CAMERA_SERIAL:-auto-detect}"

echo ""echo ""echo ""

read -p "Proceed with configuration? (y/n): " CONFIRM

if [ "$CONFIRM" != "y" ]; then

    echo "Configuration cancelled."

    exit 0# 2. Create DDS configuration file

fi

echo "================================================"echo "================================================"

# Create DDS configuration

echo ""echo "2. Create FastDDS Configuration"echo "

echo -e "${BLUE}Creating FastDDS configuration...${NC}"

mkdir -p ~/.rosecho "================================================"echo "================================================"



cat > ~/.ros/fastdds.xml << 'EOF'mkdir -p ~/.rosmkdir -p ~/.ros

<?xml version="1.0" encoding="UTF-8" ?>

<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">

    <participant profile_name="participant_profile" is_default_profile="true">

        <rtps>cat > ~/.ros/fastdds.xml << 'EOF'cat > ~/.ros/fastdds.xml << 'EOF'

            <builtin>

                <discovery_config><?xml version="1.0" encoding="UTF-8" ?><?xml version="1.0" encoding="UTF-8" ?>

                    <discoveryProtocol>SIMPLE</discoveryProtocol>

                    <leaseDuration><profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"><profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">

                        <sec>30</sec>

                    </leaseDuration>    <participant profile_name="participant_profile" is_default_profile="true">    <participant profile_name="participant_profile" is_default_profile="true">

                </discovery_config>

            </builtin>        <rtps>        <rtps>

        </rtps>

    </participant>            <builtin>            <builtin>

</profiles>

EOF                <discovery_config>                <discovery_config>



echo -e "${GREEN}✓ FastDDS configuration created at ~/.ros/fastdds.xml${NC}"                    <discoveryProtocol>SIMPLE</discoveryProtocol>                    <discoveryProtocol>SIMPLE</discoveryProtocol>



# Add environment variables to ~/.bashrc if not already present                    <leaseDuration>                    <leaseDuration>

echo ""

echo -e "${BLUE}Configuring ROS2 environment variables...${NC}"                        <sec>30</sec>                        <sec>30</sec>



BASHRC_ADDITIONS="                    </leaseDuration>                    </leaseDuration>

# ROS2 Distributed Deployment Configuration (Lododo Arm - Robot Side)

source /opt/ros/humble/setup.bash                </discovery_config>                </discovery_config>

[ -f ~/lododo-arm/install/setup.bash ] && source ~/lododo-arm/install/setup.bash

export ROS_DOMAIN_ID=$ROS_DOMAIN            </builtin>            </builtin>

export ROS_LOCALHOST_ONLY=0

export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml        </rtps>        </rtps>

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

"    </participant>    </participant>



if ! grep -q "ROS2 Distributed Deployment Configuration (Lododo Arm - Robot Side)" ~/.bashrc; then</profiles></profiles>

    echo "$BASHRC_ADDITIONS" >> ~/.bashrc

    echo -e "${GREEN}✓ Environment variables added to ~/.bashrc${NC}"EOFEOF

else

    echo -e "${YELLOW}! Environment variables already exist in ~/.bashrc${NC}"

fi

echo -e "${GREEN}✓ FastDDS configuration file created${NC}"echo -e "${GREEN}✓ FastDDS 配置文件已创建${NC}"

# Create startup script

echo ""echo ""echo ""

echo -e "${BLUE}Creating startup script...${NC}"



cat > ~/start_robot.sh << EOF

#!/bin/bash# 3. Configure environment variables

# Lododo Arm Robot Side Startup Script

# Generated by setup_robot_side.sh
echo "================================================"
echo "================================================"

source /opt/ros/humble/setup.bash
echo "3. Configure Environment Variables"

cd ~/lododo-arm

source install/setup.bash
echo "================================================"
echo "================================================"



echo "Starting Lododo Arm Robot Side..."

echo "  Raspberry Pi IP: $RPI_IP"

echo "  PC IP: $PC_IP"# Backup original .bashrc

echo "  ROS Domain ID: $ROS_DOMAIN"

echo "  Serial Port: $SERIAL_PORT"cp ~/.bashrc ~/.bashrc.backup_$(date +%Y%m%d_%H%M%S)cp ~/.bashrc ~/.bashrc.backup_$(date +%Y%m%d_%H%M%S)

echo ""



ros2 launch arm_bringup robot_side.launch.py \\

    port:=$SERIAL_PORT \\# Remove old ROS2 configuration (if exists)

    enable_camera:=true \\

    camera_serial_no:=$CAMERA_SERIAL \\sed -i '/# ROS2 Distributed Setup - Robot Side/,/# End of ROS2 Distributed Setup/d' ~/.bashrcsed -i '/# ROS2 Distributed Setup - Robot Side/,/# End of ROS2 Distributed Setup/d' ~/.bashrc

    ros_domain_id:=$ROS_DOMAIN

EOF



chmod +x ~/start_robot.sh# Add new configuration

echo -e "${GREEN}✓ Startup script created at ~/start_robot.sh${NC}"

cat >> ~/.bashrc << ENVEOFcat >> ~/.bashrc << EOF

# Display next steps

echo ""

echo -e "${BLUE}╔══════════════════════════════════════════════════════════╗${NC}"

echo -e "${BLUE}║  Configuration Complete!                                 ║${NC}"# ROS2 Distributed Setup - Robot Side (added on $(date))
echo -e "${BLUE}╚══════════════════════════════════════════════════════════╝${NC}"

echo ""source /opt/ros/humble/setup.bashsource /opt/ros/humble/setup.bash

echo -e "${GREEN}Next steps:${NC}"

echo ""[ -f ~/lododo-arm/install/setup.bash ] && source ~/lododo-arm/install/setup.bash[ -f ~/lododo-arm/install/setup.bash ] && source ~/lododo-arm/install/setup.bash

echo "1. Reload environment:"

echo "   source ~/.bashrc"

echo ""

echo "2. Test connection to PC:"# DDS Configuration

echo "   ping $PC_IP"

echo ""export ROS_DOMAIN_ID=${DOMAIN_ID}export ROS_DOMAIN_ID=${DOMAIN_ID}

echo "3. Start robot side:"

echo "   ~/start_robot.sh"export ROS_LOCALHOST_ONLY=0export ROS_LOCALHOST_ONLY=0

echo ""

echo -e "${YELLOW}Note: Make sure PC side is configured with same ROS_DOMAIN_ID ($ROS_DOMAIN)${NC}"export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xmlexport FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml

echo ""

export RMW_IMPLEMENTATION=rmw_fastrtps_cppexport RMW_IMPLEMENTATION=rmw_fastrtps_cpp



# Robot IP# 串口权限

export ROBOT_IP=${ROBOT_IP}export ROBOT_IP=${ROBOT_IP}



# Aliases# 别名

alias robot-start='ros2 launch arm_bringup robot_side.launch.py'alias robot-start='ros2 launch arm_bringup robot_side.launch.py'

alias robot-check='ros2 topic list && ros2 node list'alias robot-check='ros2 topic list && ros2 node list'

# End of ROS2 Distributed Setup# End of ROS2 Distributed Setup

ENVEOFEOF



echo -e "${GREEN}✓ Environment variables configured${NC}"echo -e "${GREEN}✓ 环境变量已配置${NC}"

echo ""echo ""



# 4. Set serial port permissions# 4. 设置串口权限

echo "================================================"echo "================================================"

echo "4. Configure Serial Port Permissions"echo "4. 设置串口权限"

echo "================================================"echo "================================================"

if groups | grep -q dialout; thenif groups | grep -q dialout; then

    echo -e "${GREEN}✓ User already in dialout group${NC}"    echo -e "${GREEN}✓ 用户已在 dialout 组中${NC}"

elseelse

    sudo usermod -a -G dialout $USER    sudo usermod -a -G dialout $USER

    echo -e "${YELLOW}⚠ Added to dialout group. Please log out and log back in for changes to take effect${NC}"    echo -e "${YELLOW}⚠ 已添加到 dialout 组，需要注销后重新登录生效${NC}"

fifi

echo ""echo ""



# 5. Check serial devices# 5. 检查串口设备

echo "================================================"echo "================================================"

echo "5. Check Serial Devices"echo "5. 检查串口设备"

echo "================================================"echo "================================================"

if ls /dev/ttyUSB* 1> /dev/null 2>&1; thenif ls /dev/ttyUSB* 1> /dev/null 2>&1; then

    echo -e "${GREEN}Found serial devices:${NC}"    echo -e "${GREEN}发现串口设备:${NC}"

    ls -l /dev/ttyUSB*    ls -l /dev/ttyUSB*

elseelse

    echo -e "${YELLOW}⚠ No USB serial devices found. Please ensure servo controller is connected${NC}"    echo -e "${YELLOW}⚠ 未发现 USB 串口设备，请确认舵机控制器已连接${NC}"

fifi

echo ""echo ""



# 6. Check camera# 6. 检查相机

echo "================================================"echo "================================================"

echo "6. Check RealSense Camera"echo "6. 检查 RealSense 相机"

echo "================================================"echo "================================================"

if command -v rs-enumerate-devices &> /dev/null; thenif command -v rs-enumerate-devices &> /dev/null; then

    echo "Running camera detection..."    echo "运行相机检测..."

    if rs-enumerate-devices | grep -q "Device"; then    if rs-enumerate-devices | grep -q "Device"; then

        echo -e "${GREEN}✓ RealSense camera detected${NC}"        echo -e "${GREEN}✓ 发现 RealSense 相机${NC}"

        rs-enumerate-devices | grep -E "Device|Serial|Name"        rs-enumerate-devices | grep -E "Device|Serial|Name"

    else    else

        echo -e "${YELLOW}⚠ No camera devices found${NC}"        echo -e "${YELLOW}⚠ 未发现相机设备${NC}"

    fi    fi

elseelse

    echo -e "${YELLOW}⚠ realsense2 tools not installed, skipping camera detection${NC}"    echo -e "${YELLOW}⚠ realsense2 工具未安装，跳过相机检测${NC}"

fifi

echo ""echo ""



# 7. Firewall configuration# 7. 防火墙配置

echo "================================================"echo "================================================"

echo "7. Configure Firewall"echo "7. 配置防火墙"

echo "================================================"echo "================================================"

if command -v ufw &> /dev/null; thenif command -v ufw &> /dev/null; then

    UFW_STATUS=$(sudo ufw status | head -1)    UFW_STATUS=$(sudo ufw status | head -1)

    if echo "$UFW_STATUS" | grep -q "inactive"; then    if echo "$UFW_STATUS" | grep -q "inactive"; then

        echo -e "${GREEN}✓ Firewall is inactive, no configuration needed${NC}"        echo -e "${GREEN}✓ 防火墙未启用，无需配置${NC}"

    else    else

        echo "Firewall is active, configuring ROS2 ports..."        echo "防火墙已启用，正在配置 ROS2 端口..."

        sudo ufw allow 7400:7500/udp comment "ROS2 DDS"        sudo ufw allow 7400:7500/udp comment "ROS2 DDS"

        sudo ufw allow 7400:7500/tcp comment "ROS2 DDS"        sudo ufw allow 7400:7500/tcp comment "ROS2 DDS"

        echo -e "${GREEN}✓ Firewall rules added${NC}"        echo -e "${GREEN}✓ 防火墙规则已添加${NC}"

    fi    fi

elseelse

    echo -e "${GREEN}✓ UFW not installed, skipping firewall configuration${NC}"    echo -e "${GREEN}✓ 未安装 UFW，跳过防火墙配置${NC}"

fifi

echo ""echo ""



# 8. Generate startup script# 8. 生成启动脚本

echo "================================================"echo "================================================"

echo "8. Generate Quick Start Script"echo "8. 生成快速启动脚本"

echo "================================================"echo "================================================"



cat > ~/start_robot.sh << 'SCRIPT'cat > ~/start_robot.sh << 'SCRIPT'

#!/bin/bash#!/bin/bash

# Robot side quick start script# 树莓派端快速启动脚本



source ~/.bashrcsource ~/.bashrc

cd ~/lododo-armcd ~/lododo-arm



# Check serial port# 检查串口

if [ ! -e /dev/ttyUSB0 ]; thenif [ ! -e /dev/ttyUSB0 ]; then

    echo "Error: Serial device /dev/ttyUSB0 not found"    echo "错误: 未找到串口设备 /dev/ttyUSB0"

    echo "Available devices:"    echo "可用设备:"

    ls /dev/ttyUSB* 2>/dev/null || echo "None"    ls /dev/ttyUSB* 2>/dev/null || echo "无"

    exit 1    exit 1

fifi



# Start robot side# 启动

echo "Starting robot side..."echo "正在启动机器人端..."

ros2 launch arm_bringup robot_side.launch.py port:=/dev/ttyUSB0ros2 launch arm_bringup robot_side.launch.py port:=/dev/ttyUSB0

SCRIPTSCRIPT



chmod +x ~/start_robot.shchmod +x ~/start_robot.sh

echo -e "${GREEN}✓ Startup script created: ~/start_robot.sh${NC}"echo -e "${GREEN}✓ 启动脚本已创建: ~/start_robot.sh${NC}"

echo ""echo ""



# Completion# 完成

echo "=========================================="echo "=========================================="

echo -e "${GREEN}Configuration Complete!${NC}"echo -e "${GREEN}配置完成！${NC}"

echo "=========================================="echo "=========================================="

echo ""echo ""

echo "Next steps:"echo "后续步骤:"

echo "1. Reload environment: source ~/.bashrc"echo "1. 重新加载环境: source ~/.bashrc"

echo "2. Or log out and log back in"echo "2. 或注销后重新登录"

echo "3. Run quick start: ~/start_robot.sh"echo "3. 运行快速启动: ~/start_robot.sh"

echo "   Or use alias: robot-start"echo "   或使用别名: robot-start"

echo ""echo ""

echo "Verification commands:"echo "验证命令:"

echo "  - View nodes: ros2 node list"echo "  - 查看节点: ros2 node list"

echo "  - View topics: ros2 topic list"echo "  - 查看话题: ros2 topic list"

echo "  - Test multicast: ros2 multicast receive"echo "  - 测试多播: ros2 multicast receive"

echo ""echo ""

echo -e "${YELLOW}Note: Ensure PC side is configured with the same ROS_DOMAIN_ID (${DOMAIN_ID})${NC}"echo -e "${YELLOW}提示: 确保 PC 端也配置了相同的 ROS_DOMAIN_ID (${DOMAIN_ID})${NC}"

echo ""echo ""

