#!/bin/bash
# Script to launch real_bringup and record logs

# Color definitions
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Configuration
WORKSPACE_DIR="$HOME/lododo-arm"
LOG_DIR="$WORKSPACE_DIR/logs/real_bringup"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/real_bringup_${TIMESTAMP}.log"
LATEST_LINK="$LOG_DIR/latest.log"

# Create log directory
mkdir -p "$LOG_DIR"

# Print information
echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN}Starting Real Arm Bringup${NC}"
echo -e "${GREEN}=========================================${NC}"
echo -e "${YELLOW}Time:${NC} $(date)"
echo -e "${YELLOW}Workspace:${NC} $WORKSPACE_DIR"
echo -e "${YELLOW}Log file:${NC} $LOG_FILE"
echo -e "${GREEN}=========================================${NC}"
echo ""

# Check if environment is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}Warning: ROS2 environment not loaded, trying to auto-load...${NC}"
    if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        source "$WORKSPACE_DIR/install/setup.bash"
        echo -e "${GREEN}✓ Environment loaded${NC}"
    else
        echo -e "${RED}✗ install/setup.bash not found${NC}"
        echo -e "${RED}Please build workspace first: colcon build${NC}"
        exit 1
    fi
fi

# Launch and record logs
echo "Starting... (Press Ctrl+C to stop)"
echo ""

# Use tee to output to both terminal and log file
ros2 launch arm_bringup real_bringup.launch.py 2>&1 | tee "$LOG_FILE"

# Save exit code
EXIT_CODE=${PIPESTATUS[0]}

# Create symlink to latest log
ln -sf "$LOG_FILE" "$LATEST_LINK"

echo ""
echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN}Launch has ended${NC}"
echo -e "${GREEN}=========================================${NC}"
echo -e "${YELLOW}Exit code:${NC} $EXIT_CODE"
echo -e "${YELLOW}Log saved to:${NC} $LOG_FILE"
echo -e "${YELLOW}Latest log link:${NC} $LATEST_LINK"
echo ""
echo -e "${YELLOW}View log:${NC} cat $LOG_FILE"
echo -e "${YELLOW}Live view:${NC} tail -f $LOG_FILE"
echo -e "${YELLOW}View errors:${NC} grep -i error $LOG_FILE"
echo -e "${GREEN}=========================================${NC}"

exit $EXIT_CODE
