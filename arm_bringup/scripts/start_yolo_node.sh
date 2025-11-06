#!/bin/bash
# Wrapper script to launch YOLO perception node in virtual environment
# This is needed because YOLO dependencies conflict with MoveIt packages

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Starting YOLO Perception Node${NC}"
echo -e "${GREEN}========================================${NC}"

# Default virtual environment path
VENV_PATH="${HOME}/yolo_venv"

# Check if custom path is provided
if [ ! -z "$1" ]; then
    VENV_PATH="$1"
fi

# Check if virtual environment exists
if [ ! -d "$VENV_PATH" ]; then
    echo -e "${YELLOW}Virtual environment not found at: $VENV_PATH${NC}"
    echo -e "${YELLOW}Creating virtual environment with system site-packages access...${NC}"
    
    # Create venv with access to system site-packages (for ROS2)
    python3 -m venv --system-site-packages "$VENV_PATH"
    
    echo -e "${GREEN}Installing required packages (CPU-only versions)...${NC}"
    source "$VENV_PATH/bin/activate"
    
    pip install --upgrade pip
    
    # Install PyTorch CPU-only version (much smaller, no CUDA)
    echo -e "${YELLOW}Installing PyTorch CPU version...${NC}"
    pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
    
    # Install other dependencies
    echo -e "${YELLOW}Installing YOLO and other packages...${NC}"
    # Use NumPy 1.x for compatibility with cv_bridge and opencv
    pip install "numpy<2.0" opencv-python-headless
    pip install ultralytics --no-deps  # Install without dependencies to avoid reinstalling torch with CUDA
    
    # Install missing ultralytics dependencies manually (without torch)
    pip install PyYAML pillow scipy matplotlib requests psutil pandas
    
    echo -e "${GREEN}Virtual environment created successfully!${NC}"
    echo -e "${YELLOW}Note: ROS2 packages are accessed from system site-packages${NC}"
else
    echo -e "${GREEN}Using existing virtual environment: $VENV_PATH${NC}"
fi

# Activate virtual environment
source "$VENV_PATH/bin/activate"

# Source ROS2 workspace (only ROS, not the conflicting packages)
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
else
    echo -e "${RED}ROS2 not found! Please install ROS2 first.${NC}"
    exit 1
fi

# Source the arm workspace for arm_perception_yolo package
WORKSPACE_ROOT="${HOME}/lododo-arm"
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/install/setup.bash"
fi

echo -e "${GREEN}Launching YOLO detection node...${NC}"
echo -e "${YELLOW}Note: This node runs in an isolated virtual environment${NC}"
echo ""

# Force Python to use the venv's packages first (especially NumPy 1.x)
export PYTHONPATH="$VENV_PATH/lib/python3.10/site-packages:$PYTHONPATH"

# Launch the complete YOLO perception system (yolo_detector + projection_node)
echo -e "${GREEN}Launching YOLO perception system (detector + projection)...${NC}"
exec ros2 launch arm_perception_yolo yolo_perception_launch.py "$@"
