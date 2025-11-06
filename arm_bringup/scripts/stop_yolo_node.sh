#!/bin/bash
# Script to stop YOLO perception node
# Kills the yolo_detector process and its parent processes

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}  Stopping YOLO Perception Node${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

# Find all yolo_detector processes
YOLO_PIDS=$(pgrep -f "yolo_detector" 2>/dev/null || true)

if [ -z "$YOLO_PIDS" ]; then
    echo -e "${YELLOW}No YOLO detector process found.${NC}"
    echo -e "${GREEN}YOLO node is not running.${NC}"
    exit 0
fi

echo -e "${YELLOW}Found YOLO detector process(es):${NC}"
ps -fp $YOLO_PIDS

echo ""
echo -e "${YELLOW}Stopping YOLO detector...${NC}"

# Kill the processes gracefully first (SIGTERM)
for PID in $YOLO_PIDS; do
    if kill -0 $PID 2>/dev/null; then
        echo -e "  Sending SIGTERM to PID $PID..."
        kill -TERM $PID 2>/dev/null || true
    fi
done

# Wait a bit for graceful shutdown
sleep 2

# Check if any processes are still running
REMAINING=$(pgrep -f "yolo_detector" 2>/dev/null || true)

if [ ! -z "$REMAINING" ]; then
    echo -e "${YELLOW}Some processes didn't stop gracefully. Forcing shutdown...${NC}"
    
    # Force kill (SIGKILL)
    for PID in $REMAINING; do
        if kill -0 $PID 2>/dev/null; then
            echo -e "  Sending SIGKILL to PID $PID..."
            kill -9 $PID 2>/dev/null || true
        fi
    done
    
    sleep 1
fi

# Final check
FINAL_CHECK=$(pgrep -f "yolo_detector" 2>/dev/null || true)

if [ -z "$FINAL_CHECK" ]; then
    echo ""
    echo -e "${GREEN}✓ YOLO detector stopped successfully!${NC}"
    echo -e "${GREEN}========================================${NC}"
    exit 0
else
    echo ""
    echo -e "${RED}✗ Failed to stop some YOLO processes:${NC}"
    ps -fp $FINAL_CHECK
    echo -e "${RED}You may need to manually kill them with:${NC}"
    echo -e "${RED}  kill -9 $FINAL_CHECK${NC}"
    exit 1
fi
