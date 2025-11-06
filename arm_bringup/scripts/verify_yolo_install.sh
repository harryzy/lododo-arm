#!/bin/bash
# Quick script to verify YOLO CPU-only installation

echo "==========================================="
echo "  YOLO Installation Verification"
echo "==========================================="
echo ""

VENV_PATH="${HOME}/yolo_venv"

if [ ! -d "$VENV_PATH" ]; then
    echo "❌ Virtual environment not found at: $VENV_PATH"
    echo "   Run: ros2 run arm_bringup start_yolo_node.sh"
    exit 1
fi

echo "✅ Virtual environment found"
echo ""

source "$VENV_PATH/bin/activate"

echo "1. Checking PyTorch version..."
python3 -c "import torch; print(f'   PyTorch: {torch.__version__}')"

echo ""
echo "2. Checking CUDA availability..."
python3 -c "import torch; print(f'   CUDA Available: {torch.cuda.is_available()}')"
if python3 -c "import torch; exit(0 if not torch.cuda.is_available() else 1)"; then
    echo "   ✅ CPU-only version (correct!)"
else
    echo "   ⚠️  CUDA version detected (uses more space)"
fi

echo ""
echo "3. Checking installed packages..."
pip list | grep -E "torch|ultralytics|opencv"

echo ""
echo "4. Estimating installation size..."
du -sh "$VENV_PATH"

echo ""
echo "5. Testing YOLO import..."
python3 -c "from ultralytics import YOLO; print('   ✅ YOLO imported successfully')" 2>/dev/null || echo "   ❌ YOLO import failed"

echo ""
echo "==========================================="
echo "  Verification Complete"
echo "==========================================="
