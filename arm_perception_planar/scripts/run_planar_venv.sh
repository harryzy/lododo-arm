#!/usr/bin/env bash
# Wrapper to run planar perception nodes using the venv python
# This ensures OpenCV with GUI support is available for camera calibration

VENV="/home/hurry/planar_venv"

# Source venv if possible (so environment variables like PYTHONPATH are set)
if [ -f "$VENV/bin/activate" ]; then
  # shellcheck source=/dev/null
  . "$VENV/bin/activate"
fi

echo "[run_planar_venv] VENV=$VENV"
echo "[run_planar_venv] PYTHON=$VENV/bin/python"
echo "[run_planar_venv] ARGS=$@"

# If ros2 launch passed a --params-file, extract ros__parameters from the
# YAML and expose them to the node via PLANAR_PRELOAD_PARAMS (JSON). This lets
# the node pick up parameters even though we reset argv to avoid rclpy parsing
# launch command-line args.
PARAMS_FILE=""
for ((i=1;i<=$#;i++)); do
  if [ "${!i}" = "--params-file" ]; then
    next=$((i+1))
    PARAMS_FILE="${!next}"
    break
  fi
done

if [ -n "$PARAMS_FILE" ] && [ -f "$PARAMS_FILE" ]; then
  # Use venv python to parse YAML -> JSON and extract ros__parameters mapping
  PLANAR_PARAMS_JSON=$("$VENV/bin/python" -c 'import sys, yaml, json
f=sys.argv[1]
data=yaml.safe_load(open(f))
def find(d):
    if isinstance(d, dict):
        if "ros__parameters" in d and isinstance(d["ros__parameters"], dict):
            return d["ros__parameters"]
        for v in d.values():
            r=find(v)
            if r:
                return r
    return {}
res=find(data)
print(json.dumps(res))' "$PARAMS_FILE")
  
  if [ -n "$PLANAR_PARAMS_JSON" ]; then
    export PLANAR_PRELOAD_PARAMS="$PLANAR_PARAMS_JSON"
    echo "[run_planar_venv] Preloaded params from $PARAMS_FILE"
  fi
fi

# Determine which node to run based on the script name or first argument
NODE_MODULE=""
if [[ "$0" == *"cube_detector"* ]] || [[ "$1" == "cube_detector" ]]; then
  NODE_MODULE="arm_perception_planar.cube_detector_node"
  shift 2>/dev/null || true
elif [[ "$0" == *"localization"* ]] || [[ "$1" == "localization" ]]; then
  NODE_MODULE="arm_perception_planar.planar_localization_node"
  shift 2>/dev/null || true
else
  # Default to cube detector if not specified
  NODE_MODULE="arm_perception_planar.cube_detector_node"
fi

# Run the node by importing and calling its main() so rclpy.init() is performed
# Reset argv so rclpy doesn't try to parse ros2-launch args.
exec "$VENV/bin/python" -u -c "import sys
sys.argv=[sys.argv[0]]
from ${NODE_MODULE} import main
main()" -- "$@"
