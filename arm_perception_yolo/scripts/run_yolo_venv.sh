#!/usr/bin/env bash
# Wrapper to run the detector module using the venv python so launch's Node mode
# can be used while ensuring the venv interpreter/site-packages are active.
VENV="/home/hurry/yolo_venv"

# v4l2-ctl --set-ctrl=brightness=40
# v4l2-ctl --set-ctrl=contrast=150

# source venv if possible (so environment variables like PYTHONPATH are set)
if [ -f "$VENV/bin/activate" ]; then
  # shellcheck source=/dev/null
  . "$VENV/bin/activate"
fi

echo "[run_yolo_venv] VENV=$VENV"
echo "[run_yolo_venv] PYTHON=$VENV/bin/python"
echo "[run_yolo_venv] ARGS=$@"

# If ros2 launch passed a --params-file, extract ros__parameters from the
# YAML and expose them to the node via YOLO_PRELOAD_PARAMS (JSON). This lets
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
  YOLO_PARAMS_JSON=$("$VENV/bin/python" -c 'import sys, yaml, json
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
  if [ -n "$YOLO_PARAMS_JSON" ]; then
    export YOLO_PRELOAD_PARAMS="$YOLO_PARAMS_JSON"
    echo "[run_yolo_venv] Preloaded params from $PARAMS_FILE"
  fi
fi

# Run the detector by importing and calling its main() so rclpy.init() is performed
# and the node will spin. Reset argv so rclpy doesn't try to parse ros2-launch args.
exec "$VENV/bin/python" -u -c 'import sys
sys.argv=[sys.argv[0]]
from arm_perception_yolo.yolo_detector import main
main()' -- "$@"
