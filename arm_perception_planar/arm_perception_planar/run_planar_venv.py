#!/usr/bin/env python3
"""
ROS2 entry point for run_planar_venv.sh script
This provides a Python wrapper that calls the bash script
"""

import os
import sys
import subprocess
from ament_index_python.packages import get_package_share_directory


def main():
    """Main entry point that calls the run_planar_venv.sh script"""
    # Get the package share directory
    pkg_share = get_package_share_directory('arm_perception_planar')
    script_path = os.path.join(pkg_share, 'scripts', 'run_planar_venv.sh')
    
    if not os.path.exists(script_path):
        print(f"Error: Script not found at {script_path}", file=sys.stderr)
        sys.exit(1)
    
    # Get command line arguments (skip the script name)
    args = sys.argv[1:] if len(sys.argv) > 1 else []
    
    # Call the bash script with the provided arguments
    try:
        result = subprocess.run([script_path] + args, check=True)
        sys.exit(result.returncode)
    except subprocess.CalledProcessError as e:
        print(f"Script failed with return code {e.returncode}", file=sys.stderr)
        sys.exit(e.returncode)
    except Exception as e:
        print(f"Error executing script: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()