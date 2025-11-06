# Workspace Setup Guide

This guide explains the standard ROS2 workspace structure for the Lododo Arm project.

## 📁 Workspace Structure

The Lododo Arm project follows the standard ROS2 workspace layout:

```
~/lododo-arm/                    # Workspace root directory
├── src/                         # Source code directory (version controlled)
│   ├── arm_bringup/             # Launch files and scripts
│   ├── arm_description/         # Robot URDF and meshes
│   ├── arm_driver_node/         # Hardware driver (C++)
│   ├── arm_interfaces/          # Custom messages and services
│   ├── arm_moveit_config/       # MoveIt2 configuration
│   ├── arm_perception_yolo/     # YOLO vision system
│   ├── arm_planning_py/         # Motion planning (Python)
│   ├── arm_rviz_plugin/         # Custom RViz plugins
│   ├── arm_voice_interface/     # Voice control interface
│   ├── README.md                # Project documentation
│   ├── CONTRIBUTING.md          # Contribution guidelines
│   └── LICENSE                  # License file
├── build/                       # Build artifacts (auto-generated, not committed)
├── install/                     # Install space (auto-generated, not committed)
└── log/                         # Build logs (auto-generated, not committed)
```

## 🚀 Initial Setup

### Step 1: Create Workspace

```bash
# Create workspace directory structure
mkdir -p ~/lododo-arm/src
cd ~/lododo-arm/src
```

### Step 2: Clone Repository

```bash
# Clone source code into src/ directory
git clone https://github.com/harryzy/lododo-arm.git .

# The '.' at the end clones directly into current directory (src/)
# This avoids nested directories like src/lododo-arm/src/
```

### Step 3: Install Dependencies

```bash
# Return to workspace root
cd ~/lododo-arm

# Install ROS2 dependencies using rosdep
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Install additional Python dependencies
pip3 install pyserial numpy scipy torch torchvision ultralytics
```

### Step 4: Build Workspace

```bash
# Build all packages
cd ~/lododo-arm
colcon build

# Or build specific packages only
colcon build --packages-select arm_bringup arm_driver_node

# For parallel builds (faster on multi-core systems)
colcon build --parallel-workers 4
```

### Step 5: Source Workspace

```bash
# Source the workspace (must do after every build)
source ~/lododo-arm/install/setup.bash

# Add to ~/.bashrc for automatic sourcing
echo "source ~/lododo-arm/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 🔄 Daily Workflow

### Building After Changes

```bash
# After modifying C++ code
cd ~/lododo-arm
colcon build --packages-select <package_name>
source install/setup.bash

# After modifying Python code (no rebuild needed, but recommended)
cd ~/lododo-arm
colcon build --packages-select <package_name> --symlink-install
source install/setup.bash
```

### Cleaning Build Artifacts

```bash
# Clean specific package
cd ~/lododo-arm
rm -rf build/<package_name> install/<package_name>
colcon build --packages-select <package_name>

# Clean entire workspace
cd ~/lododo-arm
rm -rf build install log
colcon build
```

## 📝 Important Notes

### ✅ Do's

1. **Always work from workspace root**: Most ROS2 commands expect to be run from `~/lododo-arm/`
2. **Source after building**: Run `source install/setup.bash` after every `colcon build`
3. **Use version control**: Only `src/` directory should be committed to git
4. **Check dependencies**: Use `rosdep check --from-paths src --ignore-src` to verify

### ❌ Don'ts

1. **Don't commit build artifacts**: `build/`, `install/`, and `log/` should be in `.gitignore`
2. **Don't modify install space**: Always edit source files in `src/`, never in `install/`
3. **Don't nest workspaces**: Avoid creating workspace inside another workspace
4. **Don't use absolute paths**: Use relative paths or `find_package()` in CMake

## 🔍 Troubleshooting

### Issue: "Package not found"

```bash
# Check if package exists
ls ~/lododo-arm/src/

# Rebuild workspace
cd ~/lododo-arm
colcon build
source install/setup.bash
```

### Issue: "Setup.bash not found"

```bash
# Build the workspace first
cd ~/lododo-arm
colcon build

# Then source will work
source install/setup.bash
```

### Issue: "CMake errors during build"

```bash
# Clean and rebuild
cd ~/lododo-arm
rm -rf build install log
colcon build --symlink-install
```

### Issue: "Python import errors"

```bash
# Ensure workspace is sourced
source ~/lododo-arm/install/setup.bash

# Check PYTHONPATH includes install space
echo $PYTHONPATH
# Should include: /home/<user>/lododo-arm/install/...
```

## 🌐 Distributed Deployment

When deploying across multiple machines (e.g., Raspberry Pi + PC):

### On Each Machine:

1. **Same workspace structure**:
   ```bash
   # Both machines use same path
   ~/lododo-arm/
   ```

2. **Same ROS_DOMAIN_ID**:
   ```bash
   export ROS_DOMAIN_ID=0
   ```

3. **Network accessibility**:
   ```bash
   export ROS_LOCALHOST_ONLY=0
   ```

See [distributed_deployment_guide.md](distributed_deployment_guide.md) for complete setup.

## 📚 Additional Resources

- [ROS2 Workspace Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
- [Colcon Documentation](https://colcon.readthedocs.io/)
- [ROS2 Package Creation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

## 🆘 Getting Help

If you encounter workspace-related issues:

1. Check [GitHub Issues](https://github.com/harryzy/lododo-arm/issues)
2. Verify ROS2 installation: `ros2 --version`
3. Check workspace structure: `tree -L 2 ~/lododo-arm/`
4. Review build logs: `cat ~/lododo-arm/log/latest_build/events.log`
