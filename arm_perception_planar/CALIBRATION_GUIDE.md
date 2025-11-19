# Camera Calibration Guide

This guide walks you through camera calibration — a critical step to achieve accurate planar perception and 3D localization.

---

## Contents

1. [Overview](#overview)
2. [Preparation](#preparation)
3. [Intrinsic Calibration](#intrinsic-calibration)
4. [Hand–Eye Calibration (optional)](#hand-eye-calibration)
5. [Validation](#validation)
6. [Troubleshooting](#troubleshooting)

---

## Overview

### Why calibrate?

Calibration determines:
- Intrinsics: focal length (fx, fy), principal point (cx, cy), distortion coefficients
- Extrinsics: camera pose relative to the robot `base_link` (translation and rotation)

Accurate calibration parameters directly affect 3D localization accuracy.

### High-level flow

```
1. Print a calibration checkerboard (8x6 internal corners, 25 mm squares)
2. Run the `camera_calibration` tool
3. Capture images from multiple viewpoints
4. Compute calibration parameters
5. Save parameters to configuration files
6. Validate results
```

---

## Preparation

### 1. Install dependencies

```bash
# Camera calibration tool
sudo apt install ros-humble-camera-calibration

# USB camera driver (if using USB camera)
sudo apt install ros-humble-usb-cam

# Image pipeline utilities
sudo apt install ros-humble-image-pipeline

# System OpenCV with GTK support
sudo apt install python3-opencv libgtk2.0-dev pkg-config

# Create an isolated Python virtual environment (recommended)
# This avoids dependency conflicts with packages such as ultralytics
cd ~/lododo/src/lododo-arm/arm_perception_planar
bash scripts/setup_planar_venv.sh

# If the setup script reports missing GTK, run:
# source /home/hurry/planar_venv/bin/activate
# /home/hurry/planar_venv/use_system_opencv.sh
```

Why a virtual environment?
- System OpenCV (e.g. 4.5.4) expects NumPy 1.x
- Ultralyics (YOLO) and `opencv-python` may require NumPy 2.x
- A virtual environment isolates these conflicting requirements

Important: if you previously installed NumPy 2.x via `pip`, temporarily hide it:

```bash
# Check for locally installed numpy
ls ~/.local/lib/python3.10/site-packages/ | grep numpy

# If you see numpy-2.x, temporarily rename the package directory
mv ~/.local/lib/python3.10/site-packages/numpy{,.bak.old}
mv ~/.local/lib/python3.10/site-packages/numpy-*.dist-info{,.bak} 2>/dev/null || true
```

Restore after calibration:

```bash
mv ~/.local/lib/python3.10/site-packages/numpy{.bak.old,}
mv ~/.local/lib/python3.10/site-packages/numpy-*.dist-info{.bak,} 2>/dev/null || true
```

### 2. Prepare the checkerboard

Option A: Print a checkerboard

Use: https://calib.io/pages/camera-calibration-pattern-generator

Parameters:
- Pattern: Checkerboard
- Rows: 9 (internal corners: 6)
- Columns: 11 (internal corners: 8)
- Square size: 25 mm
- Page size: A4
- Units: millimeters

Download the PDF, print it on a flat backing board.

Option B: Use a commercial calibration board

Record:
- Number of internal corners (rows x columns)
- Square size (in meters)

### 3. Measure printed board

Use a ruler to measure 3–4 squares to ensure the board was printed without scaling.

---

## Intrinsic Camera Calibration

### Step 1: Start the camera node

Note: If the camera runs on a Raspberry Pi, use a lightweight configuration to improve frame rate.

```bash
# Option A: Run on a PC (recommended for higher frame rate)
source ~/lododo/install/setup.bash
ros2 launch arm_bringup real_bringup.launch.py

# Option B: Distributed deployment — Raspberry Pi (SSH)
# Use a lightweight configuration (320x240@10fps)
ssh lododo@<raspberry-pi-ip>
cd ~/lododo-arm
source install/setup.bash
ros2 launch arm_bringup robot_side_lite.launch.py

# Check camera frame rate
ros2 topic hz /camera/image_raw
# Target: at least 8-10 Hz for responsive calibration
# If < 5 Hz, Raspberry Pi is likely overloaded
```

Frame rate troubleshooting

If `ros2 topic hz /camera/image_raw` reports a low frame rate (<5 Hz):

1. Check CPU/memory on the Raspberry Pi:
```bash
htop  # inspect resource usage
# If memory > 80%, stop other processes
```

2. Reduce camera resolution:
```bash
# Restart the launch with lower resolution
ros2 launch arm_bringup robot_side.launch.py

# Or dynamically adjust parameters:
ros2 param set /camera/usb_cam image_width 320
ros2 param set /camera/usb_cam image_height 240
ros2 param set /camera/usb_cam framerate 10
```

3. Check USB connection:
```bash
# Ensure the camera is connected to a USB 2.0/3.0 port
lsusb  # list USB devices
dmesg | tail -20  # inspect USB-related kernel messages
```

Validation: view the image stream
```bash
ros2 topic list | grep image
# Expect to see: /camera/image_raw

ros2 run rqt_image_view rqt_image_view
# Select /camera/image_raw and verify a smooth video stream
```

### Step 2: Start the calibration tool

**Recommended method (using wrapper script)**:

```bash
# Terminal 2: Use the convenient startup script
cd ~/lododo/src/lododo-arm/arm_perception_planar
bash scripts/run_calibration.sh
```

This script will automatically:
 - ✓ Activate the virtual environment
 - ✓ Configure correct Python and NumPy paths
 - ✓ Verify OpenCV GTK support
 - ✓ Check camera topics
 - ✓ Launch the calibration tool

**Manual method (for custom parameters)**:

```bash
# First activate the virtual environment
source /home/hurry/planar_venv/bin/activate
source ~/lododo/install/setup.bash

# Running the calibration tool
ros2 run camera_calibration cameracalibrator \
  --size 6x8 \
  --square 0.025 \
  --ros-args -r image:=/camera/image_raw -r camera:=/camera
```

**Parameter descriptions**:
- `--size 6x8`: Number of internal corners (columns x rows)
-- `--square 0.025`: Square size 25 mm = 0.025 m
-- `-r image:=/camera/image_raw`: Image topic remapping

**Startup success indicators**:
```
✓ NumPy version: 1.26.4
✓ OpenCV version: 4.5.4
Waiting for service camera/set_camera_info ... OK
*** Added sample 1, p_x = 0.310, p_y = 0.730, ...
```

Ignore warnings about `/left` and `/right` topics (these are stereo camera topics; not required for a monocular camera).

### Step 3: Capture calibration images

The calibration window will display 4 progress bars:
 - **X**: Move the calibration board left-right (horizontal coverage)
 - **Y**: Move the calibration board up-down (vertical coverage)
 - **Size**: Change the distance between the board and the camera (scale variation)
 - **Skew**: Tilt the calibration board (angle variation)

**Important notes**:
 - ⚠️ **Not every frame is sampled**: The tool will add a new sample only when the board position/angle is sufficiently different from previous samples
 - 💡 **Do not wait in place**: If the board is stationary or moved too slowly, the tool may appear to 'stall'
 - 📊 **Goal**: Collect 30-40 samples so that all 4 progress bars turn green

**Efficient capture tips**:

1. **Move with large displacements, avoid slow sliding**:
   ```
  ❌ Wrong: Slowly slide the board from left to right (very slow sampling)
  ✅ Correct: Move: left → lift → right → lift → center (large jumps)
   ```

2. **Monitor terminal output**:
   ```bash
   *** Added sample 3, p_x = 0.669, p_y = 0.923, p_size = 0.413, skew = 0.439
   ```
  - `p_x`: horizontal position (0.0=left, 1.0=right)
  - `p_y`: vertical position (0.0=top, 1.0=bottom)
  - `p_size`: board size (0.0=far, 1.0=near)
  - `skew`: tilt angle

3. **Fill as needed**: Move more in the direction whose progress bar is not full

**Recommended capture sequence**:
1. **X direction** (left-right horizontal):
  - Far left
  - Left-center
  - Center
  - Right-center
  - Far right

2. **Y direction** (up-down vertical):
  - Top
  - Upper-center
  - Center
  - Lower-center
  - Bottom

3. **Size direction** (distance variation):
  - Short distance (calibration board fills ~80% of the image)
  - Medium distance (calibration board occupies ~50% of the image)
  - Long distance (calibration board occupies ~30% of the image)

4. **Skew direction** (tilt angles):
  - Facing the camera (0°)
  - Tilt left 15° and 30°
  - Tilt right 15° and 30°
  - Tilt up 15° and 30°
  - Tilt down 15° and 30°

**Collection time**: The whole process typically takes 5-10 minutes (depends on movement speed)

### Step 4: Compute calibration parameters

When the progress bar turns green:
1. Click the **CALIBRATE** button
2. Wait for the computation to finish (may take 1-2 minutes)
3. After computation completes, the terminal will display the calibration results

### Step 5: Save calibration parameters

1. Click the **SAVE** button
2. Parameters will be saved to `/tmp/calibrationdata.tar.gz`
3. Extract and inspect:

```bash
cd /tmp
tar -xzf calibrationdata.tar.gz
cat ost.yaml  # Inspect the calibration result
```

### Step 6: Configure camera node

Integrate the calibration parameters into the camera:

**Method A: Use camera_info_manager**

Create `camera_calibration.yaml`:

```yaml
image_width: 640
image_height: 480
camera_name: usb_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [k1, k2, p1, p2, k3]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
```

**Method B: Update `planar_params.yaml`**

Copy intrinsics into the configuration file:
```yaml
camera_matrix:
  fx: 600.0  # Copy from calibration results
  fy: 600.0
  cx: 320.0
  cy: 240.0

distortion_coeffs: [k1, k2, p1, p2, k3]  # Copy from calibration results
```

---

## Hand–eye calibration

Hand–eye calibration determines the transform from the camera frame to the robot `base_link`.

### Method 1: Use existing parameters

If you already have a working system, you can reuse existing extrinsics:

```yaml
# Copy from arm_perception_yolo
camera_pose:
  translation:
    x: -0.2745
    y: -0.0080
    z: 0.1055
  rotation:
    pitch: 97.75
    roll: 0.0
    yaw: 0.0
```

### Method 2: Manual measurement

1. Use a tape measure to record the distance from the camera optical center to `base_link`.
2. Measure the camera pitch angle with a protractor.
3. Update `planar_params.yaml` with the measured values.

### Method 3: Use easy_handeye2 (recommended, high accuracy)

```bash
# Install the tool
sudo apt install ros-humble-easy-handeye

# Run the calibration
ros2 launch easy_handeye calibrate.launch.py
```

Reference: http://docs.ros.org/en/ros2_packages/humble/api/easy_handeye/index.html

---

## Validate calibration results

### 1. Reprojection error

Check the calibration tool output:
```
Reprojection error: 0.35 pixels  # ✅ Should be < 0.5 pixels
```

### 2. Distortion correction

```bash
# Publish distortion-corrected images
ros2 run image_proc image_proc --ros-args \
  -r image_raw:=/camera/image_raw \
  -r camera_info:=/camera/camera_info

# View the rectified/undistorted image
ros2 run rqt_image_view rqt_image_view /image_rect
```

Check:
- Straight lines remain straight (especially near image edges)
- Distortion is corrected

### 3. 3D localization accuracy test

```bash
# Run the planar perception system
ros2 launch arm_perception_planar planar_perception.launch.py

# Place a cube on the table at a known position (measure with a ruler)
# Compare the reported position with the measured ground truth

# View detection results
ros2 topic echo /planar/measured_objects
```

Accuracy targets:
- XY plane error: < 5 mm ✅
- Z axis error: < 3 mm ✅

---

## Troubleshooting

### Issue 1: Checkerboard not detected

**Symptom**: The calibration tool window shows no green corner markers

**Solution**:
- Check lighting: avoid strong reflections or deep shadows
- Adjust contrast: ensure the black/white squares are clear
- Verify print quality: edges must be sharp
- Try different distances: 0.3–0.8 m

### Issue 2: Calibration tool appears stalled

**Symptom**: Clicking the CALIBRATE button has no effect

**Solution**:
```bash
# Check the image stream
ros2 topic hz /camera/image_raw

# Expected output: average rate: 30.0
# If there is no output, restart the camera node
```

### Issue 3: Large reprojection error

**Symptom**: Reprojection error > 1.0 pixel

**Solution**:
- Re-capture with greater sample diversity
- Ensure the calibration board is flat and not warped
- Verify the printed board dimensions are accurate
- Capture more images (50+)

### Issue 4: 3D localization offset

**Symptom**: Reported position differs from actual by > 1 cm

**Possible causes and fixes**:

1. **Table height incorrect**:
  ```yaml
  # Adjust table_height_offset
  table_height_offset: 0.005  # increase by 5 mm
  ```

2. **Camera extrinsics inaccurate**:
  - Re-measure the camera pose
  - Re-run hand–eye calibration (e.g., easy_handeye)

3. **Camera intrinsics inaccurate**:
  - Re-run camera_calibration
  - Ensure reprojection error < 0.5 pixels

### Issue 5: NumPy version conflict

**Symptom**: 
```
A module that was compiled using NumPy 1.x cannot be run in NumPy 2.2.6
AttributeError: _ARRAY_API not found
```

**Cause**: Numpy 2.x installed in the user directory `~/.local/lib` conflicts with ROS2's Numpy 1.x

**Solution**:
```bash
# Method 1: Use the provided startup script (handles conflicts automatically)
cd ~/lododo/src/lododo-arm/arm_perception_planar
bash scripts/run_calibration.sh

# Method 2: Temporarily disable user-installed numpy
mv ~/.local/lib/python3.10/site-packages/numpy ~/.local/lib/python3.10/site-packages/numpy.bak.old
mv ~/.local/lib/python3.10/site-packages/numpy-*.dist-info ~/.local/lib/python3.10/site-packages/numpy.bak.dist-info 2>/dev/null || true

# After calibration, restore numpy for other applications
mv ~/.local/lib/python3.10/site-packages/numpy{.bak.old,}
mv ~/.local/lib/python3.10/site-packages/numpy.bak.dist-info ~/.local/lib/python3.10/site-packages/numpy-2.2.6.dist-info 2>/dev/null || true
```

---

## Calibration checklist

After calibration, verify the following items:

- [ ] Reprojection error < 0.5 pixels
- [ ] Straight lines preserved after distortion correction
- [ ] `camera_info` topic is published correctly
- [ ] Intrinsics configured in the camera node or `planar_params.yaml`
- [ ] Extrinsics configured in `planar_params.yaml`
- [ ] 3D localization accuracy test passed (error < 5 mm)

---

## Quick reference

### Common commands

```bash
# Run camera calibration (recommended)
cd ~/lododo/src/lododo-arm/arm_perception_planar
bash scripts/run_calibration.sh

# Or manually activate the virtual environment
source /home/hurry/planar_venv/bin/activate
source ~/lododo/install/setup.bash

# List camera topics
ros2 topic list | grep camera

# Inspect camera parameters
ros2 topic echo /camera/camera_info --once

# Test camera image stream
ros2 run rqt_image_view rqt_image_view

# Launch the planar perception system
ros2 launch arm_perception_planar planar_perception.launch.py
```

### Virtual environment management

```bash
# Create / rebuild virtual environment
cd ~/lododo/src/lododo-arm/arm_perception_planar
bash scripts/setup_planar_venv.sh

# Link system OpenCV (if GTK is unavailable)
source /home/hurry/planar_venv/bin/activate
bash /home/hurry/planar_venv/use_system_opencv.sh

# Temporarily disable user numpy 2.x (before calibration)
mv ~/.local/lib/python3.10/site-packages/numpy{,.bak.old} 2>/dev/null || true

# Restore user numpy (after calibration)
mv ~/.local/lib/python3.10/site-packages/numpy{.bak.old,} 2>/dev/null || true
```

### Configuration file locations

```
Calibration parameters: ~/lododo/src/lododo-arm/arm_perception_planar/config/planar_params.yaml
Checkerboard PDF: ~/lododo/src/lododo-arm/arm_perception_planar/resource/checkerboard/
Temporary calibration results: /tmp/calibrationdata.tar.gz
```

---

## References

- [ROS Camera Calibration](http://wiki.ros.org/camera_calibration)
- [Camera Calibration Pattern Generator](https://calib.io/)
- [Easy Handeye](http://docs.ros.org/en/ros2_packages/humble/api/easy_handeye/)
- [OpenCV Camera Calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)

---

**Need help?** Refer to the main design document or open an Issue on the GitHub repository.
