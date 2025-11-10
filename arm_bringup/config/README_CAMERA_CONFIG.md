# Camera Configuration Architecture

## 📋 Configuration Hierarchy (Single Source of Truth)

```
default_params.yaml (Master Configuration)
         ↓
  camera.launch.py (Auto-scaling)
         ↓
  ┌──────────────┴──────────────┐
  ↓                             ↓
robot_side.launch.py    real_bringup.launch.py
```

---

## 🎯 Single Source of Truth: `default_params.yaml`

All camera parameters are now centralized in **`default_params.yaml`**:

### Resolution & Framerate
```yaml
camera_width: 640            # Image width (pixels)
camera_height: 480           # Image height (pixels)
camera_fps: 15.0             # Framerate (Hz)
```

### Image Quality Parameters
```yaml
camera_brightness: 20        # 0-255, reduce reflections
camera_contrast: 50          # Enhance edges
camera_saturation: 60        # Color accuracy
camera_sharpness: 6          # Object contours
camera_auto_exposure: 1      # 1=Manual, 3=Auto
camera_exposure: 30          # 1-5000, reduce overexposure
camera_auto_white_balance: true
camera_white_balance: 4000
```

### Camera Intrinsic Parameters (Base: 640×480)
```yaml
camera_fx: 1128.1            # Focal length X
camera_fy: 1128.1            # Focal length Y
camera_cx: 320.0             # Principal point X
camera_cy: 240.0             # Principal point Y
```

---

## 🔄 Auto-Scaling Mechanism

### How It Works

`camera.launch.py` automatically scales intrinsic parameters based on resolution:

```python
# Example: 640×480 → 320×240
scale_x = 320 / 640 = 0.5
scale_y = 240 / 480 = 0.5

fx_new = 1128.1 × 0.5 = 564.05
fy_new = 1128.1 × 0.5 = 564.05
cx_new = 320.0 × 0.5 = 160.0
cy_new = 240.0 × 0.5 = 120.0
```

### Supported Resolutions

| Resolution | fx/fy | cx | cy | Use Case |
|------------|-------|----|----|----------|
| **640×480** | 1128.1 | 320.0 | 240.0 | Standard (default) |
| **320×240** | 564.05 | 160.0 | 120.0 | Lite mode (Raspberry Pi) |
| **1280×720** | 1692.15 | 640.0 | 360.0 | High precision |

---

## 📝 How to Change Camera Settings

### Method 1: Modify `default_params.yaml` (Recommended)

Change resolution to 320×240:
```yaml
camera_width: 320
camera_height: 240
camera_fps: 15.0
```

Everything else (intrinsics, quality parameters) is automatically handled!

### Method 2: Override via Launch Arguments

```bash
ros2 launch arm_bringup real_bringup.launch.py \
  camera_width:=320 \
  camera_height:=240 \
  camera_fps:=15.0
```

Or:
```bash
ros2 launch arm_bringup robot_side.launch.py \
  use_camera:=true
```

---

## 🗂️ Configuration Files

### Active Files

| File | Purpose | Auto-Generated |
|------|---------|----------------|
| **`default_params.yaml`** | Master configuration | ❌ Manual edit |
| **`camera.launch.py`** | Launch file with auto-scaling | ❌ Code |
| **`camera_info.yaml`** | Reference documentation | ✅ Runtime |

### Removed Files

| File | Status | Reason |
|------|--------|--------|
| ~~`camera_params.yaml`~~ | ❌ Deleted | Unused, caused confusion |

---

## 🔍 Troubleshooting

### Issue: Depth measurement incorrect

**Cause**: Camera intrinsics don't match actual resolution

**Solution**: Check `default_params.yaml` resolution matches actual camera settings

### Issue: Image quality poor

**Solution**: Adjust quality parameters in `default_params.yaml`:
- Reduce `camera_brightness` (e.g., 10-30) to avoid reflections
- Increase `camera_contrast` (e.g., 50-70) for better edges
- Reduce `camera_exposure` (e.g., 20-40) to eliminate overexposure

### Issue: Camera fails to start

**Check**:
1. Device exists: `ls -l /dev/video*`
2. Permissions: `sudo usermod -a -G video $USER`
3. usb_cam installed: `ros2 pkg list | grep usb_cam`

---

## 📊 Verification Commands

### Check loaded parameters:
```bash
ros2 param list /camera/usb_cam
ros2 param get /camera/usb_cam image_width
ros2 param get /camera/usb_cam camera_info_url
```

### View camera info:
```bash
ros2 topic echo /camera/camera_info --once
```

### Test camera stream:
```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

---

## 🎯 Benefits of New Architecture

✅ **Single Source of Truth**: All parameters in one place  
✅ **No Redundancy**: Eliminated duplicate configurations  
✅ **Auto-Scaling**: Intrinsics automatically match resolution  
✅ **Easy Maintenance**: Change resolution in one place  
✅ **No Manual Calibration**: Works for any resolution  
✅ **Consistent**: robot_side and real_bringup use same source  

---

## 📐 Calibration Notes

Base intrinsics (`fx=1128.1`) calibrated for 640×480:
- Method: Real-world measurement (object size vs pixel size)
- Validation: Mouse 10cm @ 147px → 33cm distance
- Camera angle: 43.6° tilt considered

**No recalibration needed** when changing resolution - auto-scaling handles it!
