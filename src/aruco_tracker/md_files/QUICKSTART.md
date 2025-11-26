# Quick Start Guide - ArUco Indoor Positioning

## Prerequisites

```bash
# Install required packages
sudo apt install ros-humble-usb-cam ros-humble-camera-calibration ros-humble-image-transport-plugins

# Build the package
cd ~/tracktor-beam
colcon build --packages-select aruco_tracker
source install/setup.bash
```

## Step-by-Step Setup

### 1. Print ArUco Marker

Visit: https://chev.me/arucogen/
- Dictionary: **4x4 (50)**
- Marker ID: **0**
- Marker size: **150mm**

Print and measure the actual size with a ruler!

### 2. Calibrate Camera

```bash
cd ~/tracktor-beam/src/aruco_tracker
./calibrate_camera.sh
```

Follow the on-screen instructions to calibrate your Logitech C270.

### 3. Update Marker Size

Edit `cfg/params_real_drone.yaml`:
```yaml
marker_size: 0.15  # YOUR MEASURED SIZE in meters
```

### 4. Configure PX4

In QGroundControl:
```
EKF2_AID_MASK = 24
EKF2_HGT_REF = 3
COM_RCL_EXCEPT = 4
```

### 5. Test Camera

```bash
# Check camera
ls /dev/video0

# Test feed
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0
```

### 6. Launch System

```bash
# Terminal 1: Start ArUco tracking + vision bridge
ros2 launch aruco_tracker aruco_tracker_real_drone.launch.py

# Terminal 2: Verify marker detection
ros2 topic echo /target_pose_down

# Terminal 3: Run offboard control
ros2 run px4_ros_com offboard_control_minimal
```

## Expected Result

When everything is working:

1. ✓ Camera streams images
2. ✓ ArUco tracker detects marker
3. ✓ Vision position bridge sends data to PX4
4. ✓ PX4 EKF2 fuses vision position
5. ✓ Offboard control can arm and fly!

## Troubleshooting

**Marker not detected?**
- Check lighting
- Verify marker is flat
- Ensure correct dictionary (DICT_4X4_50 = 0)

**Arming still fails?**
- Verify `/fmu/in/vehicle_visual_odometry` is publishing
- Check EKF2 parameters in QGC
- Ensure marker visible for >1 second before arming

**Position jumpy?**
- Recalibrate camera
- Use larger marker
- Improve lighting

See `SETUP_REAL_DRONE.md` for detailed troubleshooting.
