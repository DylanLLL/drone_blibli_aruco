# ArUco Tracker Setup for Real Drone with Logitech C270

This guide will help you set up indoor position estimation using ArUco markers and a Logitech C270 webcam.

## Hardware Requirements

1. **Logitech C270 Webcam** - USB camera
2. **Companion Computer** - Raspberry Pi 4 or similar (running ROS2)
3. **ArUco Markers** - Printed on paper/cardboard
4. **PX4 Flight Controller** - Connected to companion computer via serial/ethernet

## Software Requirements

```bash
# Install USB camera driver
sudo apt install ros-humble-usb-cam

# Install camera calibration tools
sudo apt install ros-humble-camera-calibration

# Install image transport
sudo apt install ros-humble-image-transport-plugins
```

## Setup Steps

### Step 1: Print ArUco Markers

1. Generate markers online: https://chev.me/arucogen/
   - Dictionary: 4x4 (50, 100, 250, or 1000)
   - Marker ID: 0 (or whatever you want to track)
   - Marker Size: 150mm (15cm) recommended

2. Print the marker on paper
3. **IMPORTANT**: Measure the actual printed size with a ruler!
4. Mount markers on the floor where you'll fly

### Step 2: Connect Camera

```bash
# Check if camera is detected
ls /dev/video*

# Should show /dev/video0 (or video1, video2, etc.)

# Test camera
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0
```

### Step 3: Calibrate Camera (CRITICAL!)

**This is required for accurate position estimation!**

```bash
# Download calibration pattern
wget https://raw.githubusercontent.com/opencv/opencv/master/doc/pattern.png

# Print the checkerboard pattern (make sure it prints at actual size)

# Run calibration
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.025 \
  --ros-args -r image:=/camera_down/image_raw -r camera:=/usb_cam

# Follow on-screen instructions:
# 1. Move checkerboard around (left, right, up, down, tilted)
# 2. Fill all the bars (X, Y, Size, Skew)
# 3. Click "Calibrate" button
# 4. Click "Save" button
# 5. Calibration file will be saved to /tmp/calibrationdata.tar.gz
```

Extract calibration and update config:
```bash
cd /tmp
tar -xzf calibrationdata.tar.gz
cat ost.yaml

# Copy the calibration values to:
# ~/tracktor-beam/src/aruco_tracker/cfg/logitech_c270_calibration.yaml
```

### Step 4: Update Parameters

Edit `cfg/params_real_drone.yaml`:
```yaml
aruco_tracker:
  ros__parameters:
    aruco_id: 0              # Match your printed marker ID
    dictionary: 0            # 0 = DICT_4X4_50
    marker_size: 0.15        # YOUR MEASURED SIZE in meters!
```

### Step 5: Configure PX4 Parameters

In QGroundControl, set these parameters:

```
# Enable vision position fusion
EKF2_AID_MASK = 24           # Enable vision position + vision yaw
EKF2_HGT_REF = 3             # Vision as height reference

# Vision sensor configuration
EKF2_EV_DELAY = 0            # Vision data delay (tune if needed)
EKF2_EV_POS_X = 0.0          # Camera offset from center of mass (X)
EKF2_EV_POS_Y = 0.0          # Camera offset from center of mass (Y)
EKF2_EV_POS_Z = 0.0          # Camera offset from center of mass (Z)

# Enable offboard mode without RC
COM_RCL_EXCEPT = 4           # Allow offboard without RC

# Position control parameters
MPC_XY_VEL_MAX = 1.0         # Max horizontal velocity (m/s)
MPC_Z_VEL_MAX_UP = 1.0       # Max ascent velocity (m/s)
MPC_Z_VEL_MAX_DN = 1.0       # Max descent velocity (m/s)
```

### Step 6: Build the Package

```bash
cd ~/tracktor-beam
colcon build --packages-select aruco_tracker
source install/setup.bash
```

### Step 7: Test Setup

**Test 1: Camera Feed**
```bash
# Terminal 1: Launch camera
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0

# Terminal 2: View image
ros2 run rqt_image_view rqt_image_view
# Select topic: /image_raw
```

**Test 2: ArUco Detection**
```bash
# Terminal 1: Launch everything
ros2 launch aruco_tracker aruco_tracker_real_drone.launch.py

# Terminal 2: Check if marker is detected
ros2 topic echo /target_pose_down

# You should see pose data when marker is visible!
```

**Test 3: Vision Position to PX4**
```bash
# Check if PX4 is receiving vision data
ros2 topic echo /fmu/in/vehicle_visual_odometry

# In QGC, check if EKF is using vision:
# Analyze Tools > MAVLink Inspector > ODOMETRY
```

### Step 8: Run Offboard Control

```bash
# Make sure PX4 is armed and ready

# Terminal 1: Launch ArUco tracking + vision bridge
ros2 launch aruco_tracker aruco_tracker_real_drone.launch.py

# Terminal 2: Run offboard control
ros2 run px4_ros_com offboard_control_minimal

# The drone should now:
# 1. See the marker
# 2. Get position estimate
# 3. Switch to offboard mode
# 4. ARM successfully!
# 5. Hold position
```

## Troubleshooting

### Camera not detected
```bash
# Check permissions
sudo usermod -a -G video $USER
# Log out and log back in

# Check device
v4l2-ctl --list-devices
```

### Marker not detected
- Check lighting (avoid glare, shadows)
- Ensure marker is flat and not wrinkled
- Verify marker dictionary matches config
- Check camera focus (C270 has manual focus ring)

### Position estimate jumpy/unstable
- Recalibrate camera (most common issue)
- Increase marker size for better accuracy
- Improve lighting conditions
- Reduce camera motion blur (lower framerate if needed)

### PX4 not using vision estimate
```bash
# Check EKF2 status in PX4 console
listener vehicle_local_position
ekf2 status

# Verify vision data timestamp is recent
listener vehicle_visual_odometry
```

### Offboard arming still fails
- Verify vision position is being received by PX4
- Check EKF2_AID_MASK is set correctly
- Ensure marker is visible for at least 1 second before arming
- Check battery is connected and voltage is good

## Camera Mounting Tips

1. **Downward-facing**: Mount camera facing straight down
2. **Field of view**: C270 has ~60° FOV - marker should be clearly visible
3. **Height**: At 2m height, 15cm marker is easily detected
4. **Vibration**: Use dampening foam to reduce vibration blur
5. **Cable management**: Secure USB cable to prevent disconnections

## Next Steps

Once you have stable position estimation:

1. Test hovering at different heights
2. Try slow waypoint navigation
3. Add multiple markers for larger coverage area
4. Tune EKF2 parameters for your specific setup
5. Implement failsafe behaviors

## Performance Tips

- **Marker size vs height**: Larger markers = higher flight altitude
- **Multiple markers**: Place markers in a grid for continuous coverage
- **Lighting**: Consistent, diffuse lighting works best
- **Update rate**: 30 Hz camera is sufficient, 60 Hz is better

Good luck with your indoor positioning setup! 🚁
