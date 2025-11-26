# System Architecture Diagram

## Hardware Setup
```
                    Drone
        ┌───────────────────────────┐
        │                           │
        │  ┌─────────────────────┐  │
        │  │   Flight Controller │  │
        │  │       (PX4)         │  │
        │  └──────────┬──────────┘  │
        │             │ Serial/UART │
        │             │             │
        │  ┌──────────▼──────────┐  │
        │  │  Companion Computer │  │
        │  │  (Raspberry Pi)     │  │
        │  └──────────┬──────────┘  │
        │             │ USB         │
        │  ┌──────────▼──────────┐  │
        │  │    USB Camera       │  │
        │  │   (C270) [DOWN]     │  │
        │  └─────────────────────┘  │
        │             │             │
        └─────────────┼─────────────┘
                      │
                      │ Field of View
                      ▼
              ┌───────────────┐
              │  ArUco Marker │
              │   (15cm x 15cm)│
              │   ID: 0        │
              └───────────────┘
                   On Floor
```

## Software Architecture
```
┌──────────────────────────────────────────────────────┐
│              Companion Computer (ROS2)               │
│                                                      │
│  ┌────────────┐    ┌──────────────┐    ┌─────────┐ │
│  │  usb_cam   │───>│aruco_tracker │───>│ vision_ │ │
│  │   node     │    │    node      │    │position │ │
│  │            │    │              │    │ bridge  │ │
│  └────────────┘    └──────────────┘    └────┬────┘ │
│       │                   │                  │      │
│       │ /camera/image     │ /target_pose     │      │
│       │                   │                  │      │
└───────┼───────────────────┼──────────────────┼──────┘
        │                   │                  │
        ▼                   ▼                  ▼
   Images (30Hz)    Marker Pose (30Hz)  Vision Odom (30Hz)
                                               │
                                               │ uXRCE-DDS
                                               ▼
┌──────────────────────────────────────────────────────┐
│              Flight Controller (PX4)                 │
│                                                      │
│  ┌─────────────────────────────────────────────┐   │
│  │         EKF2 (Extended Kalman Filter)        │   │
│  │                                              │   │
│  │   IMU ───┐                                   │   │
│  │          ├──> Sensor Fusion ──> Position    │   │
│  │  Vision ─┘                       Estimate    │   │
│  │                                              │   │
│  └──────────────────┬───────────────────────────┘   │
│                     │                               │
│  ┌──────────────────▼───────────────────────────┐   │
│  │         Position Controller                  │   │
│  │  (Offboard mode / Position hold)             │   │
│  └──────────────────┬───────────────────────────┘   │
│                     │                               │
│                     ▼                               │
│              Motor Commands                         │
│                                                      │
└──────────────────────────────────────────────────────┘
```

## Data Flow
```
Camera Frame           Vision Bridge         PX4 Frame
(OpenCV)              (Conversion)           (NED)

    Y ▲                                        ▲ Z (Down)
      │                                        │
      │    Z (Forward)                         │
      │  ╱                                     │
      │ ╱                                      │
      │╱                                       │
      └─────────> X (Right)    ══════>    X ──┴──> Y
                                          (North) (East)

Camera detects:                   PX4 receives:
  X = 0.5 (right)                  X = -0.5 (left)
  Y = 0.3 (down)        ═════>     Y = -0.5 (back)
  Z = 2.0 (forward)                Z = -0.3 (altitude)
```

## ROS2 Topics
```
Input Topics:
  /camera_down/image_raw              (sensor_msgs/Image)
  /camera_down/camera_info            (sensor_msgs/CameraInfo)

Internal Topics:
  /target_pose_down                   (geometry_msgs/PoseStamped)
  /image_proc_down                    (sensor_msgs/Image) - with overlay

Output Topics:
  /fmu/in/vehicle_visual_odometry     (px4_msgs/VehicleOdometry)
```

## PX4 Configuration Flow
```
QGroundControl
      │
      │ MAVLink
      ▼
┌────────────────────┐
│  PX4 Parameters    │
│                    │
│  EKF2_AID_MASK=24 ─┼──> Enables vision fusion
│  EKF2_HGT_REF=3 ───┼──> Uses vision for height
│  COM_RCL_EXCEPT=4 ─┼──> Allows offboard w/o RC
│                    │
└────────────────────┘
      │
      ▼
┌────────────────────┐
│  EKF2 Module       │
│  (State Estimator) │
└────────┬───────────┘
         │
         ▼
   Position Available!
         │
         ▼
   Offboard Can ARM ✓
```

## Calibration Flow
```
1. Print Checkerboard
        │
        ▼
2. camera_calibration Tool
        │
        ├──> Capture Multiple Views
        │    - Left/Right
        │    - Top/Bottom  
        │    - Tilted angles
        │
        ▼
3. Compute Calibration
        │
        ├──> Camera Matrix (fx, fy, cx, cy)
        └──> Distortion Coeffs (k1, k2, p1, p2, k3)
        │
        ▼
4. Save to YAML
        │
        ▼
5. Used by ArUco Tracker
        │
        ▼
   Accurate Pose Estimation ✓
```

## Coordinate Frames Reference

### Camera Frame (OpenCV Convention)
- Origin: Camera optical center
- X: Right
- Y: Down  
- Z: Forward (into scene)

### Body Frame (PX4 / FRD)
- Origin: Vehicle center of mass
- X: Forward (Front)
- Y: Right
- Z: Down

### NED Frame (PX4 / Earth-fixed)
- Origin: Marker position (or GPS home)
- X: North
- Y: East
- Z: Down

### Transformations
```
Camera → Body:
  For downward camera:
    Body_X = Camera_Z
    Body_Y = Camera_X  
    Body_Z = Camera_Y

Body → NED:
  Depends on vehicle orientation (yaw)
  Handled by PX4 EKF2
```
