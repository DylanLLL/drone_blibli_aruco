#!/bin/bash

# Camera Calibration Helper Script for Logitech C270

echo "========================================="
echo "Camera Calibration Script"
echo "========================================="
echo ""
echo "This script will help you calibrate your Logitech C270 camera"
echo ""

# Check if camera is connected
if [ ! -e /dev/video0 ]; then
    echo "ERROR: No camera found at /dev/video0"
    echo "Please check your camera connection"
    echo ""
    echo "Available video devices:"
    ls /dev/video* 2>/dev/null || echo "None found"
    exit 1
fi

echo "✓ Camera found at /dev/video0"
echo ""

# Check if usb_cam is installed
if ! ros2 pkg list | grep -q usb_cam; then
    echo "ERROR: usb_cam package not installed"
    echo "Install with: sudo apt install ros-humble-usb-cam"
    exit 1
fi

echo "✓ usb_cam package installed"
echo ""

# Check if camera_calibration is installed
if ! ros2 pkg list | grep -q camera_calibration; then
    echo "ERROR: camera_calibration package not installed"
    echo "Install with: sudo apt install ros-humble-camera-calibration"
    exit 1
fi

echo "✓ camera_calibration package installed"
echo ""

echo "========================================="
echo "Calibration Instructions:"
echo "========================================="
echo ""
echo "1. Print the checkerboard pattern:"
echo "   wget https://raw.githubusercontent.com/opencv/opencv/master/doc/pattern.png"
echo "   (Print at actual size - should be 8x6 squares, 25mm each)"
echo ""
echo "2. This script will start the camera and calibration tool"
echo ""
echo "3. In the calibration window:"
echo "   - Move the checkerboard around the camera view"
echo "   - Cover LEFT, RIGHT, TOP, BOTTOM, and different ANGLES"
echo "   - Fill all the progress bars (X, Y, Size, Skew)"
echo "   - When CALIBRATE button is enabled, click it"
echo "   - Wait for calibration to complete (may take a minute)"
echo "   - Click SAVE button"
echo "   - Calibration will be saved to /tmp/calibrationdata.tar.gz"
echo ""
echo "4. After saving, this script will extract and show the calibration"
echo ""

read -p "Press Enter to start calibration..."

echo ""
echo "Starting USB camera..."
echo ""

# Start camera in background
ros2 run usb_cam usb_cam_node_exe \
    --ros-args \
    -p video_device:=/dev/video0 \
    -p image_width:=640 \
    -p image_height:=480 \
    -p pixel_format:=yuyv \
    -p framerate:=30.0 \
    -r __node:=usb_cam_calibration &

CAMERA_PID=$!

# Give camera time to start
sleep 3

echo "Starting calibration tool..."
echo ""
echo "Calibration window should open now..."
echo "Follow the instructions above!"
echo ""

# Start calibration
ros2 run camera_calibration cameracalibrator \
    --size 8x6 \
    --square 0.025 \
    --ros-args \
    -r image:=/image_raw \
    -r camera:=/usb_cam

# Kill camera node
kill $CAMERA_PID 2>/dev/null

echo ""
echo "========================================="
echo "Calibration Complete!"
echo "========================================="
echo ""

# Check if calibration file exists
if [ -f /tmp/calibrationdata.tar.gz ]; then
    echo "✓ Calibration data saved"
    echo ""
    echo "Extracting calibration data..."
    cd /tmp
    tar -xzf calibrationdata.tar.gz
    
    if [ -f /tmp/ost.yaml ]; then
        echo ""
        echo "========================================="
        echo "Your Camera Calibration Parameters:"
        echo "========================================="
        cat /tmp/ost.yaml
        echo ""
        echo "========================================="
        echo "Next Steps:"
        echo "========================================="
        echo ""
        echo "Copy the calibration data to:"
        echo "  ~/tracktor-beam/src/aruco_tracker/cfg/logitech_c270_calibration.yaml"
        echo ""
        echo "You can do this manually or run:"
        echo "  cp /tmp/ost.yaml ~/tracktor-beam/src/aruco_tracker/cfg/logitech_c270_calibration.yaml"
        echo ""
    else
        echo "WARNING: Calibration file not found at /tmp/ost.yaml"
    fi
else
    echo "WARNING: No calibration data found at /tmp/calibrationdata.tar.gz"
    echo "Make sure you clicked the SAVE button in the calibration tool"
fi

echo ""
echo "Done!"
