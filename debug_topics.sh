#!/bin/bash

echo "=== Checking ArUco-related topics ==="
echo "Available topics:"
ros2 topic list | grep -E "(target_pose|camera|image)"

echo ""
echo "=== Checking /target_pose_down topic info ==="
ros2 topic info /target_pose_down

echo ""
echo "=== Listening to /target_pose_down for 5 seconds ==="
timeout 5s ros2 topic echo /target_pose_down --max-count 3

echo ""
echo "=== Checking camera topics ==="
ros2 topic list | grep camera

echo ""
echo "=== Topic publication rates ==="
ros2 topic hz /target_pose_down &
PID1=$!
ros2 topic hz /camera_down/image_raw &
PID2=$!

sleep 10

kill $PID1 $PID2 2>/dev/null

echo "Debug complete!"
