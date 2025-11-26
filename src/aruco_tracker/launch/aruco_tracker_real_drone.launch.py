from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Launch file for real drone with Logitech C270 USB webcam
    This will:
    1. Start the USB camera driver (usb_cam)
    2. Start the ArUco tracker
    3. Bridge vision position to PX4
    """
    
    return LaunchDescription([
        # USB Camera Node (Logitech C270)
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',  # Change if your camera is on different device
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'yuyv',
                'camera_frame_id': 'camera_optical_frame',
                'io_method': 'mmap',
                'framerate': 30.0,
                'camera_name': 'logitech_c270',
                'camera_info_url': 'file://' + PathJoinSubstitution([
                    FindPackageShare('aruco_tracker'), 
                    'cfg', 
                    'logitech_c270_calibration.yaml'
                ]).perform(None)  # We'll create this calibration file
            }],
            remappings=[
                ('/image_raw', '/camera_down/image_raw'),
                ('/camera_info', '/camera_down/camera_info')
            ]
        ),

        # ArUco Tracker Node
        Node(
            package='aruco_tracker',
            executable='aruco_tracker',
            name='aruco_tracker',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('aruco_tracker'), 'cfg', 'params_real_drone.yaml'])
            ]
        ),

        # Vision Position Bridge to PX4
        Node(
            package='aruco_tracker',
            executable='vision_position_bridge',
            name='vision_position_bridge',
            output='screen',
            parameters=[{
                'marker_size': 0.15,  # 15cm marker size - MEASURE YOUR PRINTED MARKERS!
            }]
        ),
    ])
