from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Set FastDDS configuration file
    fastdds_config = PathJoinSubstitution([FindPackageShare('precision_land'), 'config', 'fastdds_configuration.xml'])
    
    return LaunchDescription([
        # Run bridge nodes directly in the launch file
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            output='screen',
            arguments=['/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            remappings=[('/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image', '/camera')],
            additional_env={
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{name}]: {message}',
                'RCUTILS_LOGGING_USE_STDOUT': '1',
                'RCUTILS_LOGGING_BUFFERED_STREAM': '1'
            }
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_info_bridge',
            output='screen',
            arguments=['/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
            remappings=[('/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info', '/camera_info')],
            additional_env={
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{name}]: {message}',
                'RCUTILS_LOGGING_USE_STDOUT': '1',
                'RCUTILS_LOGGING_BUFFERED_STREAM': '1'
            }
        ),
        # Run Aruco tracker node
        Node(
            package='aruco_tracker',
            executable='aruco_tracker',
            name='aruco_tracker',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('aruco_tracker'), 'cfg', 'params.yaml'])
            ],
            additional_env={
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{name}]: {message}',
                'RCUTILS_LOGGING_USE_STDOUT': '1',
                'RCUTILS_LOGGING_BUFFERED_STREAM': '1'
            }
        ),
        # Run simple precision land node
        Node(
            package='aruco_tracker',
            executable='simple_precision_land',
            name='simple_precision_land',
            output='screen',
            parameters=[{
                'descent_velocity': 0.3,
                'hover_height': 3.0,
                'land_threshold': 0.1
            }],
            additional_env={
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{name}]: {message}',
                'RCUTILS_LOGGING_USE_STDOUT': '1',
                'RCUTILS_LOGGING_BUFFERED_STREAM': '1'
            }
        ),
    ])
