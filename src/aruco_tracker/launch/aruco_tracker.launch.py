from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Set FastDDS configuration file
    # fastdds_config = PathJoinSubstitution([FindPackageShare('precision_land'), 'config', 'fastdds_configuration.xml'])
    
    return LaunchDescription([
        # Set FastDDS configuration environment variable
        # SetEnvironmentVariable(
        #     name='FASTRTPS_DEFAULT_PROFILES_FILE',
        #     value=fastdds_config
        # ),

        # Run bridge nodes directly in the launch file
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_down_bridge',
            output='screen',
            arguments=['/world/warehouse/model/x500_mono_cam_down_0/link/camera_link_down/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            remappings=[('/world/warehouse/model/x500_mono_cam_down_0/link/camera_link_down/sensor/imager/image', '/camera_down/image_raw')],
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
            name='camera_info_down_bridge',
            output='screen',
            arguments=['/world/warehouse/model/x500_mono_cam_down_0/link/camera_link_down/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
            remappings=[('/world/warehouse/model/x500_mono_cam_down_0/link/camera_link_down/sensor/imager/camera_info', '/camera_down/camera_info')],
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
            name='camera_forward_bridge',
            output='screen',
            arguments=['/world/warehouse/model/x500_mono_cam_down_0/link/camera_link_forward/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            remappings=[('/world/warehouse/model/x500_mono_cam_down_0/link/camera_link_forward/sensor/imager/image', '/camera_forward/image_raw')],
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
            name='camera_info_forward_bridge',
            output='screen',
            arguments=['/world/warehouse/model/x500_mono_cam_down_0/link/camera_link_forward/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
            remappings=[('/world/warehouse/model/x500_mono_cam_down_0/link/camera_link_forward/sensor/imager/camera_info', '/camera_forward/camera_info')],
            additional_env={
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{name}]: {message}',
                'RCUTILS_LOGGING_USE_STDOUT': '1',
                'RCUTILS_LOGGING_BUFFERED_STREAM': '1'
            }
        ),

        # Continue running Aruco tracker node as a normal ROS node and display output on screen
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
    ])