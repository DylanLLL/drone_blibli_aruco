from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    # Set FastDDS configuration file
    fastdds_config = PathJoinSubstitution([FindPackageShare('precision_land'), 'config', 'fastdds_configuration.xml'])
    
    return LaunchDescription([
        # Set FastDDS configuration environment variable
        SetEnvironmentVariable(
            name='FASTRTPS_DEFAULT_PROFILES_FILE',
            value=fastdds_config
        ),
        Node(
            package='precision_land',
            executable='precision_land',
            name='precision_land',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('precision_land'), 'cfg', 'params.yaml'])
            ],
            # Add RMW configuration
            additional_env={
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{name}]: {message}',
                'RCUTILS_LOGGING_USE_STDOUT': '1',
                'RCUTILS_LOGGING_BUFFERED_STREAM': '1'
            }
        ),
    ])
