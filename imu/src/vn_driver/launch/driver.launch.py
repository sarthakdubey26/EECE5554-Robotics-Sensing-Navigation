from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port for VectorNav IMU'
        ),
        
        Node(
            package='vn_driver',
            executable='vn_driver',
            name='vn_driver',
            output='screen',
            arguments=[LaunchConfiguration('port')]
        )
    ])
