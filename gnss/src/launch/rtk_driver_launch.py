from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port for RTK GPS'
        ),
        Node(
            package='gnss',
            executable='rtk_driver.py',
            name='rtk_driver',
            output='screen',
            parameters=[],
            arguments=[LaunchConfiguration('port')]
        )
    ])
