from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare the port argument
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port for GPS device'
        ),
        
        # Launch the GNSS driver node
        Node(
            package='gnss',
            executable='gnss_driver.py',
            name='gnss_driver',
            output='screen',
            parameters=[],
            arguments=[LaunchConfiguration('port')]
        )
    ])
