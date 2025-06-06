from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port where the LIDAR is connected'
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='38400',
            description='Baud rate for serial communication'
        ),
        DeclareLaunchArgument(
            'node_name',
            default_value='sicklms',
            description='Name of the node'
        ),
        Node(
            package='sicktoolbox_wrapper',
            executable='sicklms',
            name=LaunchConfiguration('node_name'),
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud')
            }]
        )
    ])

