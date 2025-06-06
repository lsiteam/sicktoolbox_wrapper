from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sicktoolbox_wrapper',
            executable='sicklms',
            name='sicklms_node',
            output='screen'
        ),
        Node(
            package='sicktoolbox_wrapper',
            executable='log_scans',
            name='log_scans_node',
            output='screen'
        ),
        Node(
            package='sicktoolbox_wrapper',
            executable='print_scans',
            name='print_scans_node',
            output='screen'
        ),
        Node(
            package='sicktoolbox_wrapper',
            executable='time_scans',
            name='time_scans_node',
            output='screen'
        ),
    ])
