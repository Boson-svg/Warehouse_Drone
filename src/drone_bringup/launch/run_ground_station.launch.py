from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_qt',
            executable='ground_link_bridge',
            name='ground_link_bridge',
            output='screen',
        ),
        Node(
            package='drone_qt',
            executable='ground_station',
            name='ground_station',
            output='screen',
        ),
    ])
