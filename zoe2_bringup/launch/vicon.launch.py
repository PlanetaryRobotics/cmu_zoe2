import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vicon_bridge',
            executable='vicon_bridge',
            name='vicon_bridge',
            output='screen',
            parameters=[{'host_name': '192.168.10.1:801'}],
            arguments=['--ros-args', '--log-level', 'ERROR'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_pub',
            output='screen',
            arguments=['0', '0.3', '-.47', '3.1415', '0', '0', 'vicon/morphin/morphin', 'base_link']
            # transform between morphin's lid to zoe's base_link, centered on chassis and level with wheels
        )
    ])
