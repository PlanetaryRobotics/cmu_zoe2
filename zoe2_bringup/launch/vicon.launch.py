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
            arguments=['--x', '0', 
                        '--y', '0.0', 
                        '--z', '-.47', 
                        '--yaw', '3.1415', 
                        '--pitch', '0', 
                        '--roll', '0', 
                        '--frame-id', 'vicon/morphin/morphin', 
                        '--child-frame-id', 'base_link']
            # transform between morphin's lid to zoe's base_link, centered on chassis and level with wheels
            # args are (x, y, z) in m, (yaw, pitch roll) in rad
        )
    ])
