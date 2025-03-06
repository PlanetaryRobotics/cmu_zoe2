import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Use sim time if true'
        ),
    ]

    # Check if we're launching in sim
    sim = LaunchConfiguration('sim')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('zoe2_description'))
    xacro_file = os.path.join(pkg_path,'urdf','zoe2.urdf.xacro')

    xacro_params = {
        'sim_gazebo': 'true'
    }

    robot_description_config = xacro.process_file(xacro_file, mappings=xacro_params)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': sim}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    # Launch!
    return LaunchDescription(
        declared_arguments + 
        [node_robot_state_publisher]
    )