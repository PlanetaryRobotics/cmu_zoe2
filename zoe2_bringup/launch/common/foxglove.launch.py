import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # launch the vectornav sensor
    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        output="screen"
    )

    return LaunchDescription([
        foxglove_bridge
    ])