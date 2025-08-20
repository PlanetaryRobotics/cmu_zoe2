import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # launch the vectornav sensor
    vectornav_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("vectornav"),'launch','vectornav.launch.py'
        )])
    )

    return LaunchDescription([
        vectornav_node
    ])