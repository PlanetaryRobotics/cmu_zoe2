from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('zoe2_bringup')
    hw_launch = os.path.join(bringup_dir, 'launch', 'hw.launch.py')
    vicon_launch = os.path.join(bringup_dir, 'launch', 'vicon.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(hw_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(vicon_launch)),
    ])
