import os
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

bringup_package_name = 'zoe2_bringup'
odom_package_name = 'zoe2_odom'
planner_package_name = 'zoe2_planner' 


def generate_launch_description():
    # Declare Launch Arguments. 
    # These can be called by ros2 launch <package_name> <launch_file.py> <argname_1>:=<value> <argname_1>:=<value> ...
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value="empty.world",
            description='World to load into Gazebo',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "x",
            default_value="0.0",
            description='X position of the robot',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "y",
            default_value="0.0",
            description='Y position of the robot',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "z",
            default_value="0.1",
            description='Z position of the robot',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "yaw",
            default_value="0.0",
            description='yaw of the robot',
        )
    )

    # launch the odom_tf_broadcaster node
    odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(odom_package_name),'launch','odom_tf_broadcaster.launch.py'
        )])
    )
    
    # launch rviz
    rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(bringup_package_name),'launch','rviz.launch.py'
            )])
    )

    # call launch_sim.launch.py
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(bringup_package_name),'launch','sim.launch.py'
        )]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw')
        }.items()
    )

    # Include the planner launch file
    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(planner_package_name), 'launch', 'planner_launch.py'
        )])
    )
    return LaunchDescription(
        declared_arguments + 
        [
        odom,
        rviz,
        sim,
        planner
    ])