import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

from launch.event_handlers import OnProcessExit

bringup_package_name="zoe2_bringup"
odom_package_name = 'zoe2_odom'

def generate_launch_description():

    # Declare Launch Arguments. 
    # These can be called by ros2 launch <package_name> <launch_file.py> <argname_1>:=<value> <argname_1>:=<value> ...
    declared_arguments = []

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("zoe2_bringup"),
            "config",
            "zoe2_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )

    # launch the robot state publisher
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(bringup_package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'sim': 'false'}.items()
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    zoe_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zoe_controller"],
    )

    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=zoe_controller_spawner,
            on_exit=[joint_broad_spawner],
        )
    )


    # Launch them all!
    return LaunchDescription(
        declared_arguments +
        [
        control_node,
        rsp,
        zoe_controller_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
    ])