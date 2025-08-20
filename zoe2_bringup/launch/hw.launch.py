import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition

bringup_package_name="zoe2_bringup"

def generate_launch_description():

    # Declare Launch Arguments. 
    # These can be called by ros2 launch <package_name> <launch_file.py> <argname_1>:=<value> <argname_1>:=<value> ...
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_joystick",
            default_value="true",
            description='Use joystick for control if true',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description='Run in headless mode if true',
        )
    )

    # unwrap Launch Arguments
    use_joystick = LaunchConfiguration("use_joystick")
    headless = LaunchConfiguration("headless")

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("zoe2_bringup"),
            "config",
            "zoe2_hw_controllers.yaml",
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
                    get_package_share_directory(bringup_package_name),'launch','common','rsp.launch.py'
                )]), launch_arguments={'sim': 'false'}.items()
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    zoe2_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zoe2_controller"],
    )

    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=zoe2_controller_spawner,
            on_exit=[joint_broad_spawner],
        )
    )

    # launch rviz
    rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(bringup_package_name),'launch','common','rviz.launch.py'
            )]),
            launch_arguments={
                "rviz_config_file": PathJoinSubstitution([FindPackageShare("zoe2_bringup"), "rviz", "wheel_speeds.rviz"]),
                "use_sim_time": "false",
            }.items(),
            condition=UnlessCondition(headless)
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[rviz],
        )
    )

    # launch the drive_arc_visualizer node
    drive_arc_visualizer_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("zoe2_visualizers"),'launch','drive_arc_visualizer.launch.py'
        )])
    )

    # launch the vectornav sensor
    vectornav_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("vectornav"),'launch','vectornav.launch.py'
        )])
    )

    # If joystick is enabled, launch the joystick node
    joystick_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("zoe2_joystick"),'launch','joystick.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(use_joystick)
    )

    # Launch them all!
    return LaunchDescription(
        declared_arguments +
        [
        control_node,
        rsp,
        zoe2_controller_spawner,
        vectornav_node,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        drive_arc_visualizer_node,
        joystick_node
    ])