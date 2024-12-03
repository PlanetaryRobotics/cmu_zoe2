from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("zoe2_bringup"), "rviz", "zoe2.rviz"
            ]),
            description="Full path to the RViz configuration file to use",
        ),
    ]

    # Use LaunchConfiguration to retrieve the value of the rviz_config_file argument
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    log_rviz_path = LogInfo(msg=["Using RViz config file: ", rviz_config_file])

    # Define the rviz2 node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(
        declared_arguments
        + [
            log_rviz_path,
            rviz_node,
        ]
    )