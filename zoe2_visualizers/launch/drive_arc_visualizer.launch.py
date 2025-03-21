import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, EqualsSubstitution  # Import EqualsSubstitution
from launch_ros.actions import Node  # Correct import for Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use simulation time'),

        # Node to launch the drive_arc_visualizer from zoe2_visualizers package
        Node(
            package='zoe2_visualizers',  # The package where your node is defined
            executable='drive_arc_visualizer',  # The node executable you want to run
            name='drive_arc_visualizer_node',  # Node name
            output='screen',  # Output to screen
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            # remappings=[  # Optional: remap topics if needed
            #     ('/cmd_vel_unstamped', '/your/command/topic'),
            # ]
        ),
        # Log message to confirm launch
        LogInfo(
            condition=launch.conditions.IfCondition(
                EqualsSubstitution(LaunchConfiguration('use_sim_time'), 'false')),
            msg="Drive Arc Visualizer node has been launched.")
    ])
