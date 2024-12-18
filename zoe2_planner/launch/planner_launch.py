from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

declared_arguments = []

declared_arguments.append(
    DeclareLaunchArgument(
        "robot_x",
        default_value="0.0",
        description='X position of the robot',
    )
)
declared_arguments.append(
    DeclareLaunchArgument(
        "robot_y",
        default_value="0.0",
        description='Y position of the robot',
    )
)
declared_arguments.append(
    DeclareLaunchArgument(
        "robot_theta",
        default_value="0.0",
        description='yaw of the robot',
    )
)

declared_arguments.append(
    DeclareLaunchArgument(
        "goal_x",
        default_value="0.0",
        description='X position of the goal',
    )
)

declared_arguments.append(
    DeclareLaunchArgument(
        "goal_y",
        default_value="0.0",
        description='Y position of the goal',
    )
)

def generate_launch_description():
    return LaunchDescription(
        declared_arguments +
        [
        Node(
            package='zoe2_planner',  
            executable='a_star_planner_node', 
            name='a_star_planner_node',
            output='screen',
            parameters=[
                {'robot_x': LaunchConfiguration('robot_x')},
                {'robot_y': LaunchConfiguration('robot_y')},
                {'robot_theta': LaunchConfiguration('robot_theta')},
                {'goal_x': LaunchConfiguration('goal_x')},
                {'goal_y': LaunchConfiguration('goal_y')},
                # {'use_sim_time': True}
            ]
        )
    ])
