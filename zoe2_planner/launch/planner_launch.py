from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='zoe2_planner',  
            executable='a_star_planner_node', 
            name='a_star_planner_node',
            output='screen',
            # parameters=[
            #     {'use_sim_time': True}
            # ]
        )
    ])
