import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'zoe_odom', 'odom_tf_broadcaster', '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen'
        ),
    ])
