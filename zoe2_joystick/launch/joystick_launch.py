from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'deadzone',
            default_value='0.05'),
        Node(
            package='joy',
            namespace='zoe2',
            executable='joy_node',
            name='joystick',
            parameters=[{'deadzone': LaunchConfiguration('deadzone'),
                         'joy_config': 'xbox',  # Change 'xbox' to your joystick type if needed
                         'joy_dev': '/dev/input/js0'}]  # Change this if your joystick is on a different device
        ),
        DeclareLaunchArgument(
            'publish_stamped_twist',
            default_value='false'),
        DeclareLaunchArgument(
            'require_enable_button', # this is the deadman switch
            default_value='true'),
        Node(
            package='teleop_twist_joy',
            namespace='zoe2',
            executable='teleop_node',
            name='joy2twist',
            remappings=[('/zoe2/cmd_vel', '/cmd_vel_unstamped')],

            parameters=[{
                'publish_stamped_twist': LaunchConfiguration('publish_stamped_twist'),
                'require_enable_button': LaunchConfiguration('require_enable_button'),

                # #these parameters work for TX16 joystick 
                # 'enable_button': 3,  # Button SH is the deadman switch
                # 'axis_linear.x': 1,  # Left joystick Y-axis
                # 'axis_angular.yaw': 3,  # Right joystick X-axis
                # 'inverted_reverse': False,  # whether to invert turning lef-right while reversing 

                #these parameters work for logitech F710 joystick 
                'enable_button': 5,  # Button RB (right bottom) is the deadman switch
                'enable_turbo_button': 7, # Button LB (left bottom) is the turbo button
                'axis_linear.x': 3,  # Left joystick Y-axis
                'axis_angular.yaw': 2,  # Right joystick X-axis
                'inverted_reverse': False,  # whether to invert turning lef-right while reversing 
            }]
        ),
        
    ])