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
                         'joy_config': 'xbox'}],  # Change 'xbox' to your joystick type if needed
                         'joy_dev': '/dev/input/js0'  # Change this if your joystick is on a different device
        ),
        DeclareLaunchArgument(
            'publish_stamped_twist',
            default_value='true'),
        DeclareLaunchArgument(
            'require_enable_button', # this is the deadman switch
            default_value='true'),
        Node(
            package='teleop_twist_joy',
            namespace='zoe2',
            executable='teleop_node',
            name='joy2twist',
            parameters=[{
                'publish_stamped_twist': LaunchConfiguration('publish_stamped_twist'),
                'require_enable_button': LaunchConfiguration('require_enable_button'),
                # 'enable_button': 0,  # Button 0 is the deadman switch
                # 'axis_linear_x': 1,  # Left joystick Y-axis
                # 'axis_linear_y': 0,  # Left joystick X-axis
                # 'axis_angular_z': 3,  # Right joystick X-axis
                # 'axis_angular_y': 4,  # Right joystick Y-axis
                'inverted_reverse': True,  # whether to invert turning lef-right while reversing 
            }]
        ),
        
    ])