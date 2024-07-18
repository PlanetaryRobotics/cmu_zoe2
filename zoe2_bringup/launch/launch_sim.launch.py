import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

from launch.event_handlers import OnProcessExit

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

    # Unwrap Launch Arguments
    world_file = LaunchConfiguration("world")
    x_pos = LaunchConfiguration('x')
    y_pos = LaunchConfiguration('y')
    z_pos = LaunchConfiguration('z')

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    bringup_package_name="zoe2_bringup"

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(bringup_package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    
    world_path = PathJoinSubstitution([FindPackageShare(bringup_package_name), 'worlds', world_file])

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': world_path}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'zoe2',
                                   '-x', x_pos,
                                   '-y', y_pos,
                                   '-z', z_pos,
                                   ],
                        output='screen')


    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[diff_drive_spawner],
        )
    )    

    # Launch them all!
    return LaunchDescription(
        declared_arguments +
        [
        rsp,
        gazebo,
        spawn_entity,
        joint_broad_spawner,
        delayed_diff_drive_spawner,
    ])