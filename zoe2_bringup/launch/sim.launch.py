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
    declared_arguments.append(
        DeclareLaunchArgument(
            "heading",
            default_value="0.0",
            description='yaw of the robot',
        )
    )

    # Unwrap Launch Arguments
    world_file = LaunchConfiguration("world")

    world_path = PathJoinSubstitution([FindPackageShare(bringup_package_name), 'worlds', world_file])

    # launch the robot state publisher
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(bringup_package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'sim': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world_path], 'on_exit_shutdown': 'true'}.items()
             )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'zoe2',
                                   '-x', LaunchConfiguration('x'),
                                   '-y', LaunchConfiguration('y'),
                                   '-z', LaunchConfiguration('z'),
                                   '-Y', LaunchConfiguration('heading'),
                                   ],
                        output='screen')

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

    delayed_zoe2_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[zoe2_controller_spawner],
        )
    )

    bridge_params = os.path.join(get_package_share_directory(bringup_package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    # launch the odom_tf_broadcaster node
    odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(odom_package_name),'launch','odom_tf_broadcaster.launch.py'
        )])
    )

    # launch rviz
    rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(bringup_package_name),'launch','rviz.launch.py'
            )]),
            launch_arguments={
                "rviz_config_file": PathJoinSubstitution([FindPackageShare("zoe2_bringup"), "rviz", "sim.rviz"]),
            }.items()
    )

    # launch the drive_arc_visualizer node
    drive_arc_visualizer_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("zoe2_visualizers"),'launch','drive_arc_visualizer.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )


    # Launch them all!
    return LaunchDescription(
        declared_arguments +
        [
        rsp,
        gazebo,
        spawn_entity,
        joint_broad_spawner,
        delayed_zoe2_controller_spawner,
        ros_gz_bridge,
        odom,
        rviz,
        drive_arc_visualizer_node
    ])