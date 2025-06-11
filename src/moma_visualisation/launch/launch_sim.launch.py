import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    package_name = 'moma_visualisation'
    pkg_path = os.path.join(get_package_share_directory(package_name))
    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(pkg_path, 'launch', 'rsp.launch.py')]), 
            launch_arguments={'use_sim_time': 'true'}.items()
    )


    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                launch_arguments={"gz_args":[' -r '+ os.path.join(pkg_path,'worlds','obstacles.sdf')]}.items(),
                condition=UnlessCondition(LaunchConfiguration('no_gazebo'))
    )
    
    # Launch SLAM toolbox
    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]), 
                launch_arguments={'slam_params_file': [os.path.join(pkg_path, 'config', 'mapper_params_online_async.yaml'),], 
                                  'use_sim_time':['true',],}.items()
    )
    
    # Launch Nav2 stack
    nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(pkg_path,'launch','navigation.launch.py')]), 
            launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        arguments=['-entity', 'my_robot', '-topic', 'robot_description'],
        output='screen'
    )

    # Joint State Broadcaster
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # Base controller
    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_base_controller'],
        output='screen'
    )

    # Robot arm controller
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
        output='screen'
    )

    # Send controlled base velocity to gazebo
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=['/diff_drive_base_controller/cmd_vel_unstamped@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )

    # Send commanded base velocity to gazebo
    cmd_vel_gazebo_internal_teleop_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_gazebo_internal_teleop_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen',
        ros_arguments=['-r', '/cmd_vel:=/diff_drive_base_controller/cmd_vel_unstamped']
    )

    # Send Lidar scans to gazebo
    amr_lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='amr_lidar_bridge',
        arguments=['/amr_laser/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        output='screen'
    )

    # Send odom to gazebo
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        output='screen'
    )

    # Bridge for clock
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    twist_mux_params = os.path.join(pkg_path,'config','twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out','/diff_drive_base_controller/cmd_vel_unstamped')]
    )
    
    
    # Launch them all
    return LaunchDescription([
        DeclareLaunchArgument('no_gazebo', default_value='false', description='Prevents the launch of Gazebo if True'),
        clock_bridge,
        amr_lidar_bridge,
        odom_bridge,
        cmd_vel_gazebo_internal_teleop_bridge,
        cmd_vel_bridge,
        twist_mux,
        rsp,
        gazebo,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_drive_controller,load_arm_controller,slam_toolbox,
        nav2,],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        spawn_entity,
    ])