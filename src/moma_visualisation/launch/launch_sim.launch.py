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

from moveit_configs_utils import MoveItConfigsBuilder

import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


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
                launch_arguments={"gz_args":[' -r '+ os.path.join(pkg_path,'worlds','ign_lab_save.sdf')]}.items(), # husky_depot.sdf
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
    load_joint_state_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    
    load_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_base_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    load_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robotiq_gripper_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
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

    cam_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[             # ign topic -t <topic_name> --info
            '/depth_camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/depth_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
        ],
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

    moveit_config = (
        MoveItConfigsBuilder("robot", package_name="tm12s_moveit_config")
        .robot_description(file_path="config/robot.urdf.xacro")
        .robot_description_semantic(file_path="config/robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description= True, publish_robot_description_semantic=True, publish_planning_scene=True
        )
        .to_moveit_configs()
    )

    # More Moveit configs
    planning_pipeline_config = {
          'planning_pipelines' : ['ompl', 'pilz_industrial_motion_planner'],
          'ompl': {
                'planning_plugin': 'ompl_interface/OMPLPlanner',
                'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization 
                                       default_planner_request_adapters/FixWorkspaceBounds 
                                       default_planner_request_adapters/FixStartStateBounds 
                                       default_planner_request_adapters/FixStartStateCollision 
                                       default_planner_request_adapters/FixStartStatePathConstraints
                                       default_planner_request_adapters/ResolveConstraintFrames""",
                'start_state_max_bounds_error': 0.1,
          },
          'pilz_industrial_motion_planner': {
                'planning_plugin': 'pilz_industrial_motion_planner/CommandPlanner',
                'request_adapters': '',
                'start_state_max_bounds_error': 0.1,
          }
    }

    ompl_planning_yaml = load_yaml('tm12s_moveit_config', 'config/ompl_planning.yaml')
    planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    pilz_planning_yaml = load_yaml('tm12s_moveit_config', 'config/pilz_industrial_motion_planner_planning.yaml')
    planning_pipeline_config['pilz_industrial_motion_planner'].update(pilz_planning_yaml)

    controllers_yaml = load_yaml('tm12s_moveit_config', 'config/moveit_controllers.yaml')
    moveit_controllers = {'moveit_simple_controller_manager': controllers_yaml,
                          'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}
    
    joint_limits_yaml = {
        'robot_description_planning': load_yaml('tm12s_moveit_config', 'config/joint_limits.yaml')
    }

    cartesian_limits_yaml = load_yaml('tm12s_moveit_config', 'config/pilz_cartesian_limits.yaml')

    rviz_config_path = os.path.join(
        get_package_share_directory("tm12s_moveit_config"),
        "config",
        "moveit.rviz",
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            planning_pipeline_config,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_scene_monitor,
            {'use_sim_time': True}
        ],
    )

    use_sim_time={"use_sim_time": True}
    config_dict = moveit_config.to_dict()
    config_dict.update(use_sim_time)
    config_dict.update(planning_pipeline_config)
    config_dict.update(moveit_controllers)
    config_dict.update(joint_limits_yaml)
    config_dict.update(cartesian_limits_yaml)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config_dict],
        arguments=["--ros-args", "--log-level", "info"],
    )
    
    
    # Launch them all
    return LaunchDescription([
        DeclareLaunchArgument('no_gazebo', default_value='false', description='Prevents the launch of Gazebo if True'),
        clock_bridge,
        amr_lidar_bridge,
        odom_bridge,
        cmd_vel_gazebo_internal_teleop_bridge,
        cmd_vel_bridge,
        cam_bridge,
        twist_mux,
        rsp,
        gazebo,
        rviz_node,
        move_group_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_drive_controller,load_arm_controller,slam_toolbox,load_gripper_controller,nav2,],
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