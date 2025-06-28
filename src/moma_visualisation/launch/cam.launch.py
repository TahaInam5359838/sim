# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # gz_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    #     launch_arguments={'gz_args': '-r camera_sensor.sdf'}.items(),
    # )

    # # RViz
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', os.path.join(pkg_ros_gz_sim_demos, 'rviz', 'camera.rviz')],
    #     condition=IfCondition(LaunchConfiguration('rviz'))
    # )

    # Bridge
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=[#'/test/integration/CameraPlugin_imagesWithBuiltinSDF@sensor_msgs/msg/Image@gz.msgs.Image',
    #                '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
    #                '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
    #     output='screen'
    # )

    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=['/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image'],
    #     output='screen'
    # )

    # bridge = Node(
    #     package='ros_gz_image',
    #     executable='image_bridge',
    #     arguments=['camera', 'depth_camera', 'rgbd_camera/image', 'rgbd_camera/depth_image'],
    #     output='screen'
    # )


    moveit_config = (
        MoveItConfigsBuilder("robot", package_name="tm12s_moveit_config")
        .robot_description(file_path="config/robot.urdf.xacro")
        .robot_description_semantic(file_path="config/robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description= True, publish_robot_description_semantic=True, publish_planning_scene=True
        )
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .to_moveit_configs()
    )

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
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    ompl_yaml_path = os.path.join(
        get_package_share_directory("tm12s_moveit_config"),
        "config",
        "moveit_planners_ompl.yaml",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            ompl_yaml_path,
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    

    

    return LaunchDescription([
        # DeclareLaunchArgument('use_sim_time', default_value=['true'],
        #                     description='use sim time from /clock'),    
        # DeclareLaunchArgument('rviz', default_value='true',
        #                       description='Open RViz.'),
        #gz_sim,
        rviz_node,move_group_node
        # rviz_node,
        # dbridge,
        #rviz
    ])
