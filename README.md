# sim

bash setup.bash

ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/moma_visualisation/config/mapper_params_online_async.yaml  use_sim_time:=true

ros2 launch nav2_bringup navigation_launch.py 

rqt

ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller

ros2 launch moma_visualisation cam.launch.py 

ros2 run rviz2 rviz2 -d ../../Depth_Camera_Simulation/d435_ws/src/depth_d435/rviz/ns_robot2.rviz 