# What to do
```
bash setup.bash
```
This is just a bash script that rebuilds the packages and launches the simulation and moveit
```
rm -rf build install log
colcon build
source ./install/setup.bash
ros2 launch moma_visualisation launch_sim.launch.py
```


# sim

bash setup.bash

ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/moma_visualisation/config/mapper_params_online_async.yaml use_sim_time:=true

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p use_sim_time:=true --ros-args -r /cmd_vel:=/diff_drive_base_controller/cmd_vel_unstamped

ros2 run rviz2 rviz2 -d ../../Depth_Camera_Simulation/d435_ws/src/depth_d435/rviz/ns_robot2.rviz --ros-args -p use_sim_time:=true

ros2 run moveit_test joints --ros-args -p use_sim_time:=true


rqt

ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller

ros2 launch moma_visualisation cam.launch.py 