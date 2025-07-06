export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:~/Thesis/simulation/sim/install/moma_visualisation/share
colcon build --packages-select moma_visualisation
source ./install/setup.bash
ros2 launch moma_visualisation launch_sim.launch.py
