#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("moveit_joints_goal_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Use planning group name from SRDF
  static const std::string PLANNING_GROUP = "tm12S_planninggroup";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  move_group.setPlanningTime(5.0);
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  move_group.setPoseReferenceFrame("base_link");
  move_group.setEndEffectorLink("link_6");

  std::vector<double> current_pose = move_group.getCurrentJointValues();

  // Get current position
  RCLCPP_INFO(node->get_logger(), "Current pose: [J1=%.3f, J2=%.3f, J3=%.3f, J4=%.3f, J5=%.3f, J6=%.3f]",
              current_pose[0],
              current_pose[1],
              current_pose[2],
              current_pose[3],
              current_pose[4],
              current_pose[5]);

  std::map<std::string, double> target_joint_values;
  target_joint_values["joint_1"] = 1.0;
  target_joint_values["joint_2"] = 0.5;
  target_joint_values["joint_3"] = -1.2;
  // Add other joints as needed

  move_group.setJointValueTarget(target_joint_values);

  // Plan and move
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    move_group.move();  // Executes the motion
    RCLCPP_INFO(node->get_logger(), "Motion succeeded");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Motion failed");
  }

  // After moving
  current_pose = move_group.getCurrentJointValues();

  // Get current position
  RCLCPP_INFO(node->get_logger(), "Current pose: [J1=%.3f, J2=%.3f, J3=%.3f, J4=%.3f, J5=%.3f, J6=%.3f]",
              current_pose[0],
              current_pose[1],
              current_pose[2],
              current_pose[3],
              current_pose[4],
              current_pose[5]);

  rclcpp::shutdown();
  return 0;
}