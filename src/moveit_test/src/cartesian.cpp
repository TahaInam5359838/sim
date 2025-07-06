#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("moveit_cartesian_goal_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Use planning group name from SRDF
  static const std::string PLANNING_GROUP = "tm12S_planninggroup";  // change this!

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  move_group.setPlanningTime(5.0);
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  move_group.setPoseReferenceFrame("base_link");
  move_group.setEndEffectorLink("link_6");

  geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();

  // Get current position
  RCLCPP_INFO(node->get_logger(), "Current pose: [x=%.3f, y=%.3f, z=%.3f]",
              current_pose.pose.position.x,
              current_pose.pose.position.y,
              current_pose.pose.position.z);

  // Print orientation
  RCLCPP_INFO(node->get_logger(), "Orientation: [x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
              current_pose.pose.orientation.x,
              current_pose.pose.orientation.y,
              current_pose.pose.orientation.z,
              current_pose.pose.orientation.w);


  // Set target pose
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.241;
  target_pose.position.y = 0.266;
  target_pose.position.z = 1.742;
  target_pose.orientation.x = 0.5;
  target_pose.orientation.y = 0.5;
  target_pose.orientation.z = 0.5;
  target_pose.orientation.w = 0.5;

  move_group.setPoseTarget(target_pose);

  // Plan and execute
  auto success = static_cast<bool>(move_group.move());

  if (success) {
    RCLCPP_INFO(node->get_logger(), "Motion succeeded");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Motion failed");
  }

  // After moving
  current_pose = move_group.getCurrentPose();

  // Get current position
  RCLCPP_INFO(node->get_logger(), "Current pose: [x=%.3f, y=%.3f, z=%.3f]",
              current_pose.pose.position.x,
              current_pose.pose.position.y,
              current_pose.pose.position.z);

  // Print orientation
  RCLCPP_INFO(node->get_logger(), "Orientation: [x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
              current_pose.pose.orientation.x,
              current_pose.pose.orientation.y,
              current_pose.pose.orientation.z,
              current_pose.pose.orientation.w);

  rclcpp::shutdown();
  return 0;
}
