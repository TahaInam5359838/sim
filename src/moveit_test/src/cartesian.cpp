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
  static const std::string PLANNING_GROUP = "tm12S_planninggroup";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  move_group.setPlanningTime(5.0);
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  move_group.setPoseReferenceFrame("base_link");
  move_group.setEndEffectorLink("link_6");

  RCLCPP_INFO(node->get_logger(), "Waiting for current state...");
  auto current_state = move_group.getCurrentState(5.0);  // Wait up to 5 seconds

  geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();

  // Get current position
  RCLCPP_INFO(node->get_logger(), "Current pose: [x=%.10f, y=%.10f, z=%.10f]",
              current_pose.pose.position.x,
              current_pose.pose.position.y,
              current_pose.pose.position.z);

  // Print orientation
  RCLCPP_INFO(node->get_logger(), "Orientation: [x=%.10f, y=%.10f, z=%.10f, w=%.10f]",
              current_pose.pose.orientation.x,
              current_pose.pose.orientation.y,
              current_pose.pose.orientation.z,
              current_pose.pose.orientation.w);


  // Set target pose
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.2;
  target_pose.position.y = -0.25;
  target_pose.position.z = 1.4;
  tf2::Quaternion q(0.5, 0.5, 0.0, 0.0);
  q.normalize();
  target_pose.orientation.x = q.x();
  target_pose.orientation.y = q.y();
  target_pose.orientation.z = q.z();
  target_pose.orientation.w = q.w();

  // Show the goal position
  RCLCPP_INFO(node->get_logger(), "Target pose: [x=%.10f, y=%.10f, z=%.10f]",
              target_pose.position.x,
              target_pose.position.y,
              target_pose.position.z);

  // Print goal orientation
  RCLCPP_INFO(node->get_logger(), "Orientation: [x=%.10f, y=%.10f, z=%.10f, w=%.10f]",
              target_pose.orientation.x,
              target_pose.orientation.y,
              target_pose.orientation.z,
              target_pose.orientation.w);

  move_group.setPoseTarget(target_pose);

  // Plan and execute
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target_pose);

  // Plan manually
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::core::MoveItErrorCode result = move_group.plan(my_plan);

  if (result == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Planning succeeded, executing...");
    move_group.execute(my_plan);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Planning failed: %s", error_code_to_string(result).c_str());
  }

  RCLCPP_INFO(node->get_logger(), "Waiting for current state...");
  auto current_state2 = move_group.getCurrentState(5.0);  // Wait up to 5 seconds 

  // After moving
  current_pose = move_group.getCurrentPose();

  // Get current position
  RCLCPP_INFO(node->get_logger(), "Current pose: [x=%.10f, y=%.10f, z=%.10f]",
              current_pose.pose.position.x,
              current_pose.pose.position.y,
              current_pose.pose.position.z);

  // Print orientation
  RCLCPP_INFO(node->get_logger(), "Orientation: [x=%.10f, y=%.10f, z=%.10f, w=%.10f]",
              current_pose.pose.orientation.x,
              current_pose.pose.orientation.y,
              current_pose.pose.orientation.z,
              current_pose.pose.orientation.w);


  // PRINT ERRORS
  // Euclidean Position Error
  RCLCPP_INFO(node->get_logger(), "Position Error: [x=%.10e, y=%.10e, z=%.10e]",
              abs(target_pose.position.x-current_pose.pose.position.x),
              abs(target_pose.position.y-current_pose.pose.position.y),
              abs(target_pose.position.z-current_pose.pose.position.z));\

  // RCLCPP_INFO(node->get_logger(), "Euclidean Error = %.10f",
  //             abs(target_pose.position.x-current_pose.pose.position.x),
  //             abs(target_pose.position.y-current_pose.pose.position.y),
  //             abs(target_pose.position.z-current_pose.pose.position.z));

  // Print orientation
  RCLCPP_INFO(node->get_logger(), "Orientation Error: [x=%.10e, y=%.10e, z=%.10e, w=%.10e]",
              abs(target_pose.orientation.x-current_pose.pose.orientation.x),
              abs(target_pose.orientation.y-current_pose.pose.orientation.y),
              abs(target_pose.orientation.z-current_pose.pose.orientation.z),
              abs(target_pose.orientation.w-current_pose.pose.orientation.w));

  // Position Error: [x=3.2504564100e-05, y=8.9988083888e-06, z=3.4684134614e-05]
  // Orientation Error: [x=2.3682862628e-04, y=2.3667580623e-04, z=2.6969148145e-04, w=1.7687130967e-04]

  // Position Error: [x=7.0211867632e-05, y=7.3023362948e-06, z=5.7065886001e-05]
  // Orientation Error: [x=2.1012801513e-05, y=2.0994512435e-05, z=4.3793828426e-05, w=1.5186986420e-04]

  rclcpp::shutdown();
  return 0;
}
