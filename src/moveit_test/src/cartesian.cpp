#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>



using namespace std::chrono_literals;
using MoveGroup = moveit_msgs::action::MoveGroup;
using GoalHandle = rclcpp_action::ClientGoalHandle<MoveGroup>;

class CartesianPoseClient : public rclcpp::Node
{
public:
  CartesianPoseClient()
  : Node("cartesian_pose_client")
  {
    if (!this->has_parameter("use_sim_time")) {
      this->declare_parameter("use_sim_time", true);
    }

    client_ = rclcpp_action::create_client<MoveGroup>(this, "/move_action");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void send_cartesian_goal() {
    RCLCPP_INFO(get_logger(), "Waiting for move_group action server...");
    if (!client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(get_logger(), "move_group action server not available.");
      return;
    }

    rclcpp::sleep_for(500ms);

    // Initial target pose (starting point)
    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header.frame_id = "link_0";
    start_pose.pose.position.x = -0.204;
    start_pose.pose.position.y = -0.513;
    start_pose.pose.position.z =  0.386;
    tf2::Quaternion q(1.0, 0.0, 0.0, 0.0);
    q.normalize();
    start_pose.pose.orientation.x = q.x();
    start_pose.pose.orientation.y = q.y();
    start_pose.pose.orientation.z = q.z();
    start_pose.pose.orientation.w = q.w();

    // Store for error checking later
    target_pose_ = start_pose;

    // Movement Sequence (displacement from starting position in meters)
    double rad = 0.01; // move in a sphere of radius 300mm
    std::vector<std::vector<double>> displacements;

    displacements.push_back({ rad, 0.0, 0.0 });  // +X
    displacements.push_back({-rad, 0.0, 0.0 });  // -X
    displacements.push_back({ 0.0,  rad, 0.0 }); // +Y
    displacements.push_back({ 0.0, -rad, 0.0 }); // -Y
    displacements.push_back({ 0.0, 0.0,  rad }); // +Z
    displacements.push_back({ 0.0, 0.0, -rad }); // -Z

    // Diagonal directions (8 points: all combinations of ±X, ±Y, ±Z)
    std::vector<int> signs = {-1, 1};
    for (int sx : signs) {
      for (int sy : signs) {
        for (int sz : signs) {
          // Normalize direction vector to have length `radius`
          double inv_len = 1.0 / std::sqrt(3.0);
          displacements.push_back({sx * rad * inv_len, sy * rad * inv_len, sz * rad * inv_len});
        }
      }
    }


    for (const auto& disp : displacements) {
      geometry_msgs::msg::PoseStamped offset_pose = start_pose;
      offset_pose.pose.position.x += disp[0];
      offset_pose.pose.position.y += disp[1];
      offset_pose.pose.position.z += disp[2];

      RCLCPP_INFO(this->get_logger(), "Sending offset pose [%.10f, %.10f, %.10f]",
                  offset_pose.pose.position.x,
                  offset_pose.pose.position.y,
                  offset_pose.pose.position.z);

      send_pose_goal(offset_pose);
      rclcpp::sleep_for(2s);  // Wait for execution and TF update

      send_pose_goal(start_pose);  // Return to start
      rclcpp::sleep_for(2s);
    }

    // rclcpp::shutdown();
  }


  void send_pose_goal(const geometry_msgs::msg::PoseStamped &pose) {
    auto goal_msg = MoveGroup::Goal();
    goal_msg.request.group_name = "tm12S_planninggroup";
    goal_msg.request.max_velocity_scaling_factor = 0.1;
    goal_msg.request.max_acceleration_scaling_factor = 0.1;
    goal_msg.request.pipeline_id = "pilz_industrial_motion_planner";
    goal_msg.request.planner_id = "LIN";
    goal_msg.request.num_planning_attempts = 10;
    goal_msg.request.allowed_planning_time = 10.0;
    goal_msg.request.start_state.is_diff = true;
  
    moveit_msgs::msg::Constraints constraints;
    moveit_msgs::msg::PositionConstraint pos_constraint;
    moveit_msgs::msg::OrientationConstraint ori_constraint;
  
    pos_constraint.header.frame_id = "link_0";
    pos_constraint.link_name = "flange";
    pos_constraint.constraint_region.primitives.resize(1);
    pos_constraint.constraint_region.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    pos_constraint.constraint_region.primitives[0].dimensions = {0.001, 0.001, 0.001};
    pos_constraint.constraint_region.primitive_poses.push_back(pose.pose);
    pos_constraint.weight = 1.0;
  
    ori_constraint.header.frame_id = "link_0";
    ori_constraint.link_name = "flange";
    ori_constraint.orientation = pose.pose.orientation;
    ori_constraint.absolute_x_axis_tolerance = 0.01;
    ori_constraint.absolute_y_axis_tolerance = 0.01;
    ori_constraint.absolute_z_axis_tolerance = 0.01;
    ori_constraint.weight = 1.0;
  
    constraints.position_constraints.push_back(pos_constraint);
    constraints.orientation_constraints.push_back(ori_constraint);
    goal_msg.request.goal_constraints.push_back(constraints);

    rclcpp::Clock sim_clock(RCL_ROS_TIME);
    while (sim_clock.now().nanoseconds() == 0) {
      RCLCPP_WARN(this->get_logger(), "Waiting for simulation time (/clock) to start...");
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    // rclcpp::Clock clock(RCL_ROS_TIME);
    auto t_start = this->now();

    // Send goal and wait for response
    auto future_goal_handle = client_->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send goal");
      return;
    }
  
    auto goal_handle = future_goal_handle.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
      return;
    }
  
    auto result_future = client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get result");
      return;
    }
  
    auto result = result_future.get();

    // ⏱️ End timing
    auto t_end = this->now();
    double duration_sec = (t_end - t_start).seconds();

    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(this->get_logger(), "Pose goal failed with status: %d", static_cast<int>(result.code));
      return;
    }
  
    RCLCPP_INFO(this->get_logger(), "Pose goal succeeded!");
    RCLCPP_INFO(this->get_logger(), "Motion duration: %.3f seconds", duration_sec);
  
    // Wait for TF to update
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  
    try {
      std::string from_frame = "link_0";
      std::string to_frame = "flange";
      if (!tf_buffer_->canTransform(from_frame, to_frame, tf2::TimePointZero, tf2::durationFromSec(2.0))) {
        RCLCPP_WARN(this->get_logger(), "Transform %s -> %s not available.", from_frame.c_str(), to_frame.c_str());
        return;
      }
  
      auto transform = tf_buffer_->lookupTransform(from_frame, to_frame, tf2::TimePointZero);
  
      RCLCPP_INFO(this->get_logger(), "Actual EE Position: [%.10f, %.10f, %.10f]",
                  transform.transform.translation.x,
                  transform.transform.translation.y,
                  transform.transform.translation.z);
      RCLCPP_INFO(this->get_logger(), "Actual EE Orientation (Quaternion): [x=%.10f, y=%.10f, z=%.10f, w=%.10f]",
                  transform.transform.rotation.x,
                  transform.transform.rotation.y,
                  transform.transform.rotation.z,
                  transform.transform.rotation.w);
  
      RCLCPP_INFO(this->get_logger(), "Error EE Position: [%.10e, %.10e, %.10e]",
                  std::abs(pose.pose.position.x - transform.transform.translation.x),
                  std::abs(pose.pose.position.y - transform.transform.translation.y),
                  std::abs(pose.pose.position.z - transform.transform.translation.z));
      RCLCPP_INFO(this->get_logger(), "Error EE Orientation (Quaternion): [x=%.10e, y=%.10e, z=%.10e, w=%.10e]",
                  std::abs(pose.pose.orientation.x - transform.transform.rotation.x),
                  std::abs(pose.pose.orientation.y - transform.transform.rotation.y),
                  std::abs(pose.pose.orientation.z - transform.transform.rotation.z),
                  std::abs(pose.pose.orientation.w - transform.transform.rotation.w));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    }
  }

private:
  rclcpp_action::Client<MoveGroup>::SharedPtr client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_       ;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;  
  geometry_msgs::msg::PoseStamped target_pose_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);                                 
  auto node = std::make_shared<CartesianPoseClient>();
  node->send_cartesian_goal();
  rclcpp::shutdown();
  return 0;
}