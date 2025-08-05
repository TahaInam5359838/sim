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

#include <fstream>

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

  moveit_msgs::action::MoveGroup::Goal create_goal(const geometry_msgs::msg::PoseStamped &pose) {
    auto goal_msg = MoveGroup::Goal();
    goal_msg.request.group_name = "tm12S_planninggroup";
    goal_msg.request.max_velocity_scaling_factor = 1.0;
    goal_msg.request.max_acceleration_scaling_factor = 1.0;
    goal_msg.request.pipeline_id = "pilz_industrial_motion_planner";
    goal_msg.request.planner_id = "LIN";
    goal_msg.request.num_planning_attempts = 10;
    goal_msg.request.allowed_planning_time = 10.0;
    goal_msg.request.start_state.is_diff = true;

    moveit_msgs::msg::Constraints           constraints     ;
    moveit_msgs::msg::PositionConstraint    pos_constraint  ;
    moveit_msgs::msg::OrientationConstraint ori_constraint  ;
  
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

    return goal_msg;
  }

  geometry_msgs::msg::PoseStamped get_start_pose() {
    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header.frame_id = "link_0";
    start_pose.pose.position.x = -0.204;
    start_pose.pose.position.y = -0.513;
    start_pose.pose.position.z =  0.586;
    // tf2::Quaternion q(0.707, 0, 0, 0.707);
    // start_pose.pose.position.z =  0.386;
    tf2::Quaternion q(1.0, 0.0, 0.0, 0.0);

    q.normalize();
    start_pose.pose.orientation.x = q.x();
    start_pose.pose.orientation.y = q.y();
    start_pose.pose.orientation.z = q.z();
    start_pose.pose.orientation.w = q.w();

    return start_pose;
  }

  // Generate a set of displacements that create a sphere (radius in m)
  std::vector<std::vector<double>> get_spherical_path(double radius) {
    std::vector<std::vector<double>> displacements;

    displacements.push_back({ radius, 0.0, 0.0 });  // +X
    displacements.push_back({-radius, 0.0, 0.0 });  // -X
    displacements.push_back({ 0.0,  radius, 0.0 }); // +Y
    displacements.push_back({ 0.0, -radius, 0.0 }); // -Y
    displacements.push_back({ 0.0, 0.0,  radius }); // +Z
    displacements.push_back({ 0.0, 0.0, -radius }); // -Z

    // Diagonal directions (8 points: all combinations of ±X, ±Y, ±Z)
    std::vector<int> signs = {-1, 1};
    for (int sx : signs) {
      for (int sy : signs) {
        for (int sz : signs) {
          // Normalize direction vector to have length `radius`
          double inv_len = 1.0 / std::sqrt(3.0);
          displacements.push_back({sx * radius * inv_len, sy * radius * inv_len, sz * radius * inv_len});
        }
      }
    }

    return displacements;
  }

  // Generate a set of displacements that create a 'random' path
  std::vector<std::vector<double>> get_random_path() {
    std::vector<std::vector<double>> displacements;

    displacements.push_back({ 0.0, 0.0, 0.0 });
    displacements.push_back({ 0.0, -0.4, 0.0 });
    displacements.push_back({ -0.4, -0.5, 0.0 });
    displacements.push_back({ 0.0, -0.4, 0.0 });
    displacements.push_back({ 0.4, -0.5, 0.0 });
    displacements.push_back({ 0.0, 0.0, -0.3 });
    return displacements;
  }

  std::vector<std::vector<double>> get_random_path2() {
    std::vector<std::vector<double>> displacements;

    displacements.push_back({ 0.0, 0.0, 0.0 });
    displacements.push_back({ 0.0, 0.0, -0.2 });
    displacements.push_back({ 0.0, -0.5, 0.0 });
    displacements.push_back({ 0.0, 0.0, 0.0 });

    return displacements;
  }

void monitor_velocity_during_motion(const std::string &from_frame, const std::string &to_frame, std::ofstream& log_file)
{
  rclcpp::Duration sample_interval = rclcpp::Duration::from_seconds(0.1);  // 10 Hz sampling
  rclcpp::Time current_time = this->now();

  // Initialize: get first transform at current_time
  geometry_msgs::msg::TransformStamped prev_tf;
  // bool got_prev_tf = false;
  rclcpp::Time prev_time;

  while (true) {
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      if (motion_done_flag_) break;
    }

    rclcpp::Time now = this->now();

    // We want to compare transform at 'now' and 'prev_time = now - sample_interval'
    rclcpp::Time past_time = now - sample_interval;

    // Check if both transforms are available
    if (!tf_buffer_->canTransform(from_frame, to_frame, past_time, tf2::durationFromSec(0.1)) ||
        !tf_buffer_->canTransform(from_frame, to_frame, now, tf2::durationFromSec(0.1))) {
      rclcpp::sleep_for(10ms);
      continue;
    }

    geometry_msgs::msg::TransformStamped current_tf;
    geometry_msgs::msg::TransformStamped past_tf;
    try {
      current_tf = tf_buffer_->lookupTransform(from_frame, to_frame, now);
      past_tf = tf_buffer_->lookupTransform(from_frame, to_frame, past_time);
    } catch (const tf2::TransformException & ex) {
      rclcpp::sleep_for(10ms);
      continue;
    }

    double dt = (now - past_time).seconds();
    if (dt <= 0.0) {
      rclcpp::sleep_for(100ms);
      continue;
    }

    // Compute position differences
    double dx = current_tf.transform.translation.x - past_tf.transform.translation.x;
    double dy = current_tf.transform.translation.y - past_tf.transform.translation.y;
    double dz = current_tf.transform.translation.z - past_tf.transform.translation.z;

    // Compute velocity components
    double vx = dx / dt;
    double vy = dy / dt;
    double vz = dz / dt;
    double v_mag = std::sqrt(vx*vx + vy*vy + vz*vz);

    // Log only if velocity magnitude is significant
    if (v_mag > 1e-6) {
      log_file << "velocity," << now.seconds() << "," << vx << "," << vy << "," << vz << "," << v_mag << "\n";
    }

    rclcpp::sleep_for(100ms);
  }
}




  // Send the goal motions
  void send_cartesian_goal() {
    RCLCPP_INFO(get_logger(), "Waiting for move_group action server...");
    if (!client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(get_logger(), "move_group action server not available.");
      return;
    }

    rclcpp::sleep_for(500ms);

    // Initial target pose (starting point)
    geometry_msgs::msg::PoseStamped start_pose = get_start_pose();
    target_pose_ = start_pose; // Store for error checking later

    // Generate the target displacements
    double rad = 200; // mm
    std::vector<std::vector<double>> displacements = get_spherical_path(rad /= 1000);
    // std::vector<std::vector<double>> displacements = get_random_path();

    csv_file_.open("results_200mm_0.5ms2_3.csv");
    csv_file_ << "desired_x,desired_y,desired_z,"
              << "actual_x,actual_y,actual_z,"
              << "error_x,error_y,error_z,time\n";  

    int i = 0;
    for (const auto& disp : displacements) {
      RCLCPP_INFO(this->get_logger(), "Sending pose %.10d", i);
      
      geometry_msgs::msg::PoseStamped offset_pose = start_pose;
      offset_pose.pose.position.x += disp[0];
      offset_pose.pose.position.y += disp[1];
      offset_pose.pose.position.z += disp[2];

      RCLCPP_INFO(this->get_logger(), "Sending offset pose [%.10f, %.10f, %.10f]",
                  offset_pose.pose.position.x,
                  offset_pose.pose.position.y,
                  offset_pose.pose.position.z);

      // Add coords to csv

      send_pose_goal(offset_pose, csv_file_);
      rclcpp::sleep_for(2s);  // Wait for execution and TF update

      RCLCPP_INFO(this->get_logger(), "Going home");

      send_pose_goal(start_pose, csv_file_);  // Return to start
      rclcpp::sleep_for(2s);

      i++;
    }

    // RCLCPP_INFO(this->get_logger(), "Changing orientation on next movement from now");

    // tf2::Quaternion q(1, 0, 0, 0);
    // q.normalize();
    // start_pose.pose.orientation.x = q.x();
    // start_pose.pose.orientation.y = q.y();
    // start_pose.pose.orientation.z = q.z();
    // start_pose.pose.orientation.w = q.w();

    // displacements = get_random_path2();

    // for (const auto& disp : displacements) {
    //   RCLCPP_INFO(this->get_logger(), "Sending pose %.10d", i);
      
    //   geometry_msgs::msg::PoseStamped offset_pose = start_pose;
    //   offset_pose.pose.position.x += disp[0];
    //   offset_pose.pose.position.y += disp[1];
    //   offset_pose.pose.position.z += disp[2];

    //   RCLCPP_INFO(this->get_logger(), "Sending offset pose [%.10f, %.10f, %.10f]",
    //               offset_pose.pose.position.x,
    //               offset_pose.pose.position.y,
    //               offset_pose.pose.position.z);

      // Add coords to csv

      // send_pose_goal(offset_pose, csv_file_);
      // rclcpp::sleep_for(2s);  // Wait for execution and TF update

      // RCLCPP_INFO(this->get_logger(), "Going home");

      // send_pose_goal(start_pose, csv_file_);  // Return to start
      // rclcpp::sleep_for(2s);

      // i++;
    // }


    csv_file_.close();
  }

  // Actually send the goal pose to the robot
  void send_pose_goal(const geometry_msgs::msg::PoseStamped &pose, std::ofstream& log_file) {
    moveit_msgs::action::MoveGroup::Goal goal_msg = create_goal(pose);  

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

    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      motion_done_flag_ = false;
    }
    std::thread velocity_logger_thread(&CartesianPoseClient::monitor_velocity_during_motion, this,
                                   "link_0", "flange", std::ref(log_file));
  
    // Wait for motion to complete
    auto result_future = client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get result");

      // Mark motion done
      {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        motion_done_flag_ = true;
      }
      velocity_logger_thread.join();
      return;
    }
  
    auto result = result_future.get();
    
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      motion_done_flag_ = true;
    }

    velocity_logger_thread.join();

    auto t_end = this->now();
    double duration_sec = (t_end - t_start).seconds();

    

    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(this->get_logger(), "Pose goal failed with status: %d", static_cast<int>(result.code));
      return;
    }

    csv_file_ << "timestamp,vx,vy,vz,vmag\n";
    for (const auto& entry : velocity_log_) {
      double t, vx, vy, vz, vmag;
      std::tie(t, vx, vy, vz, vmag) = entry;
      csv_file_ << t << "," << vx << "," << vy << "," << vz << "," << vmag << "\n";
    }
    velocity_log_.clear();  // Clear for next motion
  
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

      const auto& actual = transform.transform.translation;
      const auto& desired = pose.pose.position;

      double err_x = std::abs(desired.x - actual.x);
      double err_y = std::abs(desired.y - actual.y);
      double err_z = std::abs(desired.z - actual.z);

      log_file  << desired.x << "," << desired.y << "," << desired.z << ","
                << actual.x  << "," << actual.y  << "," << actual.z  << ","
                << err_x     << "," << err_y     << "," << err_z     << "," << duration_sec << "\n";

  
      RCLCPP_INFO(this->get_logger(), "Actual EE Position: [%.10f, %.10f, %.10f]",
                  transform.transform.translation.x,
                  transform.transform.translation.y,
                  transform.transform.translation.z);
      RCLCPP_INFO(this->get_logger(), "Error EE Position: [%.10e, %.10e, %.10e]",
                  std::abs(pose.pose.position.x - transform.transform.translation.x),
                  std::abs(pose.pose.position.y - transform.transform.translation.y),
                  std::abs(pose.pose.position.z - transform.transform.translation.z));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    }
  }

private:
  rclcpp_action::Client<MoveGroup>::SharedPtr client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_       ;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;  
  geometry_msgs::msg::PoseStamped target_pose_;
  std::ofstream csv_file_;

  std::vector<std::tuple<double, double, double, double, double>> velocity_log_; 
  // (timestamp, vx, vy, vz, vmag)

  bool motion_done_flag_ = false;
  std::mutex motion_mutex_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);                                 
  auto node = std::make_shared<CartesianPoseClient>();
  node->send_cartesian_goal();
  rclcpp::shutdown();
  return 0;
}