// ROS orchestration for Pure Pursuit: cache inputs, run a 10 Hz control loop, and publish /cmd_vel.
#include "control_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>

// Creates input/output topics, loads tunable controller values, and starts a 100 ms control timer.
ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Keep lookahead positive so target selection always has a meaningful distance.
  lookahead_distance_ = std::max(
      this->declare_parameter<double>("lookahead_distance", 1.0), 0.01);
  // These defaults follow the wiki template; speed values are bounded to nonnegative limits.
  goal_tolerance_ = std::max(
      this->declare_parameter<double>("goal_tolerance", 0.1), 0.0);
  linear_speed_ = std::max(
      this->declare_parameter<double>("linear_speed", 0.5), 0.0);
  max_angular_speed_ = std::max(
      this->declare_parameter<double>("max_angular_speed", 1.5), 0.01);

  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

// Stores the latest path; an empty path from the planner causes the next loop to command a stop.
void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = msg;
}

// Stores current position and orientation used to compute the next steering command.
void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_odom_ = msg;
}

// Stops on missing/invalid inputs or at the goal; otherwise selects a target and publishes Pure Pursuit velocity.
void ControlNode::controlLoop() {
  const auto publishStop = [this]() {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
  };

  if (!current_path_ || !current_odom_ || current_path_->poses.empty()) {
    publishStop();
    return;
  }

  if (!current_path_->header.frame_id.empty() &&
      !current_odom_->header.frame_id.empty() &&
      current_path_->header.frame_id != current_odom_->header.frame_id) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Cannot follow path: path and odometry frames differ");
    publishStop();
    return;
  }

  const auto& robot_position = current_odom_->pose.pose.position;
  const auto& goal_position = current_path_->poses.back().pose.position;
  if (control_.computeDistance(robot_position, goal_position) <= goal_tolerance_) {
    publishStop();
    return;
  }

  const auto target = control_.findLookaheadPoint(
      *current_path_, robot_position, lookahead_distance_);
  if (!target) {
    publishStop();
    return;
  }

  const auto command = control_.computeVelocity(
      *target, *current_odom_, linear_speed_, max_angular_speed_);
  cmd_vel_pub_->publish(command);
}

// Starts ROS and processes path/odometry callbacks alongside the 10 Hz control timer.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
