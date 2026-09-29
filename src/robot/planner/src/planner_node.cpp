// ROS orchestration for A*: data callbacks cache inputs, the timer checks state, and plans go to /path.
#include "planner_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <utility>
#include <vector>

// Creates the three inputs, /path output, and a 500 ms timer for goal checks and replanning.
PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  planner_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

// Stores the latest map and requests a fresh route because occupancy changes can invalidate a path.
void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  map_received_ = true;
  if (goal_received_) {
    needs_replan_ = true;
  }
}

// Stores a new goal and starts planning for it.
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  needs_replan_ = true;
  goal_received_at_ = this->now();
}

// Stores the robot's latest position and orientation.
void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_odom_ = *msg;
  odom_received_ = true;
}

// Returns to waiting-for-goal when close enough; otherwise requests a replan after 30 seconds.
void PlannerNode::timerCallback() {
  if (!map_received_ || !goal_received_ || !odom_received_) {
    return;
  }

  // 0.25 m is a practical arrival radius for a discretized 0.1 m map and noisy odometry.
  constexpr double kGoalTolerance = 0.25;
  // A 30 s window gives the robot time to make progress before retrying a stalled route.
  constexpr auto kReplanTimeout = std::chrono::seconds(30);
  const auto& position = current_odom_.pose.pose.position;
  const double distance_to_goal = std::hypot(
      goal_.point.x - position.x, goal_.point.y - position.y);

  // checking if robot has reached goal
  if (distance_to_goal <= kGoalTolerance) {
    RCLCPP_INFO(this->get_logger(), "Goal reached");
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = this->now();
    empty_path.header.frame_id = current_map_.header.frame_id;
    path_pub_->publish(empty_path);
    goal_received_ = false;
    needs_replan_ = false;
    return;
  }

  // how much time has passed since robot started moving
  if ((this->now() - goal_received_at_).seconds() >=
      std::chrono::duration<double>(kReplanTimeout).count()) {
    RCLCPP_WARN(this->get_logger(), "Planner timed out; replanning will be needed");
    goal_received_at_ = this->now();
    needs_replan_ = true;
  }

  if (needs_replan_) {
    planAndPublish();
  }
}

// Plans in the map frame; rejects frame mismatches rather than applying coordinates incorrectly.
void PlannerNode::planAndPublish() {
  const std::string& map_frame = current_map_.header.frame_id;
  if ((!goal_.header.frame_id.empty() && goal_.header.frame_id != map_frame) ||
      (!current_odom_.header.frame_id.empty() && current_odom_.header.frame_id != map_frame)) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Cannot plan: map, goal, and odometry must use the same frame (map frame: '%s', goal: '%s', odometry: '%s')",
        map_frame.c_str(), goal_.header.frame_id.c_str(), current_odom_.header.frame_id.c_str());
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = this->now();
    empty_path.header.frame_id = map_frame;
    path_pub_->publish(empty_path);
    needs_replan_ = false;
    return;
  }

  std::vector<geometry_msgs::msg::Point> points;
  if (!planner_.findPath(
          current_map_, current_odom_.pose.pose.position, goal_.point, points)) {
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = this->now();
    empty_path.header.frame_id = map_frame;
    path_pub_->publish(empty_path);
    needs_replan_ = false;
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = map_frame;
  path.poses.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = points[i];

    double yaw = 0.0;
    if (i + 1 < points.size()) {
      yaw = std::atan2(points[i + 1].y - points[i].y, points[i + 1].x - points[i].x);
    } else if (i > 0) {
      yaw = std::atan2(points[i].y - points[i - 1].y, points[i].x - points[i - 1].x);
    }
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);
    path.poses.push_back(std::move(pose));
  }

  // Each output pose uses the direction of the next segment as its planar orientation.
  path_pub_->publish(path);
  needs_replan_ = false;
  goal_received_at_ = this->now();
}

// Starts ROS, spins the planner so subscribers/timer execute, and shuts down on exit.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
