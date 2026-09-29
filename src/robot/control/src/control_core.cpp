// Pure Pursuit calculations separated from ROS subscriptions/timers so the node can orchestrate them.
#include "control_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace robot
{

// Retains the logger provided by ControlNode for any controller diagnostics.
ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

// Finds the path waypoint nearest the robot, then selects a later point at the lookahead distance.
std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(const nav_msgs::msg::Path& path,
    const geometry_msgs::msg::Point& robot_position,
    const double lookahead_distance) const {
  if (path.poses.empty() || lookahead_distance <= 0.0) {
    return std::nullopt;
  }

  // Starting near the robot avoids steering toward old waypoints it has already passed.
  std::size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const double distance = computeDistance(robot_position, path.poses[i].pose.position);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = i;
    }
  }

  // The path samples are dense, so the first waypoint beyond 1 m approximates an exact lookahead point.
  for (std::size_t i = closest_index; i < path.poses.size(); ++i) {
    if (computeDistance(robot_position, path.poses[i].pose.position) >= lookahead_distance) {
      return path.poses[i];
    }
  }

  // Near the end of a short path, use the goal waypoint even if it is closer than the lookahead distance.
  return path.poses.back();
}

// Converts the lookahead point into robot coordinates and applies Pure Pursuit curvature to Twist.
geometry_msgs::msg::Twist ControlCore::computeVelocity(
    const geometry_msgs::msg::PoseStamped& target,
    const nav_msgs::msg::Odometry& odometry,
    const double linear_speed,
    const double max_angular_speed) const {
  geometry_msgs::msg::Twist command;
  const auto& robot_position = odometry.pose.pose.position;
  const double dx = target.pose.position.x - robot_position.x;
  const double dy = target.pose.position.y - robot_position.y;
  const double distance = std::hypot(dx, dy);
  if (distance <= 1e-6) {
    return command;
  }

  const double robot_yaw = extractYaw(odometry.pose.pose.orientation);
  const double robot_frame_x = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  const double robot_frame_y = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
  const double heading_error = std::atan2(robot_frame_y, robot_frame_x);

  if (robot_frame_x < 0.0) {
    // Turn toward a target behind the robot before driving forward.
    command.angular.z = std::clamp(2.0 * heading_error, -max_angular_speed, max_angular_speed);
    return command;
  }

  // Pure Pursuit curvature is 2*y/L^2 in the robot's coordinate frame.
  const double curvature = 2.0 * robot_frame_y / (distance * distance);
  command.linear.x = linear_speed;
  command.angular.z = std::clamp(
      linear_speed * curvature, -max_angular_speed, max_angular_speed);
  return command;
}

// Measures planar distance; z is ignored because this controller drives on a 2D ground plane.
double ControlCore::computeDistance(
    const geometry_msgs::msg::Point& a,
    const geometry_msgs::msg::Point& b) const {
  return std::hypot(a.x - b.x, a.y - b.y);
}

// Converts quaternion orientation into planar yaw for the robot-frame target calculation.
double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& quaternion) const {
  return std::atan2(
      2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
      1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

}  
