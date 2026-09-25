#include "control_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(const nav_msgs::msg::Path& path,
    const geometry_msgs::msg::Point& robot_position,
    const double lookahead_distance) const {
  if (path.poses.empty() || lookahead_distance <= 0.0) {
    return std::nullopt;
  }

  // Start looking forward from the path point closest to the robot.
  std::size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const double distance = computeDistance(robot_position, path.poses[i].pose.position);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = i;
    }
  }

  // Choose the first later waypoint at least one lookahead distance away.
  for (std::size_t i = closest_index; i < path.poses.size(); ++i) {
    if (computeDistance(robot_position, path.poses[i].pose.position) >= lookahead_distance) {
      return path.poses[i];
    }
  }

  // Near the end of a short path, steer toward its final waypoint.
  return path.poses.back();
}

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

double ControlCore::computeDistance(
    const geometry_msgs::msg::Point& a,
    const geometry_msgs::msg::Point& b) const {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& quaternion) const {
  return std::atan2(
      2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
      1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

}  
