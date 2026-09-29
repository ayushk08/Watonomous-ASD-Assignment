// ControlCore contains the Pure Pursuit math; ControlNode supplies current ROS path/odometry and publishes its Twist.
#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include <optional>

namespace robot
{

class ControlCore {
  public:
    // Keeps the node logger for the controller core's diagnostic output.
    ControlCore(const rclcpp::Logger& logger);

    // Selects a point on the path at or beyond the requested lookahead distance.
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
        const nav_msgs::msg::Path& path,
        const geometry_msgs::msg::Point& robot_position,
        double lookahead_distance) const;

    // Calculates linear and angular velocity to steer toward the lookahead point.
    geometry_msgs::msg::Twist computeVelocity(
        const geometry_msgs::msg::PoseStamped& target,
        const nav_msgs::msg::Odometry& odometry,
        double linear_speed,
        double max_angular_speed) const;

    // Returns straight-line distance between two points in the same frame.
    double computeDistance(
        const geometry_msgs::msg::Point& a,
        const geometry_msgs::msg::Point& b) const;

    // Extracts yaw, the robot's rotation around the vertical axis, from its quaternion.
    double extractYaw(const geometry_msgs::msg::Quaternion& quaternion) const;
  
  private:
    rclcpp::Logger logger_;
};

} 

#endif 
