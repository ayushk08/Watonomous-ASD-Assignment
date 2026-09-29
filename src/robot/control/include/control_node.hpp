// ControlNode caches /path and odometry, runs ControlCore at 10 Hz, and publishes /cmd_vel.
#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    // Creates path/odometry inputs, velocity output, tuned parameters, and the 10 Hz timer.
    ControlNode();

  private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoop();

    // Pure Pursuit math lives in ControlCore; this node handles ROS I/O and timer scheduling.
    robot::ControlCore control_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // The callbacks refresh these snapshots; controlLoop uses both to compute each command.
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    // A 1 m lookahead smooths steering while remaining responsive on this small simulated robot.
    double lookahead_distance_{1.0};
    // Stop within 0.1 m of the final waypoint, about one 0.1 m map cell.
    double goal_tolerance_{0.1};
    // Default forward speed from the template; angular speed bends this motion toward the path.
    double linear_speed_{0.5};
    // Caps turns at 1.5 rad/s so a large curvature does not request an extreme rotation rate.
    double max_angular_speed_{1.5};
};

#endif
