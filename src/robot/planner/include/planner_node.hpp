// Planner ROS interface: caches map, goal, and odometry; publishes A* paths and replans on updates.
#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    // Creates map/goal/odometry subscriptions, a path publisher, and the replanning timer.
    PlannerNode();

  private:
    // Saves each new map and requests a replan while a goal is active.
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    // Starts a new active goal and resets the timeout clock.
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    // Keeps the latest robot pose so the next plan starts from its current location.
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    // Checks completion/timeout every 500 ms and runs A* when a replan is pending.
    void timerCallback();
    // Checks frame compatibility, calls PlannerCore, and publishes /path (or clears it on failure).
    void planAndPublish();

    // Core runs A*; the node owns ROS subscriptions, timer state, and path publication.
    robot::PlannerCore planner_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr planner_timer_;

    // Latest inputs are cached so a timer-triggered replan uses a consistent current snapshot.
    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    nav_msgs::msg::Odometry current_odom_;
    // Reused as the last goal/plan timestamp for the 30 s retry timeout.
    rclcpp::Time goal_received_at_{0, 0, RCL_ROS_TIME};
    bool map_received_{false};
    bool goal_received_{false};
    bool odom_received_{false};
    // The two-state behavior is represented by goal_received_ plus this pending-replan flag.
    bool needs_replan_{false};
};

#endif 
