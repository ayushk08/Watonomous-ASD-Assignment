// ROS wrapper: caches latest map/odometry, fuses on a timer, and publishes the persistent /map.
#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

#include <string>

class MapMemoryNode : public rclcpp::Node {
  public:
    // Creates input subscriptions, the /map publisher, and the periodic fusion timer.
    MapMemoryNode();
    // Saves the latest local costmap for the next eligible timed fusion.
    void aggregateMaps(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);
    // Saves the robot pose and measures distance from the last successful map fusion.
    void trackRobotMovement(const nav_msgs::msg::Odometry::SharedPtr odom);

  private:
    // Timer entry point: conditionally integrates the local map, then publishes the global map.
    void updateMap();
    // Performs the initial fusion immediately, then fuses after each 1.5 m of travel.
    void tryAggregateLatestCostmap();

    // Core owns the persistent grid; this node owns ROS transport and update timing.
    robot::MapMemoryCore map_memory_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_filtered_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_pub_;
    rclcpp::TimerBase::SharedPtr map_update_timer_;
    // Only the newest local observation is needed for the next scheduled fusion.
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;

    bool has_odom_pose_{false};
    bool has_integrated_costmap_{false};
    bool has_last_update_position_{false};
    // Defaults to odom until the first odometry message provides its actual frame name.
    std::string odom_frame_{"odom"};
    double current_x_{0.0};
    double current_y_{0.0};
    double current_yaw_{0.0};
    double last_update_x_{0.0};
    double last_update_y_{0.0};
    // Euclidean distance from the last successful fusion reference pose.
    double distance_since_last_update_{0.0};

};

#endif 
