#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

#include <string>

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();
    void aggregateMaps(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);
    void trackRobotMovement(const nav_msgs::msg::Odometry::SharedPtr odom);

  private:
    void updateMap();
    void tryAggregateLatestCostmap();

    robot::MapMemoryCore map_memory_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_filtered_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_pub_;
    rclcpp::TimerBase::SharedPtr map_update_timer_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;

    bool has_odom_pose_{false};
    bool has_integrated_costmap_{false};
    bool has_last_update_position_{false};
    std::string odom_frame_{"odom"};
    double current_x_{0.0};
    double current_y_{0.0};
    double current_yaw_{0.0};
    double last_update_x_{0.0};
    double last_update_y_{0.0};
    double distance_since_last_update_{0.0};

};

#endif 
