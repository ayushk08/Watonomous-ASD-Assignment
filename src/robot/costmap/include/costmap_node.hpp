// ROS wrapper: receives lidar scans, delegates grid construction to CostmapCore, and publishes them.
#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
 
#include "costmap_core.hpp"

#include <string>
#include <cstdint>
 
class CostmapNode : public rclcpp::Node {
  public:
    // Initializes /lidar input and /costmap output around a CostmapCore instance.
    CostmapNode();
    
    // Place callback function here
    // Publishes the warm-up string message on its separate 500 ms timer.
    void publishMessage();
    // Converts the received scan into a local costmap, then publishes that grid.
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  private:
    // Serializes CostmapCore's row/column grid into a ROS OccupancyGrid message.
    void publishCostmap(const std::string& frame_id);

    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
};
 
#endif 
