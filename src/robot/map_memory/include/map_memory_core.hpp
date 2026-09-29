// MapMemoryCore owns the persistent global grid and fuses each accepted local costmap into it.
#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <string>

namespace robot
{

class MapMemoryCore {
  public:
    // Retains a logger for warnings emitted during map fusion.
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Transforms a local costmap by the robot pose, then overwrites known global cells.
    bool integrateCostmap(
        const nav_msgs::msg::OccupancyGrid& costmap,
        double robot_x,
        double robot_y,
        double robot_yaw,
        const std::string& global_frame);
    // Exposes the accumulated map for the node's periodic /map publisher.
    const nav_msgs::msg::OccupancyGrid& globalMap() const;

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid global_map_;
    bool has_global_map_{false};
};

}  

#endif  
