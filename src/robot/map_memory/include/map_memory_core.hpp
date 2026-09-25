#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <string>

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    bool integrateCostmap(
        const nav_msgs::msg::OccupancyGrid& costmap,
        double robot_x,
        double robot_y,
        double robot_yaw,
        const std::string& global_frame);
    const nav_msgs::msg::OccupancyGrid& globalMap() const;

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid global_map_;
    bool has_global_map_{false};
};

}  

#endif  
