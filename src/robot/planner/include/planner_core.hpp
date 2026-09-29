// PlannerCore searches the occupancy grid with A* and returns a route in map-frame meters.
#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <vector>

namespace robot
{

class PlannerCore {
  public:
    // Retains a logger for invalid map and no-route diagnostics.
    explicit PlannerCore(const rclcpp::Logger& logger);

    // Finds a minimum-cost route under the occupancy and unknown-cell costs defined in the core.
    bool findPath(
        const nav_msgs::msg::OccupancyGrid& map,
        const geometry_msgs::msg::Point& start,
        const geometry_msgs::msg::Point& goal,
        std::vector<geometry_msgs::msg::Point>& path) const;

  private:
    rclcpp::Logger logger_;
};

}  

#endif  
