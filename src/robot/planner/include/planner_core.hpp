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
    explicit PlannerCore(const rclcpp::Logger& logger);

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
