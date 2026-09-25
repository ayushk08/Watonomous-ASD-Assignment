#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <cstdint>
#include <vector>

namespace robot
{

class CostmapCore {
  public:
    static constexpr std::size_t GRID_WIDTH = 400;
    static constexpr std::size_t GRID_HEIGHT = 400;
    static constexpr double GRID_RESOLUTION = 0.1;
    static constexpr double MAP_MIN_X = -20.0;
    static constexpr double MAP_MIN_Y = -20.0;
    static constexpr double INFLATION_RADIUS = 1.0;
    static constexpr int MAX_COST = 100;

    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void updateFromLaserScan(const sensor_msgs::msg::LaserScan& scan);
    const std::vector<std::vector<int8_t>>& occupancyGrid() const;

  private:
    void inflateObstacles();

    rclcpp::Logger logger_;
    // Rows are Y positions; columns are X positions. Values start at 0 (free).
    std::vector<std::vector<int8_t>> occupancy_grid_;
};

}  

#endif  
