// CostmapCore turns each laser scan into a local obstacle grid and inflates obstacles for clearance.
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
    // A 400x400 grid at 0.1 m resolution spans 40 m by 40 m around the lidar.
    static constexpr std::size_t GRID_WIDTH = 400;
    static constexpr std::size_t GRID_HEIGHT = 400;
    static constexpr double GRID_RESOLUTION = 0.1;
    // The grid's lower-left corner; combined with its dimensions this centers it on the lidar.
    static constexpr double MAP_MIN_X = -20.0;
    static constexpr double MAP_MIN_Y = -20.0;
    // Inflate each detected obstacle within 1 m to leave a safety margin for the robot.
    static constexpr double INFLATION_RADIUS = 1.0;
    static constexpr int MAX_COST = 100;

    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Clears the previous local grid, marks scan endpoints as obstacles, then inflates them.
    void updateFromLaserScan(const sensor_msgs::msg::LaserScan& scan);
    // Gives the node read-only access to the grid for publishing as an OccupancyGrid.
    const std::vector<std::vector<int8_t>>& occupancyGrid() const;

  private:
    // Adds linearly decreasing costs around the original obstacle cells.
    void inflateObstacles();

    rclcpp::Logger logger_;
    // Rows are Y positions; columns are X positions. Values start at 0 (free).
    std::vector<std::vector<int8_t>> occupancy_grid_;
};

}  

#endif  
