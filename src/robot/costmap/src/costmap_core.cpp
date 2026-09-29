// Builds a fresh lidar-centered occupancy grid for each scan, then adds obstacle-clearance costs.
#include "costmap_core.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace robot
{
// Initializes all cells as free; each scan repopulates the local grid before inflation.
CostmapCore::CostmapCore(const rclcpp::Logger& logger)
  : logger_(logger),
    occupancy_grid_(GRID_HEIGHT, std::vector<int8_t>(GRID_WIDTH, 0)) {}

// Converts valid polar laser returns into grid cells and refreshes the obstacle cost field.
void CostmapCore::updateFromLaserScan(const sensor_msgs::msg::LaserScan& scan) {
  // Keep this as a local costmap: each new scan replaces the previous scan.
  for (auto& row : occupancy_grid_) {
    std::fill(row.begin(), row.end(), 0);
  }

  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      continue;
    }

    const double angle = scan.angle_min + i * scan.angle_increment;
    const double x = range * std::cos(angle);
    const double y = range * std::sin(angle);

    const int column = static_cast<int>(std::floor((x - MAP_MIN_X) / GRID_RESOLUTION));
    const int row = static_cast<int>(std::floor((y - MAP_MIN_Y) / GRID_RESOLUTION));

    if (column < 0 || column >= static_cast<int>(GRID_WIDTH) ||
        row < 0 || row >= static_cast<int>(GRID_HEIGHT)) {
      continue;
    }

    occupancy_grid_[row][column] = 100;
  }

  inflateObstacles();
}

// Adds a linearly decreasing cost around each original obstacle, up to INFLATION_RADIUS.
void CostmapCore::inflateObstacles() {
  // Save the original obstacle locations so newly inflated cells are not
  // treated as additional obstacles during this same pass.
  std::vector<std::pair<int, int>> obstacle_cells;
  for (int row = 0; row < static_cast<int>(GRID_HEIGHT); ++row) {
    for (int column = 0; column < static_cast<int>(GRID_WIDTH); ++column) {
      if (occupancy_grid_[row][column] == MAX_COST) {
        obstacle_cells.emplace_back(row, column);
      }
    }
  }

  const int cells_in_radius = static_cast<int>(std::ceil(INFLATION_RADIUS / GRID_RESOLUTION));
  for (const auto& [obstacle_row, obstacle_column] : obstacle_cells) {
    for (int row_offset = -cells_in_radius; row_offset <= cells_in_radius; ++row_offset) {
      for (int column_offset = -cells_in_radius; column_offset <= cells_in_radius; ++column_offset) {
        const int row = obstacle_row + row_offset;
        const int column = obstacle_column + column_offset;
        if (row < 0 || row >= static_cast<int>(GRID_HEIGHT) ||
            column < 0 || column >= static_cast<int>(GRID_WIDTH)) {
          continue;
        }

        const double distance = std::hypot(row_offset, column_offset) * GRID_RESOLUTION;
        if (distance >= INFLATION_RADIUS) {
          continue;
        }

        const double cost = MAX_COST * (1.0 - distance / INFLATION_RADIUS);
        const auto cell_cost = static_cast<int8_t>(std::lround(cost));
        occupancy_grid_[row][column] = std::max(occupancy_grid_[row][column], cell_cost);
      }
    }
  }
}

// Returns the current grid; row is y and column is x, matching OccupancyGrid row-major order.
const std::vector<std::vector<int8_t>>& CostmapCore::occupancyGrid() const {
  return occupancy_grid_;
}

}
