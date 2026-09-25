#include "planner_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {}

namespace
{

struct GridCell {
  int x;
  int y;
};

struct OpenNode {
  std::size_t index;
  double cost;
};

// Makes the priority queue return the open cell with the lowest estimated total cost.
struct HigherCostFirst {
  bool operator()(const OpenNode& lhs, const OpenNode& rhs) const {
    return lhs.cost > rhs.cost;
  }
};

// Extracts the 2D heading angle from a quaternion orientation.
double quaternionYaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// Converts a world position (measured in meters) into a grid cell using the map's origin and resolution.
bool worldToCell(const nav_msgs::msg::OccupancyGrid& map, const geometry_msgs::msg::Point& point, GridCell& cell) {
  const double resolution = map.info.resolution;
  const double dx = point.x - map.info.origin.position.x;
  const double dy = point.y - map.info.origin.position.y;
  const double origin_yaw = quaternionYaw(map.info.origin.orientation);
  const double cos_yaw = std::cos(origin_yaw);
  const double sin_yaw = std::sin(origin_yaw);
  const double local_x = cos_yaw * dx + sin_yaw * dy;
  const double local_y = -sin_yaw * dx + cos_yaw * dy;

  cell.x = static_cast<int>(std::floor(local_x / resolution));
  cell.y = static_cast<int>(std::floor(local_y / resolution));
  return cell.x >= 0 && cell.y >= 0 &&
      cell.x < static_cast<int>(map.info.width) &&
      cell.y < static_cast<int>(map.info.height);
}

// Flattens a grid cell's x/y coordinates into the occupancy-grid data index.
std::size_t cellIndex(const GridCell& cell, const std::uint32_t width) {
  return static_cast<std::size_t>(cell.y) * width + static_cast<std::size_t>(cell.x);
}

// Treats cells at or above the occupancy threshold as obstacles.
bool isBlocked(const nav_msgs::msg::OccupancyGrid& map, const GridCell& cell) {
  constexpr int kBlockedOccupancy = 65;
  const int8_t occupancy = map.data[cellIndex(cell, map.info.width)];
  return occupancy >= kBlockedOccupancy;
}

// Estimates remaining travel cost with the diagonal-distance heuristic.
double heuristic(const GridCell& from, const GridCell& to) {
  const int dx = std::abs(from.x - to.x);
  const int dy = std::abs(from.y - to.y);
  const int diagonal_steps = std::min(dx, dy);
  const int straight_steps = std::max(dx, dy) - diagonal_steps;
  return diagonal_steps * std::sqrt(2.0) + straight_steps;
}

// Converts a grid cell to its center position in the map's world frame.
geometry_msgs::msg::Point cellCenter(
    const nav_msgs::msg::OccupancyGrid& map, const GridCell& cell) {
  const double resolution = map.info.resolution;
  const double local_x = (cell.x + 0.5) * resolution;
  const double local_y = (cell.y + 0.5) * resolution;
  const double origin_yaw = quaternionYaw(map.info.origin.orientation);

  geometry_msgs::msg::Point point;
  point.x = map.info.origin.position.x +
      std::cos(origin_yaw) * local_x - std::sin(origin_yaw) * local_y;
  point.y = map.info.origin.position.y +
      std::sin(origin_yaw) * local_x + std::cos(origin_yaw) * local_y;
  point.z = 0.0;
  return point;
}

}  // namespace

// Runs A* on the occupancy grid and returns world-coordinate waypoints if a route exists.
bool PlannerCore::findPath( const nav_msgs::msg::OccupancyGrid& map, const geometry_msgs::msg::Point& start, const geometry_msgs::msg::Point& goal,
    std::vector<geometry_msgs::msg::Point>& path) const 
{
  path.clear();
  const std::size_t cell_count =
      static_cast<std::size_t>(map.info.width) * map.info.height;
  if (map.info.width == 0 || map.info.height == 0 ||
      !std::isfinite(map.info.resolution) || map.info.resolution <= 0.0F ||
      map.data.size() != cell_count) {
    RCLCPP_WARN(logger_, "Cannot plan: occupancy grid has invalid dimensions or data");
    return false;
  }

  GridCell start_cell{};
  GridCell goal_cell{};
  if (!worldToCell(map, start, start_cell) || !worldToCell(map, goal, goal_cell)) {
    RCLCPP_WARN(logger_, "Cannot plan: start or goal is outside the occupancy grid");
    return false;
  }
  if (isBlocked(map, start_cell) || isBlocked(map, goal_cell)) {
    RCLCPP_WARN(logger_, "Cannot plan: start or goal lies in an occupied cell");
    return false;
  }

  if (start_cell.x == goal_cell.x && start_cell.y == goal_cell.y) {
    path.push_back(start);
    if (std::hypot(goal.x - start.x, goal.y - start.y) > 1e-6) {
      path.push_back(goal);
    }
    return true;
  }

  constexpr std::array<GridCell, 8> kNeighbors{{
      {1, 0}, {-1, 0}, {0, 1}, {0, -1},
      {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};
  constexpr double kBlockedThreshold = 65.0; // minimum value for cell to be considered occupied
  constexpr double kUnknownCellPenalty = 2.0;
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> costs(cell_count, infinity);
  std::vector<std::size_t> parents(cell_count, cell_count);
  std::vector<bool> closed(cell_count, false);
  std::priority_queue<OpenNode, std::vector<OpenNode>, HigherCostFirst> open_set;

  const std::size_t start_index = cellIndex(start_cell, map.info.width);
  const std::size_t goal_index = cellIndex(goal_cell, map.info.width);
  costs[start_index] = 0.0;
  open_set.push({start_index, heuristic(start_cell, goal_cell)});

  while (!open_set.empty()) {
    const OpenNode current_node = open_set.top();
    open_set.pop();
    const std::size_t current_index = current_node.index;
    if (closed[current_index]) {
      continue;
    }
    closed[current_index] = true;

    if (current_index == goal_index) {
      break;
    }

    const GridCell current_cell{
        static_cast<int>(current_index % map.info.width),
        static_cast<int>(current_index / map.info.width)};
    for (const GridCell& offset : kNeighbors) {
      const GridCell neighbor{current_cell.x + offset.x, current_cell.y + offset.y};
      if (neighbor.x < 0 || neighbor.y < 0 ||
          neighbor.x >= static_cast<int>(map.info.width) ||
          neighbor.y >= static_cast<int>(map.info.height) || isBlocked(map, neighbor)) {
        continue;
      }

      const bool diagonal = offset.x != 0 && offset.y != 0;
      if (diagonal &&
          (isBlocked(map, {current_cell.x + offset.x, current_cell.y}) ||
           isBlocked(map, {current_cell.x, current_cell.y + offset.y}))) {
        continue;  // Do not cut diagonally through the corner of an obstacle.
      }

      const std::size_t neighbor_index = cellIndex(neighbor, map.info.width);
      if (closed[neighbor_index]) {
        continue;
      }

      const int8_t occupancy = map.data[neighbor_index];
      const double cell_penalty = occupancy < 0
          ? kUnknownCellPenalty
          : 1.0 + 4.0 * (static_cast<double>(occupancy) / kBlockedThreshold);
      const double step_distance = diagonal ? std::sqrt(2.0) : 1.0;
      const double candidate_cost = costs[current_index] + step_distance * cell_penalty;
      if (candidate_cost >= costs[neighbor_index]) {
        continue;
      }

      costs[neighbor_index] = candidate_cost;
      parents[neighbor_index] = current_index;
      open_set.push({
          neighbor_index,
          candidate_cost + heuristic(neighbor, goal_cell)});
    }
  }

  if (!closed[goal_index]) {
    RCLCPP_WARN(logger_, "No traversable path exists from start to goal");
    return false;
  }

  std::vector<std::size_t> reversed_cells;
  for (std::size_t index = goal_index; index != start_index; index = parents[index]) {
    if (index == cell_count || parents[index] == cell_count) {
      RCLCPP_WARN(logger_, "Failed to reconstruct the planned path");
      return false;
    }
    reversed_cells.push_back(index);
  }
  std::reverse(reversed_cells.begin(), reversed_cells.end());

  path.push_back(start);
  for (const std::size_t index : reversed_cells) {
    const GridCell cell{
        static_cast<int>(index % map.info.width),
        static_cast<int>(index / map.info.width)};
    path.push_back(cellCenter(map, cell));
  }
  if (std::hypot(path.back().x - goal.x, path.back().y - goal.y) > 1e-6) {
    path.push_back(goal);
  }
  return true;
}

} 
