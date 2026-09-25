#include "map_memory_core.hpp"

#include <cmath>
#include <cstddef>
#include <string>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

namespace
{

double quaternionYaw(const geometry_msgs::msg::Quaternion& quaternion) {
  return std::atan2(
      2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
      1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

}  // namespace

bool MapMemoryCore::integrateCostmap(
    const nav_msgs::msg::OccupancyGrid& costmap,
    const double robot_x,
    const double robot_y,
    const double robot_yaw,
    const std::string& global_frame) {
  const auto& source_info = costmap.info;
  const std::size_t expected_data_size =
      static_cast<std::size_t>(source_info.width) * source_info.height;
  if (source_info.width == 0 || source_info.height == 0 ||
      source_info.resolution <= 0.0F || costmap.data.size() != expected_data_size) {
    RCLCPP_WARN(logger_, "Ignoring malformed or empty costmap");
    return false;
  }

  if (!has_global_map_) {
    global_map_.header.frame_id = global_frame;
    global_map_.info = source_info;
    global_map_.info.origin.orientation.x = 0.0;
    global_map_.info.origin.orientation.y = 0.0;
    global_map_.info.origin.orientation.z = 0.0;
    global_map_.info.origin.orientation.w = 1.0;
    global_map_.data.assign(expected_data_size, -1);
    has_global_map_ = true;
  } else if (global_map_.info.width != source_info.width ||
             global_map_.info.height != source_info.height ||
             std::abs(global_map_.info.resolution - source_info.resolution) > 1e-6F) {
    RCLCPP_WARN(logger_, "Ignoring costmap whose geometry differs from the global map");
    return false;
  }

  const double source_origin_yaw = quaternionYaw(source_info.origin.orientation);
  const double source_cos = std::cos(source_origin_yaw);
  const double source_sin = std::sin(source_origin_yaw);
  const double robot_cos = std::cos(robot_yaw);
  const double robot_sin = std::sin(robot_yaw);
  const double map_origin_yaw = quaternionYaw(global_map_.info.origin.orientation);
  const double map_origin_cos = std::cos(map_origin_yaw);
  const double map_origin_sin = std::sin(map_origin_yaw);
  const double resolution = source_info.resolution;

  for (std::uint32_t row = 0; row < source_info.height; ++row) {
    for (std::uint32_t column = 0; column < source_info.width; ++column) {
      const std::size_t source_index =
          static_cast<std::size_t>(row) * source_info.width + column;
      const int8_t value = costmap.data[source_index];
      if (value < 0) {
        continue;  // Unknown source cells do not erase previously mapped data.
      }

      const double sensor_x = source_info.origin.position.x +
          source_cos * (column + 0.5) * resolution - source_sin * (row + 0.5) * resolution;
      const double sensor_y = source_info.origin.position.y +
          source_sin * (column + 0.5) * resolution + source_cos * (row + 0.5) * resolution;
      const double world_x = robot_x + robot_cos * sensor_x - robot_sin * sensor_y;
      const double world_y = robot_y + robot_sin * sensor_x + robot_cos * sensor_y;
      const double dx = world_x - global_map_.info.origin.position.x;
      const double dy = world_y - global_map_.info.origin.position.y;
      const double map_local_x = map_origin_cos * dx + map_origin_sin * dy;
      const double map_local_y = -map_origin_sin * dx + map_origin_cos * dy;
      const int map_column = static_cast<int>(std::floor(map_local_x / resolution));
      const int map_row = static_cast<int>(std::floor(map_local_y / resolution));

      if (map_column < 0 || map_row < 0 ||
          map_column >= static_cast<int>(global_map_.info.width) ||
          map_row >= static_cast<int>(global_map_.info.height)) {
        continue;
      }

      const std::size_t map_index =
          static_cast<std::size_t>(map_row) * global_map_.info.width + map_column;
      global_map_.data[map_index] = value;
    }
  }

  global_map_.header.stamp = costmap.header.stamp;
  global_map_.header.frame_id = global_frame;
  return true;
}

const nav_msgs::msg::OccupancyGrid& MapMemoryCore::globalMap() const {
  return global_map_;
}

} 
