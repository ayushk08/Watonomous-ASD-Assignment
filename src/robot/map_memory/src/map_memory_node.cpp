// Connects /costmap and /odom/filtered to the map fusion core and publishes /map at 1 Hz.
#include "map_memory_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>

// Creates both input subscriptions, the global map publisher, and a 1 s update timer.
MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::aggregateMaps, this, std::placeholders::_1));
  odom_filtered_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::trackRobotMovement, this, std::placeholders::_1));
  map_update_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
  global_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
}


// Retains the most recent local grid; expensive fusion is deferred to the periodic timer.
void MapMemoryNode::aggregateMaps(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
  latest_costmap_ = costmap;
}

// Updates the latest pose and its straight-line distance from the last fused pose.
void MapMemoryNode::trackRobotMovement(const nav_msgs::msg::Odometry::SharedPtr odom) {
  current_x_ = odom->pose.pose.position.x;
  current_y_ = odom->pose.pose.position.y;
  if (!odom->header.frame_id.empty()) {
    odom_frame_ = odom->header.frame_id;
  }
  const auto& orientation = odom->pose.pose.orientation;
  current_yaw_ = std::atan2(
      2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
      1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
  has_odom_pose_ = true;

  // Use the first odometry sample as the initial map update reference.
  if (!has_last_update_position_) {
    last_update_x_ = current_x_;
    last_update_y_ = current_y_;
    has_last_update_position_ = true;
    distance_since_last_update_ = 0.0;
  } else {
    distance_since_last_update_ = std::hypot(
        current_x_ - last_update_x_, current_y_ - last_update_y_);
  }

}

// Runs at 1 Hz to limit work and republishes the accumulated map once initialized.
void MapMemoryNode::updateMap() {
  tryAggregateLatestCostmap();

  if (!has_integrated_costmap_) {
    return;
  }

  auto global_map = map_memory_.globalMap();
  global_map.header.stamp = this->get_clock()->now();
  global_map_pub_->publish(global_map);
}

// Fuses once at startup, then only after 1.5 m of motion; resets the reference after success.
void MapMemoryNode::tryAggregateLatestCostmap() {
  // 1.5 m follows the assignment's implementation guidance and the chosen update interval.
  constexpr double kMapUpdateDistance = 1.5;
  if (!latest_costmap_ || !has_odom_pose_) {
    return;
  }

  // Integrate once at startup, then again whenever the robot has moved 1.5 m.
  if (has_integrated_costmap_ && distance_since_last_update_ < kMapUpdateDistance) {
    return;
  }

  if (!map_memory_.integrateCostmap(
          *latest_costmap_, current_x_, current_y_, current_yaw_, odom_frame_)) {
    return;
  }

  has_integrated_costmap_ = true;
  last_update_x_ = current_x_;
  last_update_y_ = current_y_;
  distance_since_last_update_ = 0.0;
}

// Starts ROS and processes map/odometry callbacks and the periodic map timer.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
