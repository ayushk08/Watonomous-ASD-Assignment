// Connects /lidar to CostmapCore and publishes the resulting grid on /costmap.
#include <chrono>
#include <functional>
#include <memory>
 
#include "costmap_node.hpp"
 
// Sets up the lidar subscription, costmap publisher, and the retained warm-up timer/publisher.
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10,
    std::bind(&CostmapNode::laserScanCallback, this, std::placeholders::_1)); // gets lidar info, then calls laserScanCallBack function
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}
 
// Keeps the repository's warm-up demonstration publisher active at 2 Hz.
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

// Runs once per lidar message so the local grid reflects the latest scan.
void CostmapNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  costmap_.updateFromLaserScan(*msg);
  publishCostmap(msg->header.frame_id);
  RCLCPP_INFO(this->get_logger(), "Received lidar scan with %zu range measurements",
              msg->ranges.size());
}

// Fills grid geometry and metadata, copies row-major cell values, and publishes /costmap.
void CostmapNode::publishCostmap(const std::string& frame_id) {
  nav_msgs::msg::OccupancyGrid message;
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = frame_id;

  message.info.resolution = static_cast<float>(robot::CostmapCore::GRID_RESOLUTION);
  message.info.width = static_cast<std::uint32_t>(robot::CostmapCore::GRID_WIDTH);
  message.info.height = static_cast<std::uint32_t>(robot::CostmapCore::GRID_HEIGHT);
  message.info.origin.position.x = robot::CostmapCore::MAP_MIN_X;
  message.info.origin.position.y = robot::CostmapCore::MAP_MIN_Y;
  message.info.origin.position.z = 0.0;
  message.info.origin.orientation.x = 0.0;
  message.info.origin.orientation.y = 0.0;
  message.info.origin.orientation.z = 0.0;
  message.info.origin.orientation.w = 1.0;

  const auto& grid = costmap_.occupancyGrid();
  message.data.reserve(message.info.width * message.info.height);
  for (const auto& row : grid) {
    for (const int8_t cell_cost : row) {
      message.data.push_back(cell_cost);
    }
  }

  costmap_pub_->publish(message);
}
 
// Starts ROS, spins the costmap node so callbacks run, then shuts ROS down cleanly.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
