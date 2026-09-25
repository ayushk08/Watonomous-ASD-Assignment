#include <chrono>
#include <functional>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10,
    std::bind(&CostmapNode::laserScanCallback, this, std::placeholders::_1)); // gets lidar info, then calls laserScanCallBack function
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

void CostmapNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  costmap_.updateFromLaserScan(*msg);
  publishCostmap(msg->header.frame_id);
  RCLCPP_INFO(this->get_logger(), "Received lidar scan with %zu range measurements",
              msg->ranges.size());
}

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
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
