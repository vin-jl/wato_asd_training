#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));

}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan){
  const int WIDTH = 30, HEIGHT = 30;
  const double RES = 0.1;
  const int MAX_COST = 100;
  const int INFLATION = 1;
  std::vector<std::vector<int>> costMap(WIDTH, std::vector<int>(HEIGHT, 0));

  // for (size_t i = 0; i < scan->ranges.size(); ++i) {
  //       double angle = scan->angle_min + i * scan->angle_increment;
  //       double range = scan->ranges[i];
  //       if (range < scan->range_max && range > scan->range_min) {
  //           // Calculate grid coordinates
  //           int x_grid, y_grid;
  //           convertToGrid(range, angle, x_grid, y_grid);
  //           markObstacle(x_grid, y_grid);
  //       }
  //     }

      nav_msgs::msg::OccupancyGrid occupancyGrid;
      occupancyGrid.header.stamp = scan->header.stamp;
      occupancyGrid.header.frame_id = scan->header.frame_id;
      occupancyGrid.info.resolution = RES;
      occupancyGrid.info.height = HEIGHT;
      occupancyGrid.info.width = WIDTH;
      occupancyGrid.info.origin.position.x = -15;
      occupancyGrid.info.origin.position.y = -15;

}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}