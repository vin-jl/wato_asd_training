#include <chrono>
#include <memory>
#include <cmath>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
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
  const double RES = 0.1;
  const int WIDTH = 30/RES, HEIGHT = 30/RES;
  const int MAX_COST = 100;
  const int INFLATION_RADIUS = 1/RES;
  std::vector<std::vector<int>> costMap(WIDTH, std::vector<int>(HEIGHT, 0));

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            //calculate relative coordinates
            double x = range * cos(angle);
            double y = range * sin(angle);

            //convert x and y to grid values
            int x_grid = (int)(x/RES + WIDTH/2);
            int y_grid = (int)(y/RES + HEIGHT/2);

            if(x_grid < 0 || x_grid >= WIDTH || y_grid < 0 || y_grid >= HEIGHT){continue;}

            //set cost
            costMap[x_grid][y_grid] = MAX_COST;

            //inflate grid
            for(int dx = -INFLATION_RADIUS; dx<= INFLATION_RADIUS; dx++){
              for(int dy = -INFLATION_RADIUS; dy<= INFLATION_RADIUS; dy++){
                if (x_grid + dx < 0 || x_grid + dx >= WIDTH || y_grid + dy < 0 || y_grid + dy >= HEIGHT) continue;
                double dist = sqrt(dx * dx + dy * dy) * RES;
                if(dist > INFLATION_RADIUS){continue;}
                costMap[x_grid+dx][y_grid+dy] = std::max((int)costMap[x_grid+dx][y_grid+dy], (int)(MAX_COST * (1- std::min(1.0, dist/(INFLATION_RADIUS*RES)))));
              }
            }
        }
  }

  nav_msgs::msg::OccupancyGrid occupancyGrid;
  occupancyGrid.header.stamp = scan->header.stamp;
  occupancyGrid.header.frame_id = scan->header.frame_id;
  occupancyGrid.info.resolution = RES;
  occupancyGrid.info.height = HEIGHT;
  occupancyGrid.info.width = WIDTH;
  occupancyGrid.info.origin.position.x = -WIDTH*RES/2;
  occupancyGrid.info.origin.position.y = -HEIGHT*RES/2;

  //convert costmap to occupancyGrid
  occupancyGrid.data.resize(WIDTH * HEIGHT);
  for(int i=0; i<WIDTH; i++){
    for(int j=0; j<HEIGHT; j++){
      occupancyGrid.data[j * WIDTH + i] = costMap[i][j];
    }
  }
  costmap_pub_->publish(occupancyGrid);
}


 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}