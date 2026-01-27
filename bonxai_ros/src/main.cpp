#include <rclcpp/rclcpp.hpp>
#include "bonxai_ros/bonxai_server.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  // Create node options (can be used for parameter overrides if needed)
  rclcpp::NodeOptions options;
  
  // Create the BonxaiServer node
  auto node = std::make_shared<Bonxai::BonxaiServer>(options);
  
  // Spin the node
  rclcpp::spin(node);
  
  // Cleanup
  rclcpp::shutdown();
  
  return 0;
}