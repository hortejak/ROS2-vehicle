#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create a basic node named "minimal_node"
  auto node = std::make_shared<rclcpp::Node>("minimal_node");

  RCLCPP_INFO(node->get_logger(), "Minimal CPP node has started and is doing nothing.");

  // Keep the node alive
  rclcpp::spin(node);

  // Shutdown cleanly
  rclcpp::shutdown();
  return 0;
}