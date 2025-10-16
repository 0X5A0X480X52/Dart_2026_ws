#include "resize_image_raw/resize_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<resize_image_raw::ResizeNode>();
  
  // No need to call initialize() manually anymore
  // It's automatically called via a one-shot timer in the constructor
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
