#include "resize_image_raw/resize_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<resize_image_raw::ResizeNode>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
