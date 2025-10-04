#include <rclcpp/rclcpp.hpp>
#include "stereo_distance_estimator/stereo_distance_estimator_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;
  auto node = std::make_shared<stereo_distance_estimator::StereoDistanceEstimatorNode>(options);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}
