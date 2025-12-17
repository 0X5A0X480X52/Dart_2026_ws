// Copyright 2025 Dart Team
// Licensed under the Apache License, Version 2.0

#include <rclcpp/rclcpp.hpp>
#include "stereo_yolo_distance/stereo_yolo_distance_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<stereo_yolo_distance::StereoYoloDistanceNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
