// Created by Chengfu Zou on 2023.7.1
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SERIAL_DRIVER_SERIAL_DRIVER_NODE_HPP_
#define SERIAL_DRIVER_SERIAL_DRIVER_NODE_HPP_

// std
#include <memory>
#include <thread>
#include <mutex>
#include <chrono>
#include <climits>
// ros2
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// project
#include "fixed_packet/fixed_packet_tool.hpp"
#include "protocol/dart_protocol.hpp"
#include "protocol/protocol_factory.hpp"

#include "rm_interfaces/srv/get_desired_pos_x.hpp"
#include "rm_interfaces/msg/visual_input_data.hpp"
#include "rm_interfaces/msg/visual_display_data.hpp"
#include "rm_interfaces/msg/target2_d_array.hpp"
#include "rm_interfaces/msg/target2_d.hpp"

// Node wrapper for SerialDriver
// Implementing secondary development through the Protocol class
class SerialDriverNode : public rclcpp::Node {
public:
  explicit SerialDriverNode(const rclcpp::NodeOptions &options);

  ~SerialDriverNode();

  void init();

private:
  // Protocol
  std::unique_ptr<Protocol> protocol_;

  std::string target2d_topic_name_;
  std::string receive_target_topic_name_;
  std::string receive_debug_data_topic_name_;
  std::string send_debug_data_topic_name_;

  bool debug_mode_;

  int desired_pos_x_;

  rm_interfaces::msg::VisualInputData::SharedPtr latest_debug_data_;
  std::mutex debug_data_mutex_;

  // Parameter service
  rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr set_param_service_;

  // GetDesiredPosX service
  rclcpp::Service<rm_interfaces::srv::GetDesiredPosX>::SharedPtr get_desired_pos_x_service_;

  // Parameter callback
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters);

  // GetDesiredPosX service callback
  void handleGetDesiredPosX(
      const std::shared_ptr<rm_interfaces::srv::GetDesiredPosX::Request> request,
      std::shared_ptr<rm_interfaces::srv::GetDesiredPosX::Response> response);

  // Subscribtion
  rclcpp::Subscription<rm_interfaces::msg::VisualInputData>::SharedPtr debug_data_sub_;
  rclcpp::Subscription<rm_interfaces::msg::Target3DArray>::SharedPtr target_sub_;
  rclcpp::Subscription<rm_interfaces::msg::Target2DArray>::SharedPtr target_2d_sub_;

  // Publisher
  rclcpp::Publisher<rm_interfaces::msg::VisualDisplayData>::SharedPtr debug_data_pub_;

  // Callback for target data
  void targetCallback(const rm_interfaces::msg::Target3DArray::SharedPtr msg);
  
  // Callback for debug data
  void debugDataCallback(const rm_interfaces::msg::VisualInputData::SharedPtr msg);

  void target2dCallback(const rm_interfaces::msg::Target2DArray::SharedPtr msg);
  
  rm_interfaces::msg::Target2D filterateTarget2D(const std::vector<rm_interfaces::msg::Target2D> &targets);
  rm_interfaces::msg::Target2D curr_target_2d_;
  std::mutex target_2d_mutex_;
};

#endif  // SERIAL_DRIVER_SERIAL_DRIVER_NODE_HPP_
