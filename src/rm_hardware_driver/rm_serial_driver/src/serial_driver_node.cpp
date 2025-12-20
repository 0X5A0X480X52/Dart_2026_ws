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

#include "serial_driver_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
// std
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <limits>
// ros2
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
// project
#include <rm_utils/logger/log.hpp>
#include <rm_utils/math/utils.hpp>
// struct
#include "serial_struct/serial_struct.hpp"
SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions &options)
: Node("serial_driver_node", options) {
  try {
    FYT_REGISTER_LOGGER("serial_driver_node", "~/fyt2024-log", INFO);
    this->init();
  } catch (const std::exception &e) {
    std::cout<<"\n[SerialDriverNode Init Error]: "<<(std::string)e.what()<<std::endl;
  }
}

void SerialDriverNode::init() {
  FYT_INFO("serial_driver_node", "Initializing SerialDriverNode!");

  // Init
  target2d_topic_name_ = this->declare_parameter("target2d_topic_name", "/detector/target2d_array");
  receive_target_topic_name_ = this->declare_parameter("receive_topic_name", "/target");
  receive_debug_data_topic_name_ = this->declare_parameter("receive_debug_data_topic_name", "/debug/processed_data");
  send_debug_data_topic_name_ = this->declare_parameter("send_debug_data_topic_name", "/debug/original_data");
  std::string port_name = this->declare_parameter("port_name", "/dev/ttyACM0");
  std::string protocol_type = this->declare_parameter("protocol", "dart");
  bool enable_data_print = this->declare_parameter("enable_data_print", false);
  desired_pos_x_ = this->declare_parameter("desiredPosX", 200);
  debug_mode_ = this->declare_parameter("debug_mode", false);

  // 创建发布者
  debug_data_pub_ = this->create_publisher<rm_interfaces::msg::VisualDisplayData>(
      send_debug_data_topic_name_, 10);

  // 创建订阅者
  target_sub_ = this->create_subscription<rm_interfaces::msg::Target3DArray>(
      receive_target_topic_name_, 10, std::bind(&SerialDriverNode::targetCallback, this, std::placeholders::_1));
  debug_data_sub_ = this->create_subscription<rm_interfaces::msg::VisualInputData>(
      receive_debug_data_topic_name_, 10, std::bind(&SerialDriverNode::debugDataCallback, this, std::placeholders::_1));
  target_2d_sub_ = this->create_subscription<rm_interfaces::msg::Target2DArray>(
      target2d_topic_name_, 10, std::bind(&SerialDriverNode::target2dCallback, this, std::placeholders::_1));

  // 创建参数服务
  set_param_service_ = this->create_service<rcl_interfaces::srv::SetParameters>(
      "~/set_parameters",
      [this](
          const std::shared_ptr<rmw_request_id_t> request_header,
          const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request,
          std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response) {
                
          (void)request_header;

          // 将请求转换为参数向量
          std::vector<rclcpp::Parameter> parameters;
          for (const auto &p : request->parameters) {
              parameters.push_back(rclcpp::Parameter::from_parameter_msg(p));
          }
                
          // 调用参数回调函数
          auto result = this->parametersCallback(parameters);
                
          // 设置响应
          response->results.push_back(result);
      }
    );
  
  // Create GetDesiredPosX service
  get_desired_pos_x_service_ = this->create_service<rm_interfaces::srv::GetDesiredPosX>(
      "~/get_desired_pos_x",
      std::bind(&SerialDriverNode::handleGetDesiredPosX, this, std::placeholders::_1, std::placeholders::_2));

  // Initialize curr_target_2d_ to prevent uninitialized access
  curr_target_2d_ = rm_interfaces::msg::Target2D();
  curr_target_2d_.x = 0.0f;
  curr_target_2d_.y = 0.0f;
  curr_target_2d_.confidence = 0.0f;

      // Create Protocol
  protocol_ = ProtocolFactory::createProtocol(protocol_type, port_name, enable_data_print);
  if (protocol_ == nullptr) {
    FYT_FATAL("serial_driver_node", "Failed to create protocol with type: {}", protocol_type);
    rclcpp::shutdown();
    return;
  }

  std::cout<<"\nfinished init"<<std::endl;

  if (debug_mode_) {
    std::thread([this]() {
      while (rclcpp::ok()) {
        if (protocol_ != nullptr) {
          SerialReceiveData receive_data;
          if (protocol_->receive(receive_data)) {
            auto msg = std::make_unique<rm_interfaces::msg::VisualDisplayData>();
            msg->dart_init.resize(1);
            msg->dart_cur.resize(1);
            msg->dart_vel.resize(1);
            msg->dart_init[0] = receive_data.Shoot_Force_Init;
            msg->dart_cur[0] = receive_data.Shoot_Force_Cur;
            msg->dart_vel[0] = receive_data.Shoot_Vel;
            debug_data_pub_->publish(std::move(msg));
          }
        }
        // Small delay to prevent busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }).detach();
  }
}

SerialDriverNode::~SerialDriverNode() {
  FYT_INFO("serial_driver_node", "Destroy SerialDriverNode!");
  rclcpp::shutdown();
}

void SerialDriverNode::target2dCallback(const rm_interfaces::msg::Target2DArray::SharedPtr msg) {
  if (msg == nullptr) {
    FYT_WARN("serial_driver_node", "Received null Target2DArray message");
    return;
  }

  const auto &targets = msg->targets;

  // Guard: empty list -> clear current target
  if (targets.empty()) {
    std::lock_guard<std::mutex> lock(target_2d_mutex_);
    curr_target_2d_ = rm_interfaces::msg::Target2D();
    has_target_ = false;
    FYT_DEBUG("serial_driver_node", "Received empty Target2DArray; cleared current target");
    return;
  }

  // Guard: protect against unreasonable sizes (likely corrupted message)
  constexpr size_t kMaxReasonableTargets = 1000;
  if (targets.size() > kMaxReasonableTargets) {
    FYT_WARN("serial_driver_node", "Target2DArray size unreasonable (%zu); ignoring message", targets.size());
    return;
  }

  // Safe path
  auto target = filterateTarget2D(targets);
  std::lock_guard<std::mutex> lock(target_2d_mutex_);
  curr_target_2d_ = target;
  has_target_ = true;
}

rm_interfaces::msg::Target2D SerialDriverNode::filterateTarget2D(const std::vector<rm_interfaces::msg::Target2D> &targets) {
  // Guard against empty input
  if (targets.empty()) {
    return rm_interfaces::msg::Target2D();
  }

  // Find the index with the highest confidence
  float max_confidence = -std::numeric_limits<float>::infinity();
  size_t idx = 0;
  for (size_t i = 0; i < targets.size(); ++i) {
    if (targets[i].confidence > max_confidence) {
      max_confidence = targets[i].confidence;
      idx = i;
    }
  }
  return targets[idx];
}

rcl_interfaces::msg::SetParametersResult SerialDriverNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto &param : parameters) {
    if (param.get_name() == "desiredPosX") {
      desired_pos_x_ = param.as_int();
      FYT_INFO("serial_driver_node", "Updated desiredPosX to {}", desired_pos_x_);
    }
    // Add more parameters here if needed
  }

  return result;
}

void SerialDriverNode::handleGetDesiredPosX(
    const std::shared_ptr<rm_interfaces::srv::GetDesiredPosX::Request> request,
    std::shared_ptr<rm_interfaces::srv::GetDesiredPosX::Response> response) {
  (void)request;  // Unused parameter
  response->desired_pos_x = desired_pos_x_;
  FYT_INFO("serial_driver_node", "Served current desiredPosX: {}", desired_pos_x_);
}

void SerialDriverNode::targetCallback(const rm_interfaces::msg::Target3DArray::SharedPtr msg) {
  // Process target data and send via serial port
  if (protocol_ != nullptr) {
    // Create SerialSendData with debug mode setting
    SerialSendData send_data;

    // If no targets, send distance = -1 and mark status
    if (msg->targets.empty()) {
      FYT_WARN("serial_driver_node", "Received empty Target3DArray, sending Distance=-1");
      send_data.Distance = -1.0f;
      has_target_ = false;
      std::lock_guard<std::mutex> lock(target_2d_mutex_);
      send_data.Pixel_Error = 0;  // no valid pixel error when no target
    } else {
      send_data.Distance = msg->targets[0].distance;
      has_target_ = true;

      std::lock_guard<std::mutex> lock(target_2d_mutex_);
      send_data.Pixel_Error = curr_target_2d_.x - desired_pos_x_;
    }

    send_data.Debug_Mode = debug_mode_;

    if (debug_mode_ && latest_debug_data_ != nullptr && !latest_debug_data_->dart_set.empty()) {
      std::lock_guard<std::mutex> lock(debug_data_mutex_);
      send_data.Shoot_Force_Set = latest_debug_data_->dart_set[0];
    }
    
    // Send data via protocol
    protocol_->send(send_data);
  }
}

void SerialDriverNode::debugDataCallback(rm_interfaces::msg::VisualInputData::SharedPtr msg) {
  std::cout<<"debugDataCallback"<<std::endl;
  std::lock_guard<std::mutex> lock(debug_data_mutex_);
  latest_debug_data_ = msg;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<SerialDriverNode>(rclcpp::NodeOptions());
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Node execution failed: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}