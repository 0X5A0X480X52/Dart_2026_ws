// Created by Chengfu Zou on 2023.7.6
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

#ifndef SERIAL_DRIVER_PROTOCOL_HPP_
#define SERIAL_DRIVER_PROTOCOL_HPP_

// std
#include <memory>
#include <string>
#include <string_view>
// ros2
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
// project
// #include <rm_interfaces/msg/target3_d_array.hpp>
#include "rm_interfaces/msg/target3_d_array.hpp"
#include "../fixed_packet/fixed_packet.hpp"
#include "../fixed_packet/fixed_packet_tool.hpp"
#include "transporter/uart_transporter.hpp"
#include "../serial_struct/serial_struct.hpp"

// typedef enum : unsigned char { Fire = 0x01, NotFire = 0x00 } FireState;

// Protocol interface
class Protocol {
public:

  Protocol(std::string_view port_name, bool enable_data_print);
  virtual ~Protocol() = default;

  // Send
  void send(const SerialSendData &data);

  // Receive
  bool receive(SerialReceiveData &data);

  std::string getErrorMessage() { return packet_tool_->getErrorMessage(); };

private:
  FixedPacketTool<15>::SharedPtr packet_tool_;

};

#endif  // SERIAL_DRIVER_PROTOCOLS_HPP_
