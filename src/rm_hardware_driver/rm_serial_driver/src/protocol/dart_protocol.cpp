// Created by Chengfu Zou
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

#include "protocol/dart_protocol.hpp"

Protocol::Protocol(std::string_view port_name, bool enable_data_print) {
  FYT_INFO("serial_driver_node", "Creating UART transporter for port: {}", port_name);
  auto uart_transporter = std::make_shared<UartTransporter>(std::string(port_name));

  FYT_INFO("serial_driver_node", "Creating FixedPacketTool");
  packet_tool_ = std::make_shared<FixedPacketTool<15>>(uart_transporter);
  
  FYT_INFO("serial_driver_node", "Opening UART transporter");
  if (!uart_transporter->open()) {
    FYT_ERROR("serial_driver_node", "Failed to open UART transporter: {}", uart_transporter->errorMessage());
    throw std::runtime_error("Failed to open UART transporter: " + uart_transporter->errorMessage());
  }

  FYT_INFO("serial_driver_node", "Enabling data print: {}", enable_data_print);
  packet_tool_->enbaleDataPrint(enable_data_print);
}

void Protocol::send(const SerialSendData &data) {
  FYT_DEBUG("serial_driver_node", "Sending data: Distance={}, Pixel_Error={}, Debug_Mode={}, Shoot_Force_Set={}",
            data.Distance, data.Pixel_Error, data.Debug_Mode, data.Shoot_Force_Set);
  FixedPacket<15> packet;
  packet.loadData<float>(data.Distance, 1);
  packet.loadData<int>(data.Pixel_Error, 5);
  packet.loadData<bool>(data.Debug_Mode, 9);
  packet.loadData<int>(data.Shoot_Force_Set, 10);
  packet_tool_->sendPacket(packet);
  FYT_DEBUG("serial_driver_node", "Sent data: Distance={}, Pixel_Error={}, Debug_Mode={}, Shoot_Force_Set={}",
            data.Distance, data.Pixel_Error, data.Debug_Mode, data.Shoot_Force_Set);
}

bool Protocol::receive(SerialReceiveData &data) {
  FixedPacket<15> packet;
  if (packet_tool_->recvPacket(packet)) {
    packet.unloadData(data.Shoot_Force_Init, 1);
    packet.unloadData(data.Shoot_Force_Cur, 5);
    packet.unloadData(data.Shoot_Vel, 9);
    FYT_DEBUG("serial_driver_node", "Received data: Shoot_Force_Init={}, Shoot_Force_Cur={}, Shoot_Vel={}",
              data.Shoot_Force_Init, data.Shoot_Force_Cur, data.Shoot_Vel);
    return true;
  } else {
    FYT_DEBUG("serial_driver_node", "Failed to receive data packet");
    return false;
  }
}

