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

#ifndef SERIAL_DRIVER_PROTOCOL_FACTORY_HPP_
#define SERIAL_DRIVER_PROTOCOL_FACTORY_HPP_

#include <memory>
#include <string_view>

#include "protocol/dart_protocol.hpp"

class ProtocolFactory {
public:
  ProtocolFactory() = delete;
  // Factory method to create a protocol
  static std::unique_ptr<Protocol> createProtocol(std::string_view protocol_type,
                                                            std::string_view port_name,
                                                            bool enable_data_print) {
    if (protocol_type == "dart") {
      return std::make_unique<Protocol>(port_name, enable_data_print);
    }

    return nullptr;
  }
};

#endif  // SERIAL_DRIVER_PROTOCOL_FACTORY_HPP_
