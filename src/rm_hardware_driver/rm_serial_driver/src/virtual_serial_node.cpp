// Created for virtual serial testing
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

#include <chrono>
#include <memory>
#include <thread>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/target3_d_array.hpp"
#include "rm_interfaces/msg/visual_input_data.hpp"
#include "rm_interfaces/msg/visual_display_data.hpp"
#include "rm_interfaces/srv/get_desired_pos_x.hpp"
#include "../include/serial_struct/serial_struct.hpp"

class VirtualSerialNode : public rclcpp::Node {
public:
    explicit VirtualSerialNode(const rclcpp::NodeOptions &options)
    : Node("virtual_serial_node", options) {
        // 初始化参数
        receive_target_topic_name_ = this->declare_parameter("receive_topic_name", "/target");
        receive_debug_data_topic_name_ = this->declare_parameter("receive_debug_data_topic_name", "/debug/processed_data");
        send_debug_data_topic_name_ = this->declare_parameter("send_debug_data_topic_name", "/debug/original_data");
        desired_pos_x_ = this->declare_parameter("desiredPosX", 100);
        debug_mode_ = this->declare_parameter("debug_mode", false);

        // 创建发布者
        debug_data_pub_ = this->create_publisher<rm_interfaces::msg::VisualDisplayData>(
            send_debug_data_topic_name_, 10);

        // 创建订阅者
        target_sub_ = this->create_subscription<rm_interfaces::msg::Target3DArray>(
            receive_target_topic_name_, 10, 
            std::bind(&VirtualSerialNode::targetCallback, this, std::placeholders::_1));
        
        debug_data_sub_ = this->create_subscription<rm_interfaces::msg::VisualInputData>(
            receive_debug_data_topic_name_, 10, 
            std::bind(&VirtualSerialNode::debugDataCallback, this, std::placeholders::_1));

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
            });

        // 创建GetDesiredPosX服务
        get_desired_pos_x_service_ = this->create_service<rm_interfaces::srv::GetDesiredPosX>(
            "~/get_desired_pos_x",
            std::bind(&VirtualSerialNode::handleGetDesiredPosX, this, std::placeholders::_1, std::placeholders::_2));

        // 启动模拟数据发布线程
        simulation_thread_ = std::thread([this]() {
            auto gen = std::default_random_engine(this->now().nanoseconds());
            auto dis = std::uniform_real_distribution<float>(-1.0, 1.0);
            
            while (rclcpp::ok()) {
                if (debug_mode_) {
                    // 模拟接收串口数据
                    SerialReceiveData receive_data;
                    receive_data.Shoot_Force_Init = 100;  // 固定值
                    receive_data.Shoot_Force_Cur = 100;   // 固定值
                    receive_data.Shoot_Vel = 100.0f;      // 固定值
                    
                    auto msg = std::make_unique<rm_interfaces::msg::VisualDisplayData>();

                    msg->dart_init.resize(1);
                    msg->dart_cur.resize(1);
                    msg->dart_vel.resize(1);

                    msg->dart_init[0] = receive_data.Shoot_Force_Init;
                    msg->dart_cur[0] = receive_data.Shoot_Force_Cur;
                    msg->dart_vel[0] = receive_data.Shoot_Vel;
                    
                    // 添加一些随机噪声使数据显示更有趣
                    msg->dart_init[0] += static_cast<int>(dis(gen) * 5);
                    msg->dart_cur[0] += static_cast<int>(dis(gen) * 5);
                    msg->dart_vel[0] += dis(gen) * 5.0f;
                    
                    debug_data_pub_->publish(std::move(msg));
                }
                
                // 控制发布频率
                std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20Hz
            }
        });
    }

    ~VirtualSerialNode() {
        if (simulation_thread_.joinable()) {
            simulation_thread_.join();
        }
    }

private:
    // 参数回调
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters) {
        
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto &param : parameters) {
            if (param.get_name() == "desiredPosX") {
                desired_pos_x_ = param.as_int();
            }
        }

        return result;
    }

    // GetDesiredPosX服务回调
    void handleGetDesiredPosX(
        const std::shared_ptr<rm_interfaces::srv::GetDesiredPosX::Request> request,
        std::shared_ptr<rm_interfaces::srv::GetDesiredPosX::Response> response) {
        
        (void)request;  // 未使用的参数
        response->desired_pos_x = desired_pos_x_;
    }

    // 目标数据回调
    void targetCallback(const rm_interfaces::msg::Target3DArray::SharedPtr msg) {
        // 在虚拟串口中，我们只模拟发送数据而不实际处理
        SerialSendData send_data;
        send_data.Distance = 0.0f;
        send_data.Pixel_Error = 0;
        send_data.Debug_Mode = debug_mode_;
        send_data.Shoot_Force_Set = 0;

        if (!msg->targets.empty()) {
            // 简单地使用第一个目标的距离作为模拟数据
            send_data.Distance = static_cast<float>(msg->targets[0].position.x);
            send_data.Pixel_Error = static_cast<int>(msg->targets[0].position.y * 100);
        }

        if (debug_mode_ && latest_debug_data_ != nullptr) {
            std::lock_guard<std::mutex> lock(debug_data_mutex_);
            send_data.Shoot_Force_Set = latest_debug_data_->dart_set[0];
        }

        // 在真实实现中这里会通过串口发送数据
        // 但在虚拟节点中我们只是模拟这个过程

        (void)send_data;
    }

    // 调试数据回调
    void debugDataCallback(rm_interfaces::msg::VisualInputData::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(debug_data_mutex_);
        latest_debug_data_ = msg;
    }

    // 成员变量
    std::string receive_target_topic_name_;
    std::string receive_debug_data_topic_name_;
    std::string send_debug_data_topic_name_;

    bool debug_mode_;
    int desired_pos_x_;

    rm_interfaces::msg::VisualInputData::SharedPtr latest_debug_data_;
    std::mutex debug_data_mutex_;

    // 服务
    rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr set_param_service_;
    rclcpp::Service<rm_interfaces::srv::GetDesiredPosX>::SharedPtr get_desired_pos_x_service_;

    // 订阅者
    rclcpp::Subscription<rm_interfaces::msg::VisualInputData>::SharedPtr debug_data_sub_;
    rclcpp::Subscription<rm_interfaces::msg::Target3DArray>::SharedPtr target_sub_;

    // 发布者
    rclcpp::Publisher<rm_interfaces::msg::VisualDisplayData>::SharedPtr debug_data_pub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr target_timer_;

    // 模拟线程
    std::thread simulation_thread_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<VirtualSerialNode>(rclcpp::NodeOptions());
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Node execution failed: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}