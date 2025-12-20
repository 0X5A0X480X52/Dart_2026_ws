// Copyright 2025 Dart Team
// Licensed under the Apache License, Version 2.0

#ifndef STEREO_YOLO_DISTANCE__STEREO_YOLO_DISTANCE_NODE_HPP_
#define STEREO_YOLO_DISTANCE__STEREO_YOLO_DISTANCE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/target2_d_array.hpp>
#include <rm_interfaces/msg/target3_d_array.hpp>
#include <rm_interfaces/msg/target2_d.hpp>
#include <rm_interfaces/msg/target3_d.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <mutex>
#include <memory>

namespace stereo_yolo_distance
{

class StereoYoloDistanceNode : public rclcpp::Node
{
public:
  explicit StereoYoloDistanceNode(const rclcpp::NodeOptions & options);

private:
  // 回调函数
  void leftTargetCallback(const rm_interfaces::msg::Target2DArray::ConstSharedPtr & msg);
  void rightTargetCallback(const rm_interfaces::msg::Target2DArray::ConstSharedPtr & msg);
  void leftCameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);
  void rightCameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

  // 处理函数
  void processMatching();
  
  // 匹配和距离计算
  bool matchTargets(
    const rm_interfaces::msg::Target2D & left_target,
    const rm_interfaces::msg::Target2D & right_target,
    double & height_iou);
  
  double calculateHeightIOU(
    float left_y, float left_height,
    float right_y, float right_height);
  
  bool calculateDistance(
    const rm_interfaces::msg::Target2D & left_target,
    const rm_interfaces::msg::Target2D & right_target,
    rm_interfaces::msg::Target3D & target_3d);

  // 订阅器
  rclcpp::Subscription<rm_interfaces::msg::Target2DArray>::SharedPtr left_target_sub_;
  rclcpp::Subscription<rm_interfaces::msg::Target2DArray>::SharedPtr right_target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_camera_info_sub_;

  // 发布器
  rclcpp::Publisher<rm_interfaces::msg::Target3DArray>::SharedPtr target3d_pub_;

  // 数据缓存
  rm_interfaces::msg::Target2DArray::ConstSharedPtr latest_left_targets_;
  rm_interfaces::msg::Target2DArray::ConstSharedPtr latest_right_targets_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr left_camera_info_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr right_camera_info_;
  
  std::mutex left_mutex_;
  std::mutex right_mutex_;
  std::mutex camera_info_mutex_;

  // 参数
  std::string left_target_topic_;
  std::string right_target_topic_;
  std::string left_camera_info_topic_;
  std::string right_camera_info_topic_;
  std::string target3d_topic_;
  
  double min_height_iou_;       // 最小高度IOU阈值
  double max_distance_;         // 最大有效距离（米）
  double min_distance_;         // 最小有效距离（米）
  double max_disparity_;        // 最大视差（像素）
  double min_disparity_;        // 最小视差（像素）
  double max_time_diff_;        // 左右检测时间差最大容忍（秒）
  
  // 相机内参（从camera_info获取或配置文件读取）
  double fx_;                   // 焦距
  double baseline_;             // 基线距离（米）
  bool use_manual_calibration_; // 是否使用手动标定参数
};

}  // namespace stereo_yolo_distance

#endif  // STEREO_YOLO_DISTANCE__STEREO_YOLO_DISTANCE_NODE_HPP_
