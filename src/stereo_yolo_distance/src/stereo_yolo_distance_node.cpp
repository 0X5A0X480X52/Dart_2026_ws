// Copyright 2025 Dart Team
// Licensed under the Apache License, Version 2.0

#include "stereo_yolo_distance/stereo_yolo_distance_node.hpp"
#include <algorithm>
#include <cmath>

namespace stereo_yolo_distance
{

StereoYoloDistanceNode::StereoYoloDistanceNode(const rclcpp::NodeOptions & options)
: Node("stereo_yolo_distance", options)
{
  // 声明参数
  this->declare_parameter("left_target_topic", "/detector/left/target2d_array");
  this->declare_parameter("right_target_topic", "/detector/right/target2d_array");
  this->declare_parameter("left_camera_info_topic", "/camera/left/camera_info");
  this->declare_parameter("right_camera_info_topic", "/camera/right/camera_info");
  this->declare_parameter("target3d_topic", "/stereo/target3d_array");
  
  this->declare_parameter("min_height_iou", 0.5);
  this->declare_parameter("max_distance", 10.0);
  this->declare_parameter("min_distance", 0.5);
  this->declare_parameter("max_disparity", 300.0);
  this->declare_parameter("min_disparity", 10.0);
  
  // 手动标定参数（可选）
  this->declare_parameter("use_manual_calibration", false);
  this->declare_parameter("fx", 1550.0);        // 从camera_info.yaml获取
  this->declare_parameter("baseline", 0.120);   // 基线距离，需要实际测量

  // 获取参数
  left_target_topic_ = this->get_parameter("left_target_topic").as_string();
  right_target_topic_ = this->get_parameter("right_target_topic").as_string();
  left_camera_info_topic_ = this->get_parameter("left_camera_info_topic").as_string();
  right_camera_info_topic_ = this->get_parameter("right_camera_info_topic").as_string();
  target3d_topic_ = this->get_parameter("target3d_topic").as_string();
  
  min_height_iou_ = this->get_parameter("min_height_iou").as_double();
  max_distance_ = this->get_parameter("max_distance").as_double();
  min_distance_ = this->get_parameter("min_distance").as_double();
  max_disparity_ = this->get_parameter("max_disparity").as_double();
  min_disparity_ = this->get_parameter("min_disparity").as_double();
  
  use_manual_calibration_ = this->get_parameter("use_manual_calibration").as_bool();
  fx_ = this->get_parameter("fx").as_double();
  baseline_ = this->get_parameter("baseline").as_double();

  // 创建订阅器
  left_target_sub_ = this->create_subscription<rm_interfaces::msg::Target2DArray>(
    left_target_topic_, 10,
    std::bind(&StereoYoloDistanceNode::leftTargetCallback, this, std::placeholders::_1));

  right_target_sub_ = this->create_subscription<rm_interfaces::msg::Target2DArray>(
    right_target_topic_, 10,
    std::bind(&StereoYoloDistanceNode::rightTargetCallback, this, std::placeholders::_1));

  left_camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    left_camera_info_topic_, 10,
    std::bind(&StereoYoloDistanceNode::leftCameraInfoCallback, this, std::placeholders::_1));

  right_camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    right_camera_info_topic_, 10,
    std::bind(&StereoYoloDistanceNode::rightCameraInfoCallback, this, std::placeholders::_1));

  // 创建发布器
  target3d_pub_ = this->create_publisher<rm_interfaces::msg::Target3DArray>(
    target3d_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "Stereo YOLO Distance Node initialized");
  RCLCPP_INFO(this->get_logger(), "  Left target topic: %s", left_target_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Right target topic: %s", right_target_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Target3D output topic: %s", target3d_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Min height IOU: %.2f", min_height_iou_);
  RCLCPP_INFO(this->get_logger(), "  Distance range: [%.2f, %.2f] m", min_distance_, max_distance_);
  RCLCPP_INFO(this->get_logger(), "  Disparity range: [%.1f, %.1f] pixels", min_disparity_, max_disparity_);
  
  if (use_manual_calibration_) {
    RCLCPP_INFO(this->get_logger(), "  Using manual calibration: fx=%.2f, baseline=%.3f m", 
                fx_, baseline_);
  } else {
    RCLCPP_INFO(this->get_logger(), "  Waiting for camera_info topics for calibration");
  }
}

void StereoYoloDistanceNode::leftTargetCallback(
  const rm_interfaces::msg::Target2DArray::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(left_mutex_);
  latest_left_targets_ = msg;

  RCLCPP_DEBUG(this->get_logger(), "Received left targets: count=%zu", msg->targets.size());
  
  // 触发匹配处理
  processMatching();
}

void StereoYoloDistanceNode::rightTargetCallback(
  const rm_interfaces::msg::Target2DArray::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(right_mutex_);
  latest_right_targets_ = msg;

  RCLCPP_DEBUG(this->get_logger(), "Received right targets: count=%zu", msg->targets.size());
  
  // 触发匹配处理
  processMatching();
}

void StereoYoloDistanceNode::leftCameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(camera_info_mutex_);
  left_camera_info_ = msg;
  
  // 从camera_info提取焦距（如果不使用手动标定）
  if (!use_manual_calibration_ && !msg->p.empty()) {
    fx_ = msg->p[0];  // P[0,0] = fx
    RCLCPP_INFO_ONCE(this->get_logger(), "Extracted fx from left camera_info: %.2f", fx_);
  }
}

void StereoYoloDistanceNode::rightCameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(camera_info_mutex_);
  right_camera_info_ = msg;
  
  // 从右相机的投影矩阵提取基线信息
  // P[0,3] = -fx * baseline (对于校正后的立体相机)
  if (!use_manual_calibration_ && !msg->p.empty() && fx_ > 0) {
    double tx = msg->p[3];  // P[0,3]
    if (tx < 0) {
      baseline_ = -tx / fx_;
      RCLCPP_INFO_ONCE(this->get_logger(), "Extracted baseline from right camera_info: %.3f m", 
                       baseline_);
    }
  }
}

void StereoYoloDistanceNode::processMatching()
{
  // 获取最新的左右目标
  rm_interfaces::msg::Target2DArray::ConstSharedPtr left_targets;
  rm_interfaces::msg::Target2DArray::ConstSharedPtr right_targets;
  
  {
    std::lock_guard<std::mutex> lock_left(left_mutex_);
    std::lock_guard<std::mutex> lock_right(right_mutex_);
    
    if (!latest_left_targets_ || !latest_right_targets_) {
      RCLCPP_DEBUG(this->get_logger(), "Waiting for both left and right targets: left=%s, right=%s",
                   latest_left_targets_ ? "ready" : "none",
                   latest_right_targets_ ? "ready" : "none");
      return;  // 还没有接收到两侧的数据
    }
    
    left_targets = latest_left_targets_;
    right_targets = latest_right_targets_;
  }

  // 检查是否有标定参数
  if (fx_ <= 0 || baseline_ <= 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Camera calibration not ready: fx=%.2f, baseline=%.3f", fx_, baseline_);
    RCLCPP_DEBUG(this->get_logger(), "Left camera_info present=%s, right camera_info present=%s",
                 left_camera_info_ ? "yes" : "no", right_camera_info_ ? "yes" : "no");
    return;
  }

  // 检查时间戳差异（允许一定的时间差）
  double time_diff = std::abs(
    rclcpp::Time(left_targets->header.stamp).seconds() - 
    rclcpp::Time(right_targets->header.stamp).seconds());
  
  if (time_diff > 0.1) {  // 100ms 容差
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Large time difference between left and right detections: %.3f s", time_diff);
    RCLCPP_DEBUG(this->get_logger(), "Left stamp: %u.%u, Right stamp: %u.%u",
                 left_targets->header.stamp.sec, left_targets->header.stamp.nanosec,
                 right_targets->header.stamp.sec, right_targets->header.stamp.nanosec);
  }

  // 当前假设只有一个目标
  if (left_targets->targets.empty() || right_targets->targets.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "No targets: left=%zu, right=%zu",
                 left_targets->targets.size(), right_targets->targets.size());
    return;  // 至少一侧没有检测到目标
  }

  // 简化处理：取置信度最高的目标
  const auto & left_target = *std::max_element(
    left_targets->targets.begin(), left_targets->targets.end(),
    [](const auto & a, const auto & b) { return a.confidence < b.confidence; });

  const auto & right_target = *std::max_element(
    right_targets->targets.begin(), right_targets->targets.end(),
    [](const auto & a, const auto & b) { return a.confidence < b.confidence; });

  // 尝试匹配
  double height_iou = 0.0;
  if (!matchTargets(left_target, right_target, height_iou)) {
    RCLCPP_DEBUG(this->get_logger(), "Targets do not match (height IOU: %.3f). left=(id=%d,x=%.1f,y=%.1f,w=%.1f,h=%.1f,cls=%s), right=(id=%d,x=%.1f,y=%.1f,w=%.1f,h=%.1f,cls=%s)",
                 height_iou,
                 left_target.id, left_target.x, left_target.y, left_target.width, left_target.height, left_target.class_name.c_str(),
                 right_target.id, right_target.x, right_target.y, right_target.width, right_target.height, right_target.class_name.c_str());
    return;
  }

  // 计算3D位置
  rm_interfaces::msg::Target3D target_3d;
  if (calculateDistance(left_target, right_target, target_3d)) {
    // 发布3D目标
    rm_interfaces::msg::Target3DArray target3d_array;
    target3d_array.header = left_targets->header;
    target3d_array.header.frame_id = "camera_left_optical_frame";  // 以左相机为参考坐标系
    target3d_array.targets.push_back(target_3d);
    
    target3d_pub_->publish(target3d_array);
    
    RCLCPP_INFO(this->get_logger(), 
                "Published 3D target: distance=%.3f m, position=(%.3f, %.3f, %.3f), IOU=%.3f",
                target_3d.distance, 
                target_3d.position.x, target_3d.position.y, target_3d.position.z,
                height_iou);
  }
  else {
    RCLCPP_WARN(this->get_logger(), "calculateDistance failed for matched targets: left.x=%.2f,right.x=%.2f,fx=%.2f,baseline=%.3f",
                left_target.x, right_target.x, fx_, baseline_);
  }
}

bool StereoYoloDistanceNode::matchTargets(
  const rm_interfaces::msg::Target2D & left_target,
  const rm_interfaces::msg::Target2D & right_target,
  double & height_iou)
{
  // 计算高度IOU
  height_iou = calculateHeightIOU(
    left_target.y, left_target.height,
    right_target.y, right_target.height);

  // 检查是否满足IOU阈值
  if (height_iou < min_height_iou_) {
    return false;
  }

  // 可选：检查类别是否一致
  if (left_target.class_name != right_target.class_name) {
    RCLCPP_DEBUG(this->get_logger(), "Class mismatch: %s vs %s",
                 left_target.class_name.c_str(), right_target.class_name.c_str());
    return false;
  }

  // 检查视差是否合理（左相机的x应该大于右相机的x）
  double disparity = left_target.x - right_target.x;
  if (disparity < min_disparity_ || disparity > max_disparity_) {
    RCLCPP_DEBUG(this->get_logger(), "Invalid disparity: %.2f (range: [%.1f, %.1f])",
                 disparity, min_disparity_, max_disparity_);
    return false;
  }

  return true;
}

double StereoYoloDistanceNode::calculateHeightIOU(
  float left_y, float left_height,
  float right_y, float right_height)
{
  // 计算上下边界
  float left_top = left_y - left_height / 2.0f;
  float left_bottom = left_y + left_height / 2.0f;
  float right_top = right_y - right_height / 2.0f;
  float right_bottom = right_y + right_height / 2.0f;

  // 计算交集
  float intersection_top = std::max(left_top, right_top);
  float intersection_bottom = std::min(left_bottom, right_bottom);
  float intersection_height = std::max(0.0f, intersection_bottom - intersection_top);

  // 计算并集
  float union_height = left_height + right_height - intersection_height;

  // 避免除零
  if (union_height <= 0) {
    return 0.0;
  }

  return static_cast<double>(intersection_height) / union_height;
}

bool StereoYoloDistanceNode::calculateDistance(
  const rm_interfaces::msg::Target2D & left_target,
  const rm_interfaces::msg::Target2D & right_target,
  rm_interfaces::msg::Target3D & target_3d)
{
  // 计算视差
  double disparity = left_target.x - right_target.x;

  // 检查视差有效性
  if (disparity <= 0) {
    RCLCPP_WARN(this->get_logger(), "Invalid disparity: %.2f", disparity);
    return false;
  }

  // 计算深度: Z = (fx * baseline) / disparity
  double depth = (fx_ * baseline_) / disparity;

  // 检查距离是否在有效范围内
  if (depth < min_distance_ || depth > max_distance_) {
    RCLCPP_DEBUG(this->get_logger(), "Distance out of range: %.3f m (range: [%.2f, %.2f])",
                 depth, min_distance_, max_distance_);
    return false;
  }

  // 使用左相机坐标系计算3D坐标
  // 假设相机内参中心点在图像中心
  double cx = 640.0;  // 可以从camera_info获取
  double cy = 512.0;  // 可以从camera_info获取
  
  // 从camera_info获取主点
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    if (left_camera_info_ && !left_camera_info_->p.empty()) {
      cx = left_camera_info_->p[2];  // P[0,2]
      cy = left_camera_info_->p[6];  // P[1,2]
    }
  }

  // 使用针孔相机模型计算3D坐标
  // X = (u - cx) * Z / fx
  // Y = (v - cy) * Z / fy (假设fy ≈ fx)
  double x = (left_target.x - cx) * depth / fx_;
  double y = (left_target.y - cy) * depth / fx_;  // 使用fx作为近似
  double z = depth;

  // 填充Target3D消息
  target_3d.header = left_target.header;
  target_3d.position.x = x;
  target_3d.position.y = y;
  target_3d.position.z = z;
  target_3d.distance = static_cast<float>(depth);
  target_3d.confidence = std::min(left_target.confidence, right_target.confidence);
  target_3d.class_name = left_target.class_name;
  target_3d.id = left_target.id;
  target_3d.is_filtered = false;

  return true;
}

}  // namespace stereo_yolo_distance

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stereo_yolo_distance::StereoYoloDistanceNode)
