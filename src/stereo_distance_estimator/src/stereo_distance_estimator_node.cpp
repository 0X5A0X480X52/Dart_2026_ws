#include "stereo_distance_estimator/stereo_distance_estimator_node.hpp"
#include <cmath>

namespace stereo_distance_estimator
{

StereoDistanceEstimatorNode::StereoDistanceEstimatorNode(const rclcpp::NodeOptions & options)
: Node("stereo_distance_estimator", options)
{
  // 声明并获取参数
  this->declare_parameter("target2d_topic", "/filter/target2d_array");
  this->declare_parameter("disparity_topic", "/stereo/disparity");
  this->declare_parameter("pointcloud_topic", "/stereo/points2");
  this->declare_parameter("target3d_topic", "/stereo/target3d_array_raw");
  this->declare_parameter("queue_size", 10);
  this->declare_parameter("use_pointcloud", true);
  this->declare_parameter("max_distance", 10.0);
  this->declare_parameter("min_distance", 0.1);
  
  // 相机内参（如果使用视差图）
  this->declare_parameter("fx", 600.0);
  this->declare_parameter("fy", 600.0);
  this->declare_parameter("cx", 320.0);
  this->declare_parameter("cy", 240.0);
  this->declare_parameter("baseline", 0.12);  // 12cm 基线

  target2d_topic_ = this->get_parameter("target2d_topic").as_string();
  disparity_topic_ = this->get_parameter("disparity_topic").as_string();
  pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
  target3d_topic_ = this->get_parameter("target3d_topic").as_string();
  queue_size_ = this->get_parameter("queue_size").as_int();
  use_pointcloud_ = this->get_parameter("use_pointcloud").as_bool();
  max_distance_ = this->get_parameter("max_distance").as_double();
  min_distance_ = this->get_parameter("min_distance").as_double();
  
  fx_ = this->get_parameter("fx").as_double();
  fy_ = this->get_parameter("fy").as_double();
  cx_ = this->get_parameter("cx").as_double();
  cy_ = this->get_parameter("cy").as_double();
  baseline_ = this->get_parameter("baseline").as_double();

  // 创建订阅器
  target2d_sub_.subscribe(this, target2d_topic_);
  disparity_sub_.subscribe(this, disparity_topic_);
  pointcloud_sub_.subscribe(this, pointcloud_topic_);

  // 创建同步器
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(queue_size_), target2d_sub_, disparity_sub_, pointcloud_sub_);
  sync_->registerCallback(
    std::bind(&StereoDistanceEstimatorNode::syncCallback, this,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // 创建发布器
  target3d_pub_ = this->create_publisher<rm_interfaces::msg::Target3DArray>(
    target3d_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "Stereo Distance Estimator Node initialized");
  RCLCPP_INFO(this->get_logger(), "  Subscribing to:");
  RCLCPP_INFO(this->get_logger(), "    - %s", target2d_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "    - %s", disparity_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "    - %s", pointcloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Publishing to: %s", target3d_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Use pointcloud: %s", use_pointcloud_ ? "true" : "false");
}

void StereoDistanceEstimatorNode::syncCallback(
  const rm_interfaces::msg::Target2DArray::ConstSharedPtr & targets_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & disparity_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg)
{
  // 创建输出消息
  auto target3d_array = rm_interfaces::msg::Target3DArray();
  target3d_array.header = targets_msg->header;
  target3d_array.header.frame_id = "camera_link";  // 可配置

  // 遍历每个2D目标
  for (const auto & target2d : targets_msg->targets) {
    // 获取目标中心像素坐标
    int pixel_x = static_cast<int>(target2d.x);
    int pixel_y = static_cast<int>(target2d.y);

    geometry_msgs::msg::Point point_3d;
    bool success = false;

    // 根据配置选择使用点云或视差图
    if (use_pointcloud_) {
      success = get3DPointFromCloud(cloud_msg, pixel_x, pixel_y, point_3d);
    } else {
      success = get3DPointFromDisparity(disparity_msg, pixel_x, pixel_y, point_3d);
    }

    // 如果成功获取3D坐标
    if (success) {
      // 计算距离
      float distance = calculateDistance(point_3d);

      // 距离有效性检查
      if (distance >= min_distance_ && distance <= max_distance_ && std::isfinite(distance)) {
        // 创建3D目标
        rm_interfaces::msg::Target3D target3d;
        target3d.header = target3d_array.header;
        target3d.position = point_3d;
        target3d.distance = distance;
        target3d.confidence = target2d.confidence;
        target3d.class_name = target2d.class_name;
        target3d.id = target2d.id;
        target3d.is_filtered = false;  // raw 数据

        target3d_array.targets.push_back(target3d);
      } else {
        RCLCPP_DEBUG(this->get_logger(), 
          "Target at (%d, %d) has invalid distance: %.2f m", pixel_x, pixel_y, distance);
      }
    } else {
      RCLCPP_DEBUG(this->get_logger(), 
        "Failed to get 3D point for target at (%d, %d)", pixel_x, pixel_y);
    }
  }

  // 发布结果
  target3d_pub_->publish(target3d_array);
  
  RCLCPP_DEBUG(this->get_logger(), 
    "Processed %zu 2D targets -> %zu 3D targets", 
    targets_msg->targets.size(), target3d_array.targets.size());
}

bool StereoDistanceEstimatorNode::get3DPointFromCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
  int x, int y,
  geometry_msgs::msg::Point & point)
{
  // 检查坐标是否在点云范围内
  if (x < 0 || x >= static_cast<int>(cloud_msg->width) ||
      y < 0 || y >= static_cast<int>(cloud_msg->height))
  {
    RCLCPP_WARN(this->get_logger(), 
      "Pixel coordinates (%d, %d) out of cloud bounds (%u x %u)", 
      x, y, cloud_msg->width, cloud_msg->height);
    return false;
  }

  // 使用 PointCloud2Iterator 访问点云数据
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

  // 计算点在点云中的索引（row-major order）
  int index = y * cloud_msg->width + x;
  iter_x += index;
  iter_y += index;
  iter_z += index;

  // 提取 XYZ 坐标
  float px = *iter_x;
  float py = *iter_y;
  float pz = *iter_z;

  // 检查点是否有效（NaN 或 Inf）
  if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
    RCLCPP_DEBUG(this->get_logger(), 
      "Invalid point at (%d, %d): (%.2f, %.2f, %.2f)", x, y, px, py, pz);
    return false;
  }

  // 赋值输出
  point.x = px;
  point.y = py;
  point.z = pz;

  return true;
}

bool StereoDistanceEstimatorNode::get3DPointFromDisparity(
  const sensor_msgs::msg::Image::ConstSharedPtr & disparity_msg,
  int x, int y,
  geometry_msgs::msg::Point & point)
{
  // 检查坐标是否在视差图范围内
  if (x < 0 || x >= static_cast<int>(disparity_msg->width) ||
      y < 0 || y >= static_cast<int>(disparity_msg->height))
  {
    return false;
  }

  // 将 ROS Image 转换为 OpenCV Mat
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(disparity_msg, sensor_msgs::image_encodings::TYPE_32FC1);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return false;
  }

  // 获取视差值
  float disparity = cv_ptr->image.at<float>(y, x);

  // 检查视差是否有效
  if (disparity <= 0.0f || !std::isfinite(disparity)) {
    return false;
  }

  // 使用视差计算深度：Z = (fx * baseline) / disparity
  float Z = (fx_ * baseline_) / disparity;

  // 使用针孔相机模型计算 X, Y
  // X = (u - cx) * Z / fx
  // Y = (v - cy) * Z / fy
  float X = (x - cx_) * Z / fx_;
  float Y = (y - cy_) * Z / fy_;

  // 赋值输出
  point.x = X;
  point.y = Y;
  point.z = Z;

  return true;
}

float StereoDistanceEstimatorNode::calculateDistance(const geometry_msgs::msg::Point & point)
{
  // 计算欧氏距离
  return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

}  // namespace stereo_distance_estimator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stereo_distance_estimator::StereoDistanceEstimatorNode)
