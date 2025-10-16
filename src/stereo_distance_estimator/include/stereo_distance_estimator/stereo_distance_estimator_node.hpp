#ifndef STEREO_DISTANCE_ESTIMATOR__STEREO_DISTANCE_ESTIMATOR_NODE_HPP_
#define STEREO_DISTANCE_ESTIMATOR__STEREO_DISTANCE_ESTIMATOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rm_interfaces/msg/target2_d_array.hpp>
#include <rm_interfaces/msg/target3_d_array.hpp>
#include <rm_interfaces/msg/target3_d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <mutex>

namespace stereo_distance_estimator
{

class StereoDistanceEstimatorNode : public rclcpp::Node
{
public:
  explicit StereoDistanceEstimatorNode(const rclcpp::NodeOptions & options);

private:
  // 回调函数
  void target2dCallback(const rm_interfaces::msg::Target2DArray::ConstSharedPtr & msg);
  void disparityCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);

  // 从点云中获取指定像素的3D坐标
  bool get3DPointFromCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
    int x, int y,
    geometry_msgs::msg::Point & point);

  // 从视差图计算3D坐标（备选方法）
  bool get3DPointFromDisparity(
    const sensor_msgs::msg::Image::ConstSharedPtr & disparity_msg,
    int x, int y,
    geometry_msgs::msg::Point & point);

  // 计算点到相机原点的距离
  float calculateDistance(const geometry_msgs::msg::Point & point);

  // 订阅器
  rclcpp::Subscription<rm_interfaces::msg::Target2DArray>::SharedPtr target2d_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr disparity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  // 发布器
  rclcpp::Publisher<rm_interfaces::msg::Target3DArray>::SharedPtr target3d_pub_;

  // 缓存最新的消息
  sensor_msgs::msg::Image::ConstSharedPtr latest_disparity_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_pointcloud_;
  std::mutex disparity_mutex_;
  std::mutex pointcloud_mutex_;

  // 参数
  std::string target2d_topic_;
  std::string disparity_topic_;
  std::string pointcloud_topic_;
  std::string target3d_topic_;
  int queue_size_;
  bool use_pointcloud_;  // true: 使用点云, false: 使用视差图
  double max_distance_;  // 最大有效距离 (m)
  double min_distance_;  // 最小有效距离 (m)

  // 相机内参（用于视差图反投影）
  double fx_, fy_;  // 焦距
  double cx_, cy_;  // 主点
  double baseline_;  // 基线距离 (m)
};

}  // namespace stereo_distance_estimator

#endif  // STEREO_DISTANCE_ESTIMATOR__STEREO_DISTANCE_ESTIMATOR_NODE_HPP_
