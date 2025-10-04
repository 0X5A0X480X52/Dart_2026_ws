#ifndef STEREO_DISTANCE_ESTIMATOR__STEREO_DISTANCE_ESTIMATOR_NODE_HPP_
#define STEREO_DISTANCE_ESTIMATOR__STEREO_DISTANCE_ESTIMATOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rm_interfaces/msg/target2_d_array.hpp>
#include <rm_interfaces/msg/target3_d_array.hpp>
#include <rm_interfaces/msg/target3_d.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace stereo_distance_estimator
{

class StereoDistanceEstimatorNode : public rclcpp::Node
{
public:
  explicit StereoDistanceEstimatorNode(const rclcpp::NodeOptions & options);

private:
  // 回调函数：同步处理 Target2DArray、Disparity 和 PointCloud2
  void syncCallback(
    const rm_interfaces::msg::Target2DArray::ConstSharedPtr & targets_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & disparity_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg);

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
  message_filters::Subscriber<rm_interfaces::msg::Target2DArray> target2d_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> disparity_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pointcloud_sub_;

  // 同步器
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    rm_interfaces::msg::Target2DArray,
    sensor_msgs::msg::Image,
    sensor_msgs::msg::PointCloud2>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // 发布器
  rclcpp::Publisher<rm_interfaces::msg::Target3DArray>::SharedPtr target3d_pub_;

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
