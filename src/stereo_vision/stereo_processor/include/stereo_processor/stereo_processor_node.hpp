#ifndef STEREO_PROCESSOR__STEREO_PROCESSOR_NODE_HPP_
#define STEREO_PROCESSOR__STEREO_PROCESSOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/stereo_camera_model.h>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace stereo_processor
{

class StereoProcessorNode : public rclcpp::Node
{
public:
  explicit StereoProcessorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~StereoProcessorNode() = default;

private:
  // 回调函数
  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & left_image_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & right_image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & left_info_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & right_info_msg);

  // 立体校正
  void rectifyImages(
    const cv::Mat & left_raw,
    const cv::Mat & right_raw,
    cv::Mat & left_rect,
    cv::Mat & right_rect);

  // 计算视差图
  void computeDisparity(
    const cv::Mat & left_rect,
    const cv::Mat & right_rect,
    cv::Mat & disparity);

  // 生成点云
  void generatePointCloud(
    const cv::Mat & disparity,
    const cv::Mat & left_rect,
    sensor_msgs::msg::PointCloud2 & pointcloud_msg);

  // 发布视差图消息
  void publishDisparityImage(
    const cv::Mat & disparity,
    const std_msgs::msg::Header & header);

  // 订阅器 (使用 message_filters 进行时间同步)
  message_filters::Subscriber<sensor_msgs::msg::Image> left_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> right_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> left_info_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> right_info_sub_;

  // 时间同步策略
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo,
    sensor_msgs::msg::CameraInfo> ApproxSyncPolicy;
  
  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_;

  // 发布器
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_rect_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_rect_pub_;
  rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr disparity_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

  // 立体相机模型
  image_geometry::StereoCameraModel stereo_model_;
  bool camera_model_initialized_;

  // 立体匹配器
  cv::Ptr<cv::StereoSGBM> stereo_matcher_;
  cv::Mat Q_;  // 重投影矩阵

  // 参数
  std::string stereo_algorithm_;
  int min_disparity_;
  int num_disparities_;
  int block_size_;
  int uniqueness_ratio_;
  int speckle_window_size_;
  int speckle_range_;
  int disp12_max_diff_;
  int prefilter_cap_;
  int prefilter_size_;
  int texture_threshold_;
  bool use_color_;

  // 校正映射
  cv::Mat left_map1_, left_map2_;
  cv::Mat right_map1_, right_map2_;
  bool rectification_maps_initialized_;
};

}  // namespace stereo_processor

#endif  // STEREO_PROCESSOR__STEREO_PROCESSOR_NODE_HPP_
