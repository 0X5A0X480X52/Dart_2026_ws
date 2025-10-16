#ifndef RESIZE_IMAGE_RAW__RESIZE_NODE_HPP_
#define RESIZE_IMAGE_RAW__RESIZE_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include <opencv2/opencv.hpp>

namespace resize_image_raw
{

class ResizeNode : public rclcpp::Node
{
public:
  explicit ResizeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg);
  
  sensor_msgs::msg::CameraInfo scaleCameraInfo(
    const sensor_msgs::msg::CameraInfo & input_info,
    double scale_width,
    double scale_height);

  // Subscribers
  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Publishers
  image_transport::Publisher image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  // Parameters
  double scale_width_;
  double scale_height_;
  int interpolation_;
  
  // Topic names
  std::string input_image_topic_;
  std::string input_camera_info_topic_;
  std::string output_image_topic_;
  std::string output_camera_info_topic_;

  // Image transport
  std::shared_ptr<image_transport::ImageTransport> it_;

  // Store latest camera info
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info_;
  bool camera_info_received_;
};

}  // namespace resize_image_raw

#endif  // RESIZE_IMAGE_RAW__RESIZE_NODE_HPP_
