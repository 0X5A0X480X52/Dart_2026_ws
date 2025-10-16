#include "resize_image_raw/resize_node.hpp"

namespace resize_image_raw
{

ResizeNode::ResizeNode(const rclcpp::NodeOptions & options)
: Node("resize_node", options),
  camera_info_received_(false)
{
  // Declare parameters
  this->declare_parameter("scale_width", 1.0);
  this->declare_parameter("scale_height", 1.0);
  this->declare_parameter("interpolation", 1);  // 1 = LINEAR, 0 = NEAREST, 2 = CUBIC, 3 = AREA
  
  // Declare topic name parameters
  this->declare_parameter("input_image_topic", "image");
  this->declare_parameter("input_camera_info_topic", "camera_info");
  this->declare_parameter("output_image_topic", "resized/image");
  this->declare_parameter("output_camera_info_topic", "resized/camera_info");

  // Get parameters
  scale_width_ = this->get_parameter("scale_width").as_double();
  scale_height_ = this->get_parameter("scale_height").as_double();
  interpolation_ = this->get_parameter("interpolation").as_int();
  
  // Get topic name parameters
  input_image_topic_ = this->get_parameter("input_image_topic").as_string();
  input_camera_info_topic_ = this->get_parameter("input_camera_info_topic").as_string();
  output_image_topic_ = this->get_parameter("output_image_topic").as_string();
  output_camera_info_topic_ = this->get_parameter("output_camera_info_topic").as_string();

  RCLCPP_INFO(this->get_logger(), 
    "Resize node initialized with scale_width=%.2f, scale_height=%.2f, interpolation=%d",
    scale_width_, scale_height_, interpolation_);
  
  RCLCPP_INFO(this->get_logger(),
    "Topic configuration:\n"
    "  Input image: %s\n"
    "  Input camera_info: %s\n"
    "  Output image: %s\n"
    "  Output camera_info: %s",
    input_image_topic_.c_str(),
    input_camera_info_topic_.c_str(),
    output_image_topic_.c_str(),
    output_camera_info_topic_.c_str());

  // Validate parameters
  if (scale_width_ <= 0.0 || scale_height_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Scale factors must be positive!");
    throw std::invalid_argument("Invalid scale factors");
  }
}

void ResizeNode::initialize()
{
  // Initialize image transport (must be called after shared_from_this() is safe)
  it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());

  // Create subscribers with configured topic names
  image_sub_ = it_->subscribe(
    input_image_topic_, 
    10,
    std::bind(&ResizeNode::imageCallback, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    input_camera_info_topic_,
    10,
    std::bind(&ResizeNode::cameraInfoCallback, this, std::placeholders::_1));

  // Create publishers with configured topic names
  image_pub_ = it_->advertise(output_image_topic_, 10);
  
  camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
    output_camera_info_topic_,
    10);

  RCLCPP_INFO(this->get_logger(), "Resize node started. Waiting for images...");
}

void ResizeNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{
  try {
    // Convert ROS image to OpenCV
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
      image_msg, 
      image_msg->encoding);

    // Calculate new dimensions
    int new_width = static_cast<int>(cv_ptr->image.cols * scale_width_);
    int new_height = static_cast<int>(cv_ptr->image.rows * scale_height_);

    // Ensure dimensions are at least 1x1
    new_width = std::max(1, new_width);
    new_height = std::max(1, new_height);

    // Determine interpolation method
    int cv_interpolation;
    switch (interpolation_) {
      case 0:
        cv_interpolation = cv::INTER_NEAREST;
        break;
      case 2:
        cv_interpolation = cv::INTER_CUBIC;
        break;
      case 3:
        cv_interpolation = cv::INTER_AREA;
        break;
      case 1:
      default:
        cv_interpolation = cv::INTER_LINEAR;
        break;
    }

    // Resize the image
    cv::Mat resized_image;
    cv::resize(cv_ptr->image, resized_image, cv::Size(new_width, new_height), 
               0, 0, cv_interpolation);

    // Create output message
    cv_bridge::CvImage out_msg;
    out_msg.header = image_msg->header;
    out_msg.encoding = image_msg->encoding;
    out_msg.image = resized_image;

    // Publish resized image
    image_pub_.publish(out_msg.toImageMsg());

    // If we have camera info, scale and publish it
    if (camera_info_received_ && latest_camera_info_) {
      auto scaled_info = scaleCameraInfo(*latest_camera_info_, scale_width_, scale_height_);
      scaled_info.header = image_msg->header;
      camera_info_pub_->publish(scaled_info);
    }

  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (cv::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
  }
}

void ResizeNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  latest_camera_info_ = info_msg;
  camera_info_received_ = true;
}

sensor_msgs::msg::CameraInfo ResizeNode::scaleCameraInfo(
  const sensor_msgs::msg::CameraInfo & input_info,
  double scale_width,
  double scale_height)
{
  sensor_msgs::msg::CameraInfo output_info = input_info;

  // Scale image dimensions
  output_info.width = static_cast<uint32_t>(input_info.width * scale_width);
  output_info.height = static_cast<uint32_t>(input_info.height * scale_height);

  // Ensure dimensions are at least 1x1
  output_info.width = std::max(1u, output_info.width);
  output_info.height = std::max(1u, output_info.height);

  // Scale camera matrix K
  // K = [fx  0  cx]
  //     [ 0 fy  cy]
  //     [ 0  0   1]
  output_info.k[0] = input_info.k[0] * scale_width;   // fx
  output_info.k[2] = input_info.k[2] * scale_width;   // cx
  output_info.k[4] = input_info.k[4] * scale_height;  // fy
  output_info.k[5] = input_info.k[5] * scale_height;  // cy

  // Scale projection matrix P
  // P = [fx'  0  cx' Tx]
  //     [ 0  fy' cy' Ty]
  //     [ 0   0   1   0]
  output_info.p[0] = input_info.p[0] * scale_width;   // fx'
  output_info.p[2] = input_info.p[2] * scale_width;   // cx'
  output_info.p[5] = input_info.p[5] * scale_height;  // fy'
  output_info.p[6] = input_info.p[6] * scale_height;  // cy'
  output_info.p[3] = input_info.p[3] * scale_width;   // Tx (for stereo)

  // ROI (Region of Interest) scaling
  if (input_info.roi.width > 0 && input_info.roi.height > 0) {
    output_info.roi.x_offset = static_cast<uint32_t>(input_info.roi.x_offset * scale_width);
    output_info.roi.y_offset = static_cast<uint32_t>(input_info.roi.y_offset * scale_height);
    output_info.roi.width = static_cast<uint32_t>(input_info.roi.width * scale_width);
    output_info.roi.height = static_cast<uint32_t>(input_info.roi.height * scale_height);
  }

  return output_info;
}

}  // namespace resize_image_raw
