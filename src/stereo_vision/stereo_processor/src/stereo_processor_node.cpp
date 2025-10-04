#include "stereo_processor/stereo_processor_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace stereo_processor
{

StereoProcessorNode::StereoProcessorNode(const rclcpp::NodeOptions & options)
: Node("stereo_processor", options),
  camera_model_initialized_(false),
  rectification_maps_initialized_(false)
{
  // 声明参数
  this->declare_parameter<std::string>("stereo_algorithm", "sgbm");
  this->declare_parameter<int>("min_disparity", 0);
  this->declare_parameter<int>("num_disparities", 128);
  this->declare_parameter<int>("block_size", 15);
  this->declare_parameter<int>("uniqueness_ratio", 10);
  this->declare_parameter<int>("speckle_window_size", 100);
  this->declare_parameter<int>("speckle_range", 4);
  this->declare_parameter<int>("disp12_max_diff", 1);
  this->declare_parameter<int>("prefilter_cap", 31);
  this->declare_parameter<int>("prefilter_size", 5);
  this->declare_parameter<int>("texture_threshold", 10);
  this->declare_parameter<bool>("use_color", true);
  
  // 获取参数
  this->get_parameter("stereo_algorithm", stereo_algorithm_);
  this->get_parameter("min_disparity", min_disparity_);
  this->get_parameter("num_disparities", num_disparities_);
  this->get_parameter("block_size", block_size_);
  this->get_parameter("uniqueness_ratio", uniqueness_ratio_);
  this->get_parameter("speckle_window_size", speckle_window_size_);
  this->get_parameter("speckle_range", speckle_range_);
  this->get_parameter("disp12_max_diff", disp12_max_diff_);
  this->get_parameter("prefilter_cap", prefilter_cap_);
  this->get_parameter("prefilter_size", prefilter_size_);
  this->get_parameter("texture_threshold", texture_threshold_);
  this->get_parameter("use_color", use_color_);

  // 确保 num_disparities 是 16 的倍数
  if (num_disparities_ % 16 != 0) {
    num_disparities_ = ((num_disparities_ / 16) + 1) * 16;
    RCLCPP_WARN(this->get_logger(), "num_disparities adjusted to %d (must be multiple of 16)", num_disparities_);
  }

  // 确保 block_size 是奇数
  if (block_size_ % 2 == 0) {
    block_size_ += 1;
    RCLCPP_WARN(this->get_logger(), "block_size adjusted to %d (must be odd)", block_size_);
  }

  // 创建 SGBM 立体匹配器
  stereo_matcher_ = cv::StereoSGBM::create(
    min_disparity_,
    num_disparities_,
    block_size_,
    8 * block_size_ * block_size_,   // P1
    32 * block_size_ * block_size_,  // P2
    disp12_max_diff_,
    prefilter_cap_,
    uniqueness_ratio_,
    speckle_window_size_,
    speckle_range_,
    cv::StereoSGBM::MODE_SGBM_3WAY
  );

  RCLCPP_INFO(this->get_logger(), "Stereo matcher initialized with algorithm: %s", stereo_algorithm_.c_str());
  RCLCPP_INFO(this->get_logger(), "  min_disparity: %d", min_disparity_);
  RCLCPP_INFO(this->get_logger(), "  num_disparities: %d", num_disparities_);
  RCLCPP_INFO(this->get_logger(), "  block_size: %d", block_size_);

  // 创建订阅器
  left_image_sub_.subscribe(this, "/camera/left/image_raw");
  right_image_sub_.subscribe(this, "/camera/right/image_raw");
  left_info_sub_.subscribe(this, "/camera/left/camera_info");
  right_info_sub_.subscribe(this, "/camera/right/camera_info");

  // 创建时间同步器
  sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
    ApproxSyncPolicy(10),
    left_image_sub_,
    right_image_sub_,
    left_info_sub_,
    right_info_sub_
  );

  sync_->registerCallback(
    std::bind(
      &StereoProcessorNode::imageCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3,
      std::placeholders::_4
    )
  );

  // 创建发布器
  left_rect_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/camera/left/image_rect", 10);
  right_rect_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/camera/right/image_rect", 10);
  disparity_pub_ = this->create_publisher<stereo_msgs::msg::DisparityImage>(
    "/stereo/disparity", 10);
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/stereo/points2", 10);

  RCLCPP_INFO(this->get_logger(), "Stereo Processor Node initialized successfully");
  RCLCPP_INFO(this->get_logger(), "Subscribed to:");
  RCLCPP_INFO(this->get_logger(), "  - /camera/left/image_raw");
  RCLCPP_INFO(this->get_logger(), "  - /camera/right/image_raw");
  RCLCPP_INFO(this->get_logger(), "  - /camera/left/camera_info");
  RCLCPP_INFO(this->get_logger(), "  - /camera/right/camera_info");
  RCLCPP_INFO(this->get_logger(), "Publishing to:");
  RCLCPP_INFO(this->get_logger(), "  - /camera/left/image_rect");
  RCLCPP_INFO(this->get_logger(), "  - /camera/right/image_rect");
  RCLCPP_INFO(this->get_logger(), "  - /stereo/disparity");
  RCLCPP_INFO(this->get_logger(), "  - /stereo/points2");
}

void StereoProcessorNode::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & left_image_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & right_image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & left_info_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & right_info_msg)
{
  // 初始化相机模型
  if (!camera_model_initialized_) {
    stereo_model_.fromCameraInfo(*left_info_msg, *right_info_msg);
    camera_model_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Stereo camera model initialized");
  }

  // 转换图像消息到 OpenCV 格式
  cv_bridge::CvImageConstPtr left_cv_ptr, right_cv_ptr;
  try {
    left_cv_ptr = cv_bridge::toCvShare(left_image_msg, sensor_msgs::image_encodings::BGR8);
    right_cv_ptr = cv_bridge::toCvShare(right_image_msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // 立体校正
  cv::Mat left_rect, right_rect;
  rectifyImages(left_cv_ptr->image, right_cv_ptr->image, left_rect, right_rect);

  // 发布校正后的图像
  if (left_rect_pub_->get_subscription_count() > 0) {
    cv_bridge::CvImage left_rect_msg;
    left_rect_msg.header = left_image_msg->header;
    left_rect_msg.encoding = sensor_msgs::image_encodings::BGR8;
    left_rect_msg.image = left_rect;
    left_rect_pub_->publish(*left_rect_msg.toImageMsg());
  }

  if (right_rect_pub_->get_subscription_count() > 0) {
    cv_bridge::CvImage right_rect_msg;
    right_rect_msg.header = right_image_msg->header;
    right_rect_msg.encoding = sensor_msgs::image_encodings::BGR8;
    right_rect_msg.image = right_rect;
    right_rect_pub_->publish(*right_rect_msg.toImageMsg());
  }

  // 计算视差图
  cv::Mat disparity;
  computeDisparity(left_rect, right_rect, disparity);

  // 发布视差图
  if (disparity_pub_->get_subscription_count() > 0) {
    publishDisparityImage(disparity, left_image_msg->header);
  }

  // 生成并发布点云
  if (pointcloud_pub_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    generatePointCloud(disparity, left_rect, pointcloud_msg);
    pointcloud_msg.header = left_image_msg->header;
    pointcloud_msg.header.frame_id = "stereo_camera_frame";
    pointcloud_pub_->publish(pointcloud_msg);
  }
}

void StereoProcessorNode::rectifyImages(
  const cv::Mat & left_raw,
  const cv::Mat & right_raw,
  cv::Mat & left_rect,
  cv::Mat & right_rect)
{
  // 初始化校正映射
  if (!rectification_maps_initialized_ && camera_model_initialized_) {
    cv::Mat R1, R2, P1, P2;
    stereo_model_.left().rectifyImage(left_raw, left_rect, cv::INTER_LINEAR);
    stereo_model_.right().rectifyImage(right_raw, right_rect, cv::INTER_LINEAR);
    
    // 获取重投影矩阵并转换为 cv::Mat
    Q_ = cv::Mat(stereo_model_.reprojectionMatrix());
    
    rectification_maps_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Rectification maps initialized");
  } else if (camera_model_initialized_) {
    stereo_model_.left().rectifyImage(left_raw, left_rect, cv::INTER_LINEAR);
    stereo_model_.right().rectifyImage(right_raw, right_rect, cv::INTER_LINEAR);
  } else {
    // 如果相机模型未初始化，直接复制图像
    left_rect = left_raw.clone();
    right_rect = right_raw.clone();
  }
}

void StereoProcessorNode::computeDisparity(
  const cv::Mat & left_rect,
  const cv::Mat & right_rect,
  cv::Mat & disparity)
{
  // 转换为灰度图
  cv::Mat left_gray, right_gray;
  cv::cvtColor(left_rect, left_gray, cv::COLOR_BGR2GRAY);
  cv::cvtColor(right_rect, right_gray, cv::COLOR_BGR2GRAY);

  // 计算视差
  cv::Mat disparity_16s;
  stereo_matcher_->compute(left_gray, right_gray, disparity_16s);

  // 转换为浮点型并归一化
  disparity_16s.convertTo(disparity, CV_32F, 1.0 / 16.0);
}

void StereoProcessorNode::generatePointCloud(
  const cv::Mat & disparity,
  const cv::Mat & left_rect,
  sensor_msgs::msg::PointCloud2 & pointcloud_msg)
{
  if (Q_.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Reprojection matrix Q is not initialized, cannot generate point cloud");
    return;
  }

  // 重投影到3D
  cv::Mat points3d;
  cv::reprojectImageTo3D(disparity, points3d, Q_, true);

  // 创建 PointCloud2 消息
  pointcloud_msg.height = disparity.rows;
  pointcloud_msg.width = disparity.cols;
  pointcloud_msg.is_dense = false;
  pointcloud_msg.is_bigendian = false;

  // 定义字段
  sensor_msgs::PointCloud2Modifier modifier(pointcloud_msg);
  if (use_color_) {
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  } else {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  }

  // 填充点云数据
  sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud_msg, "z");
  
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(pointcloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(pointcloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(pointcloud_msg, "b");

  for (int v = 0; v < disparity.rows; ++v) {
    for (int u = 0; u < disparity.cols; ++u) {
      const cv::Vec3f & point = points3d.at<cv::Vec3f>(v, u);
      
      // 过滤无效点
      if (std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2])) {
        *iter_x = point[0] / 1000.0f;  // 转换为米
        *iter_y = point[1] / 1000.0f;
        *iter_z = point[2] / 1000.0f;

        if (use_color_) {
          const cv::Vec3b & color = left_rect.at<cv::Vec3b>(v, u);
          *iter_r = color[2];  // OpenCV uses BGR
          *iter_g = color[1];
          *iter_b = color[0];
        }
      } else {
        *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        if (use_color_) {
          *iter_r = *iter_g = *iter_b = 0;
        }
      }

      ++iter_x; ++iter_y; ++iter_z;
      if (use_color_) {
        ++iter_r; ++iter_g; ++iter_b;
      }
    }
  }
}

void StereoProcessorNode::publishDisparityImage(
  const cv::Mat & disparity,
  const std_msgs::msg::Header & header)
{
  stereo_msgs::msg::DisparityImage disparity_msg;
  disparity_msg.header = header;
  
  // 填充视差图信息
  disparity_msg.f = stereo_model_.right().fx();
  disparity_msg.t = stereo_model_.baseline();
  disparity_msg.min_disparity = min_disparity_;
  disparity_msg.max_disparity = min_disparity_ + num_disparities_;
  disparity_msg.delta_d = 1.0 / 16.0;  // SGBM 使用 16 位子像素精度

  // 转换视差图为图像消息
  cv_bridge::CvImage disparity_image;
  disparity_image.header = header;
  disparity_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  disparity_image.image = disparity;
  
  disparity_msg.image = *disparity_image.toImageMsg();
  disparity_pub_->publish(disparity_msg);
}

}  // namespace stereo_processor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<stereo_processor::StereoProcessorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
