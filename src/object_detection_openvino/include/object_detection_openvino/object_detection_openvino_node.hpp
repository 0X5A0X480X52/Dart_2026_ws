#ifndef OBJECT_DETECTION_OPENVINO_NODE_HPP_
#define OBJECT_DETECTION_OPENVINO_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rm_interfaces/msg/target2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "object_detection_openvino/ros2_openvino_infer.hpp"

class ObjectDetectionOpenvinoNode : public rclcpp::Node
{
public:
    ObjectDetectionOpenvinoNode();
    ~ObjectDetectionOpenvinoNode();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    // ROS2 publishers and subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<rm_interfaces::msg::Target2DArray>::SharedPtr target_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_publisher_;
    
    // OpenVINO inference engine
    std::unique_ptr<ROS2OpenvinoInfer> openvino_infer_;
    
    // Parameters
    std::string mode_;
    int input_width_;
    int input_height_;
    double score_threshold_;
    double nms_threshold_;
    bool startup_;
    
    // ROI parameters
    std::string roi_mode_;
    int roi_width_;
    int roi_height_;
    std::string center_mode_;  // "auto", "pixel", "percent", "fraction"
    double center_x_;  // Center X for ROI: -1 = auto center; [0..1] fraction; (0..100] percent; >100 pixels
    double center_y_;  // Center Y for ROI: -1 = auto center; [0..1] fraction; (0..100] percent; >100 pixels
    cv::Rect roi_rect_;  // Current ROI rectangle in original image coordinates
    
    // Model paths
    std::string xml_path_;
    std::string bin_path_;
    std::string device_;
    std::string image_topic_;
    std::string detection_topic_;
    std::string debug_image_topic_;
    bool publish_debug_image_;
    
    void initializeParameters();
    void loadModel();
    rm_interfaces::msg::Target2DArray convertToRosMessage(
        const std::vector<ROS2OpenvinoInfer::Light>& detections,
        const std_msgs::msg::Header& header);
    void drawDebugImage(
        cv::Mat& image,
        const std::vector<ROS2OpenvinoInfer::Light>& detections,
        const cv::Rect& roi_rect);
};

#endif // OBJECT_DETECTION_OPENVINO_NODE_HPP_