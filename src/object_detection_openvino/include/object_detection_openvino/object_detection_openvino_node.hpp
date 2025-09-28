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
    
    // OpenVINO inference engine
    std::unique_ptr<ROS2OpenvinoInfer> openvino_infer_;
    
    // Parameters
    std::string mode_;
    int input_width_;
    int input_height_;
    double score_threshold_;
    double nms_threshold_;
    bool startup_;
    
    // Model paths
    std::string xml_path_;
    std::string bin_path_;
    std::string device_;
    std::string image_topic_;
    std::string detection_topic_;
    
    void initializeParameters();
    void loadModel();
    rm_interfaces::msg::Target2DArray convertToRosMessage(
        const std::vector<ROS2OpenvinoInfer::Light>& detections,
        const std_msgs::msg::Header& header);
};

#endif // OBJECT_DETECTION_OPENVINO_NODE_HPP_