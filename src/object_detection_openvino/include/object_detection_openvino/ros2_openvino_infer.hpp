#ifndef ROS2_OPENVINO_INFER_HPP_
#define ROS2_OPENVINO_INFER_HPP_

#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <string>

class ROS2OpenvinoInfer {
public:
    struct Resize {
        cv::Mat resized_image;
        int dw;
        int dh;
    };

    struct Light {
        int id = -1;
        double score = 0.;
        cv::Rect2d box;
        cv::Point2d center_point;
    };

    explicit ROS2OpenvinoInfer(
        const std::map<std::string, std::string>& path,
        double score_threshold = 0.5,
        double nms_threshold = 0.4
    );
    
    ~ROS2OpenvinoInfer() = default;

    std::vector<Light> infer(cv::Mat& src, const cv::Size2d& dst_size, const int& my_color, const bool& startup);

private:
    static Resize letterBox(cv::Mat& src, const cv::Size2d& dst_size);
    void fitRec(std::vector<Light>& bboxes, cv::Size2d ori_size, cv::Size2d now_size);

    ov::Core core_;
    ov::CompiledModel compiled_model_;
    ov::CompiledModel compiled_model_next_;
    std::shared_ptr<ov::Model> model_;
    std::map<bool, ov::InferRequest> infer_requests_;
    
    // Configuration parameters
    double score_threshold_;
    double nms_threshold_;
};

#endif // ROS2_OPENVINO_INFER_HPP_