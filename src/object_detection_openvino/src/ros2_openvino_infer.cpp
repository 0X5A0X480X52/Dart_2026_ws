#include "object_detection_openvino/ros2_openvino_infer.hpp"
#include <opencv2/dnn.hpp>
#include <cmath>

#define BLUE 0
#define RED  1
#define NONE_ 2
#define CURR true
#define NEXT false

ROS2OpenvinoInfer::ROS2OpenvinoInfer(
    const std::map<std::string, std::string>& path,
    const std::string& output_format,
    double score_threshold,
    double nms_threshold
) : output_format_(output_format), score_threshold_(score_threshold), nms_threshold_(nms_threshold)
{
    // Load model from XML and BIN files
    this->model_ = this->core_.read_model(path.at("XML"), path.at("BIN"));

    // Configure preprocessing
    ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(this->model_);

    ppp.input()
       .tensor()
       .set_element_type(ov::element::u8)
       .set_layout("NHWC")
       .set_color_format(ov::preprocess::ColorFormat::BGR);

    ppp.input()
       .preprocess()
       .convert_element_type(ov::element::f32)
       .convert_color(ov::preprocess::ColorFormat::RGB)
       .scale({255.f, 255.f, 255.f});

    ppp.input()
       .model()
       .set_layout("NCHW");

    // Configure output preprocessing for models with multiple outputs
    // Set output tensor type for all outputs
    for (size_t i = 0; i < this->model_->outputs().size(); i++) {
        ppp.output(i)
           .tensor()
           .set_element_type(ov::element::f32);
    }

    this->model_ = ppp.build();

    // Compile model for the specified device
    this->compiled_model_ = this->core_.compile_model(this->model_, path.at("DEVICE"));
    this->compiled_model_next_ = this->core_.compile_model(this->model_, path.at("DEVICE"));

    // Create inference requests
    this->infer_requests_[CURR] = this->compiled_model_.create_infer_request();
    this->infer_requests_[NEXT] = this->compiled_model_next_.create_infer_request();
}

std::vector<ROS2OpenvinoInfer::Light> ROS2OpenvinoInfer::infer(
    cv::Mat& src, 
    const cv::Size2d& dst_size, 
    const int& my_color, 
    const bool& startup)
{
    // Guard against config/model mismatch. The provided resize target must
    // exactly match model input shape, otherwise input tensor memory is invalid.
    const ov::Shape model_input_shape = this->compiled_model_.input().get_shape();
    if (model_input_shape.size() != 4) {
        throw std::runtime_error("Unsupported model input rank: expected NCHW (4D)");
    }

    int model_input_h = -1;
    int model_input_w = -1;

    // The compiled input may be NCHW ([N,C,H,W]) or NHWC ([N,H,W,C])
    // depending on preprocessing configuration.
    if (model_input_shape[1] == 3) {
        // NCHW
        model_input_h = static_cast<int>(model_input_shape[2]);
        model_input_w = static_cast<int>(model_input_shape[3]);
    } else if (model_input_shape[3] == 3) {
        // NHWC
        model_input_h = static_cast<int>(model_input_shape[1]);
        model_input_w = static_cast<int>(model_input_shape[2]);
    } else {
        throw std::runtime_error(
            "Unable to infer input layout from model shape: [" +
            std::to_string(model_input_shape[0]) + "," +
            std::to_string(model_input_shape[1]) + "," +
            std::to_string(model_input_shape[2]) + "," +
            std::to_string(model_input_shape[3]) + "]");
    }
    const int requested_h = static_cast<int>(std::lround(dst_size.height));
    const int requested_w = static_cast<int>(std::lround(dst_size.width));

    if (requested_w != model_input_w || requested_h != model_input_h) {
        throw std::runtime_error(
            "Configured input size (" + std::to_string(requested_w) + "x" + std::to_string(requested_h) +
            ") does not match model input size (" + std::to_string(model_input_w) + "x" +
            std::to_string(model_input_h) + "). Please update input_width/input_height.");
    }

    // Apply letterbox preprocessing
    ROS2OpenvinoInfer::Resize resize = ROS2OpenvinoInfer::letterBox(src, dst_size);
    auto *input_data = (uint8_t *)resize.resized_image.data;

    // Create input tensor
    ov::Tensor input_tensor = ov::Tensor(
        this->compiled_model_.input().get_element_type(),
        this->compiled_model_.input().get_shape(),
        input_data);

    // Simplified single-request inference (no double buffering)
    this->infer_requests_[CURR].set_input_tensor(input_tensor);
    this->infer_requests_[CURR].infer();  // Synchronous inference
    
    // Handle startup phase
    if (startup) {
        return {};
    }

    // Process output
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    std::vector<ROS2OpenvinoInfer::Light> Lights;
    std::vector<int> nms_result;
    const ov::Tensor &outputTensor = this->infer_requests_[CURR].get_output_tensor(0);

    ov::Shape outputShape = outputTensor.get_shape();
    auto *outputData = outputTensor.data<float>();

    if (outputShape.size() != 3) {
        throw std::runtime_error("Unsupported output tensor rank: expected 3 dimensions");
    }

    if (output_format_ == "legacy27") {
        // Legacy Katrin output: [1, num_boxes, attrs], attrs typically includes
        // [x, y, w, h, obj, class_scores...].
        for (size_t i = 0; i < outputShape[1]; ++i) {
            float* detection = &outputData[i * outputShape[2]];
            const float confidence = detection[4];

            if (confidence < score_threshold_) {
                continue;
            }

            // Keep historical behavior: class count fixed to 9, only class id 8 is accepted.
            float* classesConfidence = &detection[5];
            cv::Mat scores(1, 9, CV_32FC1, classesConfidence);

            cv::Point classIdPoint;
            double maxClassConfidence;
            cv::minMaxLoc(scores, nullptr, &maxClassConfidence, nullptr, &classIdPoint);

            if (maxClassConfidence < score_threshold_ || classIdPoint.x != 8) {
                continue;
            }

            ROS2OpenvinoInfer::Light Light;
            Light.id = classIdPoint.x;
            Light.score = confidence;
            Light.center_point = cv::Point2f(detection[0], detection[1]);
            Light.box = cv::Rect2d(
                detection[0] - detection[2] / 2.,
                detection[1] - detection[3] / 2.,
                detection[2],
                detection[3]);

            Lights.emplace_back(Light);
            confidences.emplace_back(confidence);
            boxes.emplace_back(cv::Rect(
                detection[0] - detection[2] / 2.,
                detection[1] - detection[3] / 2.,
                detection[2],
                detection[3]));
        }
    } else if (output_format_ == "xywh_conf_5xn") {
        // New INT8 output: [1, 5, num_boxes] where channels are [x, y, w, h, conf].
        if (outputShape[1] != 5) {
            throw std::runtime_error("Output shape mismatch for xywh_conf_5xn: expected channel dimension = 5");
        }

        const size_t num_boxes = outputShape[2];
        for (size_t i = 0; i < num_boxes; ++i) {
            const float x = outputData[0 * num_boxes + i];
            const float y = outputData[1 * num_boxes + i];
            const float w = outputData[2 * num_boxes + i];
            const float h = outputData[3 * num_boxes + i];
            const float confidence = outputData[4 * num_boxes + i];

            if (confidence < score_threshold_) {
                continue;
            }

            ROS2OpenvinoInfer::Light Light;
            // No class logits in this output format; keep downstream armor ID compatibility.
            Light.id = 8;
            Light.score = confidence;
            Light.center_point = cv::Point2f(x, y);
            Light.box = cv::Rect2d(
                x - w / 2.0,
                y - h / 2.0,
                w,
                h);

            Lights.emplace_back(Light);
            confidences.emplace_back(confidence);
            boxes.emplace_back(cv::Rect(
                x - w / 2.0,
                y - h / 2.0,
                w,
                h));
        }
    } else {
        throw std::runtime_error("Unknown model_output_format: " + output_format_ +
                                 ". Supported values: legacy27, xywh_conf_5xn");
    }

    // Apply Non-Maximum Suppression
    cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, nms_result);
    
    std::vector<ROS2OpenvinoInfer::Light> result;
    for(const int &idx : nms_result) {
        result.emplace_back(ROS2OpenvinoInfer::Light {
            Lights[idx].id,
            Lights[idx].score,
            Lights[idx].box,
            Lights[idx].center_point
        });
    }

    // Adjust coordinates back to original image
    fitRec(result, src.size(), dst_size);
    return result;
}

ROS2OpenvinoInfer::Resize ROS2OpenvinoInfer::letterBox(cv::Mat& src, const cv::Size2d& dst_size) {
    auto in_h = static_cast<float>(src.rows);
    auto in_w = static_cast<float>(src.cols);
    float out_h = dst_size.height;
    float out_w = dst_size.width;

    float scale = std::min(out_w / in_w, out_h / in_h);

    int mid_h = static_cast<int>(in_h * scale);
    int mid_w = static_cast<int>(in_w * scale);

    // Create a copy to avoid modifying the source image
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(mid_w, mid_h));

    int dh = (static_cast<int>(out_h) - mid_h) / 2;
    int dw = (static_cast<int>(out_w) - mid_w) / 2;

    cv::Mat padded;
    cv::copyMakeBorder(resized, padded, dh, dh, dw, dw, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

    ROS2OpenvinoInfer::Resize resize;
    resize.resized_image = padded;
    resize.dh = dh;
    resize.dw = dw;
    return resize;
}

void ROS2OpenvinoInfer::fitRec(
    std::vector<Light>& bboxes, 
    cv::Size2d ori_size, 
    cv::Size2d now_size) 
{
    float scale = std::min(now_size.width / ori_size.width, now_size.height / ori_size.height);
    int dh = (static_cast<int>(now_size.height) - static_cast<int>(ori_size.height * scale)) / 2;
    int dw = (static_cast<int>(now_size.width) - static_cast<int>(ori_size.width * scale)) / 2;

    for (auto& bbox : bboxes) {
        bbox.center_point.x = (bbox.center_point.x - dw) / scale;
        bbox.center_point.y = (bbox.center_point.y - dh) / scale;
        bbox.box.x = (bbox.box.x - dw) / scale;
        bbox.box.y = (bbox.box.y - dh) / scale;
        bbox.box.width = bbox.box.width / scale;
        bbox.box.height = bbox.box.height / scale;
    }
}