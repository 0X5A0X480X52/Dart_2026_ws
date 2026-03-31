#include "object_detection_openvino/object_detection_openvino_node.hpp"
#include <rm_interfaces/msg/target2_d.hpp>
#include <std_msgs/msg/header.hpp>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>

ObjectDetectionOpenvinoNode::ObjectDetectionOpenvinoNode() 
    : Node("object_detection_openvino_node"), startup_(true)
{
    RCLCPP_INFO(this->get_logger(), "Initializing Object Detection OpenVINO Node");
    
    // Initialize parameters
    initializeParameters();
    
    // Load OpenVINO model
    loadModel();
    
    // Create subscription to image topic
    const auto image_queue_depth = static_cast<std::size_t>(std::max(1, image_queue_size_));
    rclcpp::QoS image_qos{rclcpp::KeepLast(image_queue_depth)};
    if (use_sensor_data_qos_) {
        image_qos = rclcpp::SensorDataQoS().keep_last(image_queue_depth);
    }

    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, image_qos,
        std::bind(&ObjectDetectionOpenvinoNode::imageCallback, this, std::placeholders::_1));
    
    // Create publisher for detection results
    target_publisher_ = this->create_publisher<rm_interfaces::msg::Target2DArray>(
        detection_topic_, 10);
    
    // Create publisher for debug image (if enabled)
    if (publish_debug_image_) {
        auto debug_qos_profile = use_sensor_data_qos_ ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
        debug_qos_profile.depth = static_cast<size_t>(std::max(1, debug_image_queue_size_));

        debug_image_publisher_ = image_transport::create_publisher(
            this, debug_image_topic_, debug_qos_profile);

        RCLCPP_INFO(
            this->get_logger(),
            "Debug image publisher created (base topic: %s, transport: raw/compressed if plugin is available)",
            debug_image_topic_.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Debug image publishing is disabled");
    }

    perf_window_start_ = std::chrono::steady_clock::now();
    
    RCLCPP_INFO(this->get_logger(), "Object Detection OpenVINO Node initialized successfully");
}

ObjectDetectionOpenvinoNode::~ObjectDetectionOpenvinoNode()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down Object Detection OpenVINO Node");
}

void ObjectDetectionOpenvinoNode::initializeParameters()
{
    // Declare and get parameters
    this->declare_parameter("mode", "armor");
    this->declare_parameter("input_width", 640);
    this->declare_parameter("input_height", 640);
    this->declare_parameter("score_threshold", 0.5);
    this->declare_parameter("nms_threshold", 0.4);
    this->declare_parameter("model_output_format", "legacy27");
    this->declare_parameter("xml_path", "./src/object_detection_openvino/config/openvino/Katrin.xml");
    this->declare_parameter("bin_path", "./src/object_detection_openvino/config/openvino/Katrin.bin");
    this->declare_parameter("device", "CPU");
    this->declare_parameter("image_topic", "/camera_left/image_raw");
    this->declare_parameter("detection_topic", "/detector/target2d_array");
    this->declare_parameter("debug_image_topic", "/detector/debug_image");
    this->declare_parameter("publish_debug_image", true);
    this->declare_parameter("enable_performance_log", true);
    this->declare_parameter("perf_log_interval", 60);
    this->declare_parameter("image_queue_size", 10);
    this->declare_parameter("debug_image_queue_size", 2);
    this->declare_parameter("use_sensor_data_qos", false);
    this->declare_parameter("roi_mode", "center");
    this->declare_parameter("roi_width", 1280);
    this->declare_parameter("roi_height", 1024);
    this->declare_parameter("center_mode", "auto");
    this->declare_parameter("center_x", -1.0);
    this->declare_parameter("center_y", -1.0);
    
    mode_ = this->get_parameter("mode").as_string();
    input_width_ = this->get_parameter("input_width").as_int();
    input_height_ = this->get_parameter("input_height").as_int();
    score_threshold_ = this->get_parameter("score_threshold").as_double();
    nms_threshold_ = this->get_parameter("nms_threshold").as_double();
    model_output_format_ = this->get_parameter("model_output_format").as_string();
    xml_path_ = this->get_parameter("xml_path").as_string();
    bin_path_ = this->get_parameter("bin_path").as_string();
    device_ = this->get_parameter("device").as_string();
    image_topic_ = this->get_parameter("image_topic").as_string();
    detection_topic_ = this->get_parameter("detection_topic").as_string();
    debug_image_topic_ = this->get_parameter("debug_image_topic").as_string();
    publish_debug_image_ = this->get_parameter("publish_debug_image").as_bool();
    enable_performance_log_ = this->get_parameter("enable_performance_log").as_bool();
    perf_log_interval_ = this->get_parameter("perf_log_interval").as_int();
    image_queue_size_ = this->get_parameter("image_queue_size").as_int();
    debug_image_queue_size_ = this->get_parameter("debug_image_queue_size").as_int();
    use_sensor_data_qos_ = this->get_parameter("use_sensor_data_qos").as_bool();
    roi_mode_ = this->get_parameter("roi_mode").as_string();
    roi_width_ = this->get_parameter("roi_width").as_int();
    roi_height_ = this->get_parameter("roi_height").as_int();
    center_mode_ = this->get_parameter("center_mode").as_string();
    center_x_ = this->get_parameter("center_x").as_double();
    center_y_ = this->get_parameter("center_y").as_double();
    
    RCLCPP_INFO(this->get_logger(), "Parameters initialized:");
    RCLCPP_INFO(this->get_logger(), "  Mode: %s", mode_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Input size: %dx%d", input_width_, input_height_);
    RCLCPP_INFO(this->get_logger(), "  Score threshold: %.2f", score_threshold_);
    RCLCPP_INFO(this->get_logger(), "  NMS threshold: %.2f", nms_threshold_);
    RCLCPP_INFO(this->get_logger(), "  Model output format: %s", model_output_format_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Model XML: %s", xml_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Model BIN: %s", bin_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Device: %s", device_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Image topic: %s", image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Detection topic: %s", detection_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Debug image topic: %s", debug_image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Publish debug image: %s", publish_debug_image_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Enable performance log: %s", enable_performance_log_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Performance log interval: %d frames", perf_log_interval_);
    RCLCPP_INFO(this->get_logger(), "  Image queue size: %d", image_queue_size_);
    RCLCPP_INFO(this->get_logger(), "  Debug image queue size: %d", debug_image_queue_size_);
    RCLCPP_INFO(this->get_logger(), "  Use sensor data QoS: %s", use_sensor_data_qos_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  ROI mode: %s", roi_mode_.c_str());
    RCLCPP_INFO(this->get_logger(), "  ROI size: %dx%d", roi_width_, roi_height_);
    RCLCPP_INFO(this->get_logger(), "  Center mode: %s", center_mode_.c_str());
    RCLCPP_INFO(this->get_logger(), "  ROI center: (%.3g, %.3g)", center_x_, center_y_);
}

void ObjectDetectionOpenvinoNode::loadModel()
{
    try {
        std::map<std::string, std::string> path_map;
        path_map["XML"] = xml_path_;
        path_map["BIN"] = bin_path_;
        path_map["DEVICE"] = device_;
        
        openvino_infer_ = std::make_unique<ROS2OpenvinoInfer>(
            path_map, 
            model_output_format_,
            score_threshold_, 
            nms_threshold_
        );
        RCLCPP_INFO(this->get_logger(), "OpenVINO model loaded successfully");
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load OpenVINO model: %s", e.what());
        throw;
    }
}

void ObjectDetectionOpenvinoNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        const auto callback_start = std::chrono::steady_clock::now();
        bool has_msg_stamp = !(msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0);
        double msg_age_ms = 0.0;
        if (has_msg_stamp) {
            msg_age_ms = (this->now() - rclcpp::Time(msg->header.stamp)).seconds() * 1000.0;
        }

        // Convert ROS image message to OpenCV Mat (use share to avoid copy when possible)
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception&) {
            // Fallback to copy if share fails
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        const cv::Mat& image = cv_ptr->image;
        
        // Prepare ROI image and record ROI rectangle
        cv::Mat roi_image;
        cv::Rect roi_rect;
        
        if (roi_mode_ == "center") {
            // Center-crop strategy: extract centered region with specified size
            int img_width = image.cols;
            int img_height = image.rows;
            
            // Calculate ROI dimensions (clamp to image size)
            int actual_roi_w = std::min(roi_width_, img_width);
            int actual_roi_h = std::min(roi_height_, img_height);
            
            // Determine center point based on center_mode
            double raw_cx = center_x_;
            double raw_cy = center_y_;

            double cx_px_d;
            double cy_px_d;
            
            // X coordinate interpretation
            if (center_mode_ == "pixel") {
                cx_px_d = raw_cx;
            } else if (center_mode_ == "percent") {
                cx_px_d = (raw_cx / 100.0) * img_width;
            } else if (center_mode_ == "fraction") {
                cx_px_d = raw_cx * img_width;
            } else { // "auto" mode - auto-detect based on value
                if (raw_cx < 0.0) {
                    cx_px_d = img_width / 2.0;
                } else if (raw_cx <= 1.0) {
                    cx_px_d = raw_cx * img_width;
                } else if (raw_cx <= 100.0) {
                    cx_px_d = (raw_cx / 100.0) * img_width;
                } else {
                    cx_px_d = raw_cx;
                }
            }
            
            // Y coordinate interpretation
            if (center_mode_ == "pixel") {
                cy_px_d = raw_cy;
            } else if (center_mode_ == "percent") {
                cy_px_d = (raw_cy / 100.0) * img_height;
            } else if (center_mode_ == "fraction") {
                cy_px_d = raw_cy * img_height;
            } else { // "auto" mode - auto-detect based on value
                if (raw_cy < 0.0) {
                    cy_px_d = img_height / 2.0;
                } else if (raw_cy <= 1.0) {
                    cy_px_d = raw_cy * img_height;
                } else if (raw_cy <= 100.0) {
                    cy_px_d = (raw_cy / 100.0) * img_height;
                } else {
                    cy_px_d = raw_cy;
                }
            }

            // Round and clamp to valid image bounds
            int center_x = std::min(std::max(static_cast<int>(std::lround(cx_px_d)), 0), img_width - 1);
            int center_y = std::min(std::max(static_cast<int>(std::lround(cy_px_d)), 0), img_height - 1);

            if (center_x != static_cast<int>(std::lround(cx_px_d)) || center_y != static_cast<int>(std::lround(cy_px_d))) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "Requested ROI center interpreted as (%.3g,%.3g) -> clamped to (%d,%d)",
                    raw_cx,
                    raw_cy,
                    center_x,
                    center_y);
            }

            // Calculate top-left corner based on center point
            int x = center_x - actual_roi_w / 2;
            int y = center_y - actual_roi_h / 2;
            
            roi_rect = cv::Rect(x, y, actual_roi_w, actual_roi_h);
            
            // Clamp ROI to valid image bounds (safety check)
            roi_rect.x = std::max(0, roi_rect.x);
            roi_rect.y = std::max(0, roi_rect.y);
            roi_rect.width = std::min(roi_rect.width, img_width - roi_rect.x);
            roi_rect.height = std::min(roi_rect.height, img_height - roi_rect.y);
            
            // Extract ROI (shallow copy, no data duplication)
            roi_image = image(roi_rect);
            
            RCLCPP_DEBUG(this->get_logger(), "ROI center mode: rect=[%d,%d,%d,%d]", 
                        roi_rect.x, roi_rect.y, roi_rect.width, roi_rect.height);
            // Log the effective center used for this ROI (after applying -1 => image center and clamping)
            int effective_cx = roi_rect.x + roi_rect.width / 2;
            int effective_cy = roi_rect.y + roi_rect.height / 2;
            RCLCPP_DEBUG_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Using ROI center=(%d,%d), rect=[%d,%d,%d,%d]",
                effective_cx,
                effective_cy,
                roi_rect.x,
                roi_rect.y,
                roi_rect.width,
                roi_rect.height);
        } else {
            // Full image mode: use entire image as ROI
            roi_image = image;
            roi_rect = cv::Rect(0, 0, image.cols, image.rows);
            
            RCLCPP_DEBUG(this->get_logger(), "ROI full mode: using entire image");
        }
        
        // Store ROI rect for visualization
        roi_rect_ = roi_rect;

        const auto infer_start = std::chrono::steady_clock::now();
        
        // Perform inference on ROI image
        auto detection_results = openvino_infer_->infer(
            roi_image, 
            cv::Size(input_width_, input_height_), 
            0,  // my_color (not used in current implementation)
            startup_
        );
        const auto infer_end = std::chrono::steady_clock::now();
        const double infer_ms = std::chrono::duration<double, std::milli>(infer_end - infer_start).count();
        
        // Handle startup phase (skip first frame)
        if (startup_) {
            startup_ = false;
            RCLCPP_INFO(this->get_logger(), "Model warmup completed");
            return;
        }
        
        // Map detection results from ROI coordinates back to full image coordinates
        for (auto& detection : detection_results) {
            detection.center_point.x += roi_rect.x;
            detection.center_point.y += roi_rect.y;
            detection.box.x += roi_rect.x;
            detection.box.y += roi_rect.y;
        }
        
        // CRITICAL: Make deep copy of header to avoid race conditions with concurrent callbacks
        // The msg->header.frame_id (std::string) must not be shared across threads
        std_msgs::msg::Header header_copy;
        header_copy.stamp = msg->header.stamp;
        header_copy.frame_id = std::string(msg->header.frame_id);  // Force deep copy of string
        
        // Convert detection results to ROS message
        auto target_array_msg = convertToRosMessage(detection_results, header_copy);
        
        // Publish detection results (do this ASAP for synchronization)
        target_publisher_->publish(target_array_msg);

        double debug_publish_ms = 0.0;
        
        // Publish debug image if enabled (after main publish to minimize delay)
        if (publish_debug_image_ && debug_image_publisher_) {
            const auto debug_start = std::chrono::steady_clock::now();

            // Only clone when necessary for debug visualization
            cv::Mat debug_image = image.clone();
            drawDebugImage(debug_image, detection_results, roi_rect);
            
            // Use the same deep-copied header for debug image
            auto debug_msg = cv_bridge::CvImage(header_copy, "bgr8", debug_image).toImageMsg();
            debug_image_publisher_.publish(*debug_msg);

            const auto debug_end = std::chrono::steady_clock::now();
            debug_publish_ms = std::chrono::duration<double, std::milli>(debug_end - debug_start).count();
        }

        const auto callback_end = std::chrono::steady_clock::now();
        const double total_ms = std::chrono::duration<double, std::milli>(callback_end - callback_start).count();
        updatePerformanceStats(infer_ms, total_ms, debug_publish_ms, msg_age_ms, has_msg_stamp);
        
        RCLCPP_DEBUG(this->get_logger(), "Published %zu detections", detection_results.size());
    }
    catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Detection processing error: %s", e.what());
    }
}

void ObjectDetectionOpenvinoNode::updatePerformanceStats(
    double infer_ms,
    double total_ms,
    double debug_ms,
    double msg_age_ms,
    bool has_msg_stamp)
{
    if (!enable_performance_log_ || perf_log_interval_ <= 0) {
        return;
    }

    if (perf_sample_count_ == 0) {
        perf_window_start_ = std::chrono::steady_clock::now();
    }

    perf_sample_count_ += 1;
    perf_infer_ms_sum_ += infer_ms;
    perf_total_ms_sum_ += total_ms;
    perf_debug_ms_sum_ += debug_ms;

    if (has_msg_stamp) {
        perf_msg_age_ms_sum_ += msg_age_ms;
        perf_msg_age_count_ += 1;
    }

    if (perf_sample_count_ < static_cast<std::size_t>(perf_log_interval_)) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const double window_seconds = std::chrono::duration<double>(now - perf_window_start_).count();

    const double avg_infer_ms = perf_infer_ms_sum_ / static_cast<double>(perf_sample_count_);
    const double avg_total_ms = perf_total_ms_sum_ / static_cast<double>(perf_sample_count_);
    const double avg_debug_ms = perf_debug_ms_sum_ / static_cast<double>(perf_sample_count_);
    const double infer_fps = avg_infer_ms > 0.0 ? 1000.0 / avg_infer_ms : 0.0;
    const double process_fps = avg_total_ms > 0.0 ? 1000.0 / avg_total_ms : 0.0;
    const double throughput_fps = window_seconds > 0.0 ? static_cast<double>(perf_sample_count_) / window_seconds : 0.0;

    if (perf_msg_age_count_ > 0) {
        const double avg_msg_age_ms = perf_msg_age_ms_sum_ / static_cast<double>(perf_msg_age_count_);
        RCLCPP_INFO(
            this->get_logger(),
            "[Perf] infer=%.2f ms (%.2f FPS), total=%.2f ms (%.2f FPS), debug_pub=%.2f ms, input_age=%.2f ms, throughput=%.2f FPS",
            avg_infer_ms,
            infer_fps,
            avg_total_ms,
            process_fps,
            avg_debug_ms,
            avg_msg_age_ms,
            throughput_fps);

        if (avg_msg_age_ms > 500.0) {
            RCLCPP_WARN(
                this->get_logger(),
                "[Perf] input_age=%.2f ms is high. Potential queue backlog (try image_queue_size:=1 or sensor-data QoS).",
                avg_msg_age_ms);
        }
    } else {
        RCLCPP_INFO(
            this->get_logger(),
            "[Perf] infer=%.2f ms (%.2f FPS), total=%.2f ms (%.2f FPS), debug_pub=%.2f ms, throughput=%.2f FPS",
            avg_infer_ms,
            infer_fps,
            avg_total_ms,
            process_fps,
            avg_debug_ms,
            throughput_fps);
    }

    perf_sample_count_ = 0;
    perf_msg_age_count_ = 0;
    perf_infer_ms_sum_ = 0.0;
    perf_total_ms_sum_ = 0.0;
    perf_debug_ms_sum_ = 0.0;
    perf_msg_age_ms_sum_ = 0.0;
    perf_window_start_ = now;
}

rm_interfaces::msg::Target2DArray ObjectDetectionOpenvinoNode::convertToRosMessage(
    const std::vector<ROS2OpenvinoInfer::Light>& detections,
    const std_msgs::msg::Header& header)
{
    rm_interfaces::msg::Target2DArray target_array;

    // Header：整体赋值（Target2DArray 的 header 已足够）
    target_array.header = header;

    // 预分配 vector，避免反复 realloc / 拷贝
    target_array.targets.reserve(detections.size());

    for (const auto& detection : detections) {
        // 原地构造，避免临时对象 + push_back 拷贝
        auto& target = target_array.targets.emplace_back();

        // 不设置每个 target 的 header（Target2DArray 已有，避免冗余序列化）
        // target.header 保持默认构造状态

        // ---- 几何信息 ----
        target.x = static_cast<float>(detection.center_point.x);
        target.y = static_cast<float>(detection.center_point.y);
        target.width  = static_cast<float>(detection.box.width);
        target.height = static_cast<float>(detection.box.height);

        // ---- 置信度 ----
        target.confidence = static_cast<float>(detection.score);

        // ---- 类别映射 ----
        // rm_interfaces::Target2D.class_id is the semantic category.
        // For xywh_conf_5xn there are no class logits and this model detects armor only.
        if (model_output_format_ == "xywh_conf_5xn") {
            target.class_id = 1;  // armor
        } else {
            switch (detection.id) {
                case 0:
                    target.class_id = 0;  // armor_blue
                    break;
                case 1:
                    target.class_id = 1;  // armor_red
                    break;
                case 8:
                    target.class_id = 8;  // armor
                    break;
                default:
                    target.class_id = 255;  // unknown
                    break;
            }
        }

        // ---- 跟踪 ID ----
        target.id = detection.id;

        // ---- 滤波标志 ----
        target.is_filtered = false;
    }

    return target_array;  // RVO / NRVO，不拷贝
}

void ObjectDetectionOpenvinoNode::drawDebugImage(
    cv::Mat& image,
    const std::vector<ROS2OpenvinoInfer::Light>& detections,
    const cv::Rect& roi_rect)
{
    // Define colors for different classes
    const cv::Scalar COLOR_GREEN(0, 255, 0);
    const cv::Scalar COLOR_RED(0, 0, 255);
    const cv::Scalar COLOR_BLUE(255, 0, 0);
    const cv::Scalar COLOR_YELLOW(0, 255, 255);
    const cv::Scalar COLOR_CYAN(255, 255, 0);
    
    // Draw ROI rectangle with thick cyan border
    if (roi_rect.width > 0 && roi_rect.height > 0) {
        cv::rectangle(image, roi_rect, COLOR_CYAN, 3);
        
        // Add ROI label
        std::string roi_label = "ROI: " + std::to_string(roi_rect.width) + "x" + std::to_string(roi_rect.height);
        int baseline = 0;
        cv::Size label_size = cv::getTextSize(roi_label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
        
        // Position ROI label at top-left corner of ROI
        cv::Point roi_label_pos(roi_rect.x + 5, roi_rect.y + label_size.height + 5);
        
        // Draw background for ROI label
        cv::rectangle(image, 
                     cv::Point(roi_label_pos.x - 2, roi_label_pos.y - label_size.height - 2),
                     cv::Point(roi_label_pos.x + label_size.width + 2, roi_label_pos.y + baseline + 2),
                     COLOR_CYAN, -1);
        
        cv::putText(image, roi_label, roi_label_pos, 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
    }
    
    // Draw detection boxes (already in full image coordinates)
    for (const auto& detection : detections) {
        // Choose color based on detection ID
        cv::Scalar box_color;
        std::string class_name;
        
        switch (detection.id) {
            case 0:
                box_color = COLOR_BLUE;
                class_name = "armor_blue";
                break;
            case 1:
                box_color = COLOR_RED;
                class_name = "armor_red";
                break;
            case 8:
                box_color = COLOR_GREEN;
                class_name = "armor";
                break;
            default:
                box_color = COLOR_YELLOW;
                class_name = "unknown";
                break;
        }
        
        // Draw bounding box
        cv::rectangle(image, detection.box, box_color, 2);
        
        // Draw center point
        cv::circle(image, detection.center_point, 4, box_color, -1);
        
        // Prepare label with class name and confidence
        std::stringstream label_stream;
        label_stream << class_name << ": " << std::fixed << std::setprecision(2) << detection.score;
        std::string label = label_stream.str();
        
        // Get label size for background rectangle
        int baseline = 0;
        cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        
        // Draw background rectangle for label
        cv::Point label_origin(detection.box.x, detection.box.y - 5);
        if (label_origin.y < 0) label_origin.y = detection.box.y + detection.box.height + 15;
        
        cv::rectangle(image, 
                     cv::Point(label_origin.x, label_origin.y - label_size.height - 5),
                     cv::Point(label_origin.x + label_size.width, label_origin.y + baseline),
                     box_color, -1);
        
        // Draw label text
        cv::putText(image, label, label_origin, 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
    
    // Draw detection count in top-left corner
    std::stringstream info_stream;
    info_stream << "Detections: " << detections.size();
    cv::putText(image, info_stream.str(), cv::Point(10, 30),
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<ObjectDetectionOpenvinoNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Node execution failed: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}