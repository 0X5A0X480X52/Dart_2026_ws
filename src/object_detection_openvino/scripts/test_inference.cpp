#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    try {
        ov::Core core;
        std::string xml_path = "/home/amatrix/Dart_2026_ws/src/object_detection_openvino/config/openvino/Katrin.xml";
        std::string bin_path = "/home/amatrix/Dart_2026_ws/src/object_detection_openvino/config/openvino/Katrin.bin";
        
        // Load model
        auto model = core.read_model(xml_path, bin_path);
        
        // Print input info
        std::cout << "=== MODEL INPUT ===" << std::endl;
        for (const auto& input : model->inputs()) {
            std::cout << "Name: " << input.get_any_name() << std::endl;
            std::cout << "Shape: ";
            for (auto dim : input.get_shape()) {
                std::cout << dim << " ";
            }
            std::cout << std::endl;
        }
        
        // Print output info
        std::cout << "\n=== MODEL OUTPUTS ===" << std::endl;
        for (size_t i = 0; i < model->outputs().size(); i++) {
            auto output = model->outputs()[i];
            std::cout << "Output " << i << ":" << std::endl;
            std::cout << "  Name: " << output.get_any_name() << std::endl;
            std::cout << "  Shape: ";
            for (auto dim : output.get_shape()) {
                std::cout << dim << " ";
            }
            std::cout << std::endl;
        }
        
        // Compile and run inference with a test image
        auto compiled_model = core.compile_model(model, "CPU");
        auto infer_request = compiled_model.create_infer_request();
        
        // Create a dummy input (black image)
        cv::Mat test_image = cv::Mat::zeros(384, 640, CV_8UC3);
        
        // Create input tensor
        ov::Shape input_shape = {1, 3, 384, 640};
        ov::Tensor input_tensor(ov::element::u8, input_shape);
        
        // Fill with test data
        uint8_t* input_data = input_tensor.data<uint8_t>();
        for (int c = 0; c < 3; c++) {
            for (int h = 0; h < 384; h++) {
                for (int w = 0; w < 640; w++) {
                    input_data[c * 384 * 640 + h * 640 + w] = test_image.at<cv::Vec3b>(h, w)[c];
                }
            }
        }
        
        infer_request.set_input_tensor(input_tensor);
        infer_request.infer();
        
        // Check output 0 (main detection output)
        std::cout << "\n=== OUTPUT 0 ANALYSIS ===" << std::endl;
        auto output_tensor = infer_request.get_output_tensor(0);
        auto output_shape = output_tensor.get_shape();
        const float* output_data = output_tensor.data<const float>();
        
        std::cout << "Shape: ";
        for (auto dim : output_shape) {
            std::cout << dim << " ";
        }
        std::cout << std::endl;
        
        // Print first few values
        std::cout << "First detection values (first 27 floats):" << std::endl;
        for (int i = 0; i < 27 && i < output_shape[1] * output_shape[2]; i++) {
            std::cout << "  [" << i << "]: " << output_data[i] << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
