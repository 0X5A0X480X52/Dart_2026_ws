#include <openvino/openvino.hpp>
#include <iostream>

int main() {
    try {
        ov::Core core;
        std::string xml_path = "/home/amatrix/Dart_2026_ws/src/object_detection_openvino/config/openvino/Katrin.xml";
        std::string bin_path = "/home/amatrix/Dart_2026_ws/src/object_detection_openvino/config/openvino/Katrin.bin";
        
        auto model = core.read_model(xml_path, bin_path);
        
        std::cout << "Model outputs:\n";
        for (size_t i = 0; i < model->outputs().size(); i++) {
            auto output = model->outputs()[i];
            std::cout << "Output " << i << ":\n";
            std::cout << "  Name: " << output.get_any_name() << "\n";
            std::cout << "  Shape: ";
            auto shape = output.get_shape();
            for (size_t j = 0; j < shape.size(); j++) {
                std::cout << shape[j];
                if (j < shape.size() - 1) std::cout << ", ";
            }
            std::cout << "\n\n";
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
