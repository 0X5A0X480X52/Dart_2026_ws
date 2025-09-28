cmake_minimum_required(VERSION 3.8)
project(object_detection_openvino)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

# Find OpenVINO - use direct library linking instead of pkg-config
find_library(OPENVINO_LIB openvino REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  /usr/include/openvino
)

# Add executable for ROS2 node
add_executable(object_detection_openvino_node
  src/object_detection_openvino_node.cpp
  src/ros2_openvino_infer.cpp
)

# Link libraries
target_link_libraries(object_detection_openvino_node
  ${OpenCV_LIBS}
  ${OPENVINO_LIB}
)

# Add compile definitions (removed pkg-config specific flags)

# Specify dependencies for ament
ament_target_dependencies(object_detection_openvino_node
  rclcpp
  sensor_msgs
  rm_interfaces
  cv_bridge
  std_msgs
  image_transport
)

# Install executable
install(TARGETS
  object_detection_openvino_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# Install scripts
install(PROGRAMS
  scripts/test_detection_results.py
  scripts/check_config.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
