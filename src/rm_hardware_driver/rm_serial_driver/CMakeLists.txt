cmake_minimum_required(VERSION 3.15)
project(rm_serial_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(rm_utils REQUIRED)

# Include directories
include_directories(
  include
)

# Library
add_library(${PROJECT_NAME} SHARED
  src/serial_driver_node.cpp
  src/protocol/dart_protocol.cpp
  src/transporter/uart_transporter.cpp
  ../../rm_utils/logger/impl/writer.cpp
  ../../rm_utils/logger/impl/logger_impl.cpp
  ../../rm_utils/logger/impl/logger_pool.cpp
)

target_compile_definitions(${PROJECT_NAME} PUBLIC EIGEN_NO_DEBUG)

# Add dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  Eigen3
  rm_interfaces
  rm_utils
)

# Register component
rclcpp_components_register_nodes(${PROJECT_NAME} "fyt::serial_driver::SerialDriverNode")

# Executable
add_executable(serial_driver_node
  src/serial_driver_node.cpp
)

# Link libraries to executable
target_link_libraries(serial_driver_node
  ${PROJECT_NAME}
  fmt
)

# Add dependencies to executable
ament_target_dependencies(serial_driver_node
  rclcpp
  rm_interfaces
  rm_utils
)

# 添加虚拟串口节点可执行文件
add_executable(virtual_serial_node
  src/virtual_serial_node.cpp
)

# 为虚拟串口节点链接库
target_link_libraries(virtual_serial_node
  ${PROJECT_NAME}
  fmt
)

# 为虚拟串口节点添加依赖
ament_target_dependencies(virtual_serial_node
  rclcpp
  rm_interfaces
  rm_utils
)

# Install
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  serial_driver_node
  virtual_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files and config
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
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