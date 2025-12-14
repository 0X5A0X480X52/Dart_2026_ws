cmake_minimum_required(VERSION 3.15)
project(rm_serial_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

add_library(${PROJECT_NAME} SHARED
  src/serial_driver_node.cpp
  src/protocol/dart_protocol.cpp
  src/transporter/uart_transporter.cpp
)

target_compile_definitions(${PROJECT_NAME} PUBLIC EIGEN_NO_DEBUG)

# ✅ 现代 include 写法（替换 include_directories）
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

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

rclcpp_components_register_nodes(
  ${PROJECT_NAME}
  "fyt::serial_driver::SerialDriverNode"
)

add_executable(serial_driver_node
  src/serial_driver_node.cpp
)
target_link_libraries(serial_driver_node
  ${PROJECT_NAME}
)

ament_target_dependencies(serial_driver_node
  rclcpp
)

add_executable(virtual_serial_node
  src/virtual_serial_node.cpp
)
target_link_libraries(virtual_serial_node
  ${PROJECT_NAME}
)

ament_target_dependencies(virtual_serial_node
  rclcpp
)

install(TARGETS
  ${PROJECT_NAME}
  serial_driver_node
  virtual_serial_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
