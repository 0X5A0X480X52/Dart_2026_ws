# resize_image_raw

一个基于 C++ 的 ROS2 图像缩放节点，类似于 `image_proc` 的 `resize_node` 功能。

## 功能特点

- 支持订阅图像和相机信息（camera_info）
- 可配置的宽度和高度缩放比例
- 支持多种插值方法（NEAREST, LINEAR, CUBIC, AREA）
- 自动缩放相机内参矩阵（K 和 P 矩阵）
- 处理 ROI（感兴趣区域）

## 依赖

- ROS2 (Humble/Iron/Rolling)
- OpenCV
- cv_bridge
- image_transport
- sensor_msgs

## 编译

```bash
cd /home/amatrix/Dart_2026_ws
colcon build --packages-select resize_image_raw
source install/setup.bash
```

## 使用方法

### 方法 1: 使用配置文件（推荐）

使用默认配置文件：
```bash
ros2 launch resize_image_raw resize_with_config.launch.py
```

使用自定义配置文件：
```bash
ros2 launch resize_image_raw resize_with_config.launch.py \
    config_file:=/path/to/your/config.yaml
```

使用预设配置文件：
```bash
# 缩小到一半（高质量）
ros2 launch resize_image_raw downscale_half.launch.py

# 放大到两倍（高质量）
ros2 launch resize_image_raw upscale_double.launch.py
```

### 方法 2: 使用命令行参数

直接运行：
```bash
ros2 run resize_image_raw resize_node_exe --ros-args \
    -r image:=/camera_left/image_raw \
    -r camera_info:=/camera_left/camera_info \
    -r resized/image:=/resize/image_raw \
    -r resized/camera_info:=/resize/camera_info \
    -p scale_height:=0.5 \
    -p scale_width:=0.5
```

使用 launch 文件：
```bash
ros2 launch resize_image_raw resize_node.launch.py \
    scale_width:=0.5 \
    scale_height:=0.5 \
    image_topic:=/camera_left/image_raw \
    camera_info_topic:=/camera_left/camera_info \
    resized_image_topic:=/resize/image_raw \
    resized_camera_info_topic:=/resize/camera_info
```

### 方法 3: 混合模式

使用配置文件但覆盖话题名称：
```bash
ros2 launch resize_image_raw resize_with_config.launch.py \
    config_file:=/path/to/config.yaml \
    image_topic:=/my_camera/image_raw
```

## 参数说明

### 缩放参数

- `scale_width` (double, 默认: 1.0): 宽度缩放比例
- `scale_height` (double, 默认: 1.0): 高度缩放比例
- `interpolation` (int, 默认: 1): 插值方法
  - 0: NEAREST (最近邻)
  - 1: LINEAR (线性)
  - 2: CUBIC (三次)
  - 3: AREA (区域)

### 话题名称参数（新功能！）

- `input_image_topic` (string, 默认: "image"): 输入图像话题
- `input_camera_info_topic` (string, 默认: "camera_info"): 输入相机信息话题
- `output_image_topic` (string, 默认: "resized/image"): 输出图像话题
- `output_camera_info_topic` (string, 默认: "resized/camera_info"): 输出相机信息话题

## 配置文件

包提供了多个预设配置文件，位于 `config/` 目录：

- `resize_params.yaml`: 默认配置（缩小到 50%，线性插值）
- `downscale_half.yaml`: 高质量缩小到一半（AREA 插值）
- `upscale_double.yaml`: 高质量放大到两倍（CUBIC 插值）
- `fast_resize.yaml`: 快速处理（NEAREST 插值）
- `custom_aspect.yaml`: 自定义宽高比示例

### 创建自定义配置文件

```yaml
resize_node:
  ros__parameters:
    scale_width: 0.5      # 宽度缩放比例
    scale_height: 0.5     # 高度缩放比例
    interpolation: 1      # 插值方法 (0-3)
```

## 话题

### 订阅

- `image` (sensor_msgs/Image): 输入图像
- `camera_info` (sensor_msgs/CameraInfo): 输入相机信息

### 发布

- `resized/image` (sensor_msgs/Image): 缩放后的图像
- `resized/camera_info` (sensor_msgs/CameraInfo): 缩放后的相机信息

## 示例

### 示例 1: 基本缩放

将图像缩小到原来的 50%：

```bash
ros2 run resize_image_raw resize_node_exe --ros-args \
    -p scale_width:=0.5 \
    -p scale_height:=0.5
```

### 示例 2: 使用配置文件指定话题

使用左相机配置文件：

```bash
ros2 launch resize_image_raw resize_with_config.launch.py \
    config_file:=/path/to/camera_left.yaml
```

### 示例 3: 通过参数指定话题（不使用 remapping）

```bash
ros2 run resize_image_raw resize_node_exe --ros-args \
    -p scale_width:=0.5 \
    -p scale_height:=0.5 \
    -p input_image_topic:=/camera_left/image_raw \
    -p input_camera_info_topic:=/camera_left/camera_info \
    -p output_image_topic:=/camera_left/resized/image_raw \
    -p output_camera_info_topic:=/camera_left/resized/camera_info
```

### 示例 4: 双目相机同时处理

启动左右两个相机的缩放节点：

```bash
ros2 launch resize_image_raw stereo_cameras_with_config.launch.py
```

### 示例 5: 使用最近邻插值方法（适合快速处理）

```bash
ros2 run resize_image_raw resize_node_exe --ros-args \
    -p scale_width:=0.5 \
    -p scale_height:=0.5 \
    -p interpolation:=0
```
