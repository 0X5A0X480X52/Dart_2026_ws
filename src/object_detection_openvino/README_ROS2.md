# Object Detection OpenVINO ROS2 Node

这是一个基于OpenVINO的目标检测ROS2节点，用于实时检测图像中的目标并发布检测结果。

## 功能特点

- 订阅相机的 `image_raw` 话题进行实时检测
- 使用OpenVINO进行高性能推理
- 发布检测结果到 `/detector/target2d_array` 话题
- 支持多种OpenVINO支持的硬件设备（CPU、GPU等）
- 通过ROS2参数系统进行配置

## 依赖项

- ROS2 (推荐 Humble 或更新版本)
- OpenVINO Runtime
- OpenCV
- cv_bridge
- rm_interfaces

## 编译

```bash
# 在工作空间根目录下
colcon build --packages-select object_detection_openvino
source install/setup.bash
```

## 使用方法

### 1. 准备模型文件

确保你有OpenVINO格式的模型文件（.xml 和 .bin）。

### 2. 配置参数

编辑 `config/params.yaml` 文件，更新模型路径和其他参数：

```yaml
object_detection_openvino_node:
  ros__parameters:
    xml_path: "/path/to/your/model.xml"
    bin_path: "/path/to/your/model.bin"
    device: "CPU"  # 或 "GPU", "MYRIAD" 等
    input_width: 640
    input_height: 640
    score_threshold: 0.5
    nms_threshold: 0.4
```

### 3. 启动节点

#### 方法1：使用启动文件

```bash
ros2 launch object_detection_openvino object_detection_openvino.launch.py \
    xml_path:="/path/to/your/model.xml" \
    bin_path:="/path/to/your/model.bin" \
    image_topic:="/camera/image_raw"
```

#### 方法2：直接运行节点

```bash
ros2 run object_detection_openvino object_detection_openvino_node \
    --ros-args \
    -p xml_path:="/path/to/your/model.xml" \
    -p bin_path:="/path/to/your/model.bin" \
    -r image_raw:="/camera/image_raw"
```

#### 方法3：使用参数文件

```bash
ros2 run object_detection_openvino object_detection_openvino_node \
    --ros-args --params-file src/object_detection_openvino/config/params.yaml \
    -r image_raw:="/camera/image_raw"
```

## 话题接口

### 订阅话题

- `image_raw` (sensor_msgs/Image): 输入图像

### 发布话题

- `/detector/target2d_array` (rm_interfaces/Target2DArray): 检测结果

## 参数

| 参数名称 | 类型 | 默认值 | 描述 |
|---------|------|--------|------|
| mode | string | "armor" | 检测模式 |
| input_width | int | 640 | 模型输入图像宽度 |
| input_height | int | 640 | 模型输入图像高度 |
| score_threshold | double | 0.5 | 检测置信度阈值 |
| nms_threshold | double | 0.4 | 非最大值抑制阈值 |
| xml_path | string | "/path/to/model.xml" | OpenVINO模型XML文件路径 |
| bin_path | string | "/path/to/model.bin" | OpenVINO模型BIN文件路径 |
| device | string | "CPU" | OpenVINO推理设备 |
| image_topic | string | "image_raw" | 订阅的图像话题名称 |
| detection_topic | string | "/detector/target2d_array" | 发布的检测结果话题名称 |

## 检测结果格式

检测结果使用 `rm_interfaces/Target2DArray` 消息类型，包含以下信息：

- 检测框中心坐标 (x, y)
- 检测框尺寸 (width, height)
- 置信度分数
- 目标类别名称
- 目标ID

## 性能优化

1. **设备选择**: 如果有GPU，设置 `device: "GPU"` 可以提高推理速度
2. **模型尺寸**: 根据实际需求调整 `input_width` 和 `input_height`
3. **阈值调优**: 根据检测精度需求调整 `score_threshold` 和 `nms_threshold`

## 故障排除

### 常见问题

1. **模型加载失败**
   - 检查模型文件路径是否正确
   - 确认模型文件格式是否为OpenVINO格式
   - 检查文件权限

2. **推理设备不可用**
   - 检查OpenVINO是否支持指定的设备
   - 尝试使用 "CPU" 作为备用设备

3. **图像订阅失败**
   - 检查图像话题名称是否正确
   - 确认相机节点是否正在发布图像

### 调试信息

启用调试输出：

```bash
ros2 run object_detection_openvino object_detection_openvino_node --ros-args --log-level debug
```

## 许可证

TODO: 添加许可证信息
