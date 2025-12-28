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

#### 方法4：启用调试图像

```bash
# 在启动文件中启用调试图像
ros2 launch object_detection_openvino object_detection_openvino.launch.py \
    publish_debug_image:=true \
    debug_image_topic:="/my_debug_image"

# 或通过命令行参数
ros2 run object_detection_openvino object_detection_openvino_node \
    --ros-args \
    -p publish_debug_image:=true \
    -p debug_image_topic:="/detector/debug_image" \
    -r image_raw:="/camera/image_raw"
```

## 话题接口

### 订阅话题

- `image_raw` (sensor_msgs/Image): 输入图像

### 发布话题

- `/detector/target2d_array` (rm_interfaces/Target2DArray): 检测结果
- `/detector/debug_image` (sensor_msgs/Image): 带有边界框和标签的调试图像（可选）

## 参数

| 参数名称 | 类型 | 默认值 | 描述 |
|---------|------|--------|------|
| mode | string | "armor" | 检测模式 |
| input_width | int | 640 | 模型输入图像宽度 |
| input_height | int | 384 | 模型输入图像高度 |
| score_threshold | double | 0.5 | 检测置信度阈值 |
| nms_threshold | double | 0.4 | 非最大值抑制阈值 |
| xml_path | string | "/path/to/model.xml" | OpenVINO模型XML文件路径 |
| bin_path | string | "/path/to/model.bin" | OpenVINO模型BIN文件路径 |
| device | string | "CPU" | OpenVINO推理设备 |
| image_topic | string | "image_raw" | 订阅的图像话题名称 |
| detection_topic | string | "/detector/target2d_array" | 发布的检测结果话题名称 |
| debug_image_topic | string | "/detector/debug_image" | 发布的调试图像话题名称 |
| publish_debug_image | bool | false | 是否发布带有边界框和标签的调试图像 |
| roi_mode | string | "full" | ROI（感兴趣区域）模式："full" 使用整个图像作为输入；"center" 使用图像中心的指定大小区域作为输入 |
| roi_width | int | 1280 | 当 roi_mode 为 "center" 时的ROI宽度（对 "full" 模式无效） |
| roi_height | int | 1024 | 当 roi_mode 为 "center" 时的ROI高度（对 "full" 模式无效） |
| center_mode | string | "auto" | ROI中心坐标的解释模式："auto" 自动检测格式；"pixel" 像素坐标；"percent" 百分比；"fraction" 分数 |
| center_x | double | -1.0 | 当 roi_mode 为 "center" 时的ROI中心X坐标（根据center_mode解释） |
| center_y | double | -1.0 | 当 roi_mode 为 "center" 时的ROI中心Y坐标（根据center_mode解释） |

## ROI（感兴趣区域）功能

该节点支持两种ROI策略，允许你选择图像的哪一部分作为模型的输入：

### 1. 全图模式（默认）

```yaml
roi_mode: "full"
```

使用完整的输入图像进行检测。这是默认行为，保持与之前版本的兼容性。

### 2. 中心裁剪模式

```yaml
roi_mode: "center"
roi_width: 1280
roi_height: 1024
center_mode: "auto"  # 可选：auto, pixel, percent, fraction
center_x: -1.0
center_y: -1.0
```

此外可以通过 `center_x` / `center_y` 指定 ROI 的中心坐标，支持多种格式：

```yaml
roi_mode: "center"
roi_width: 800
roi_height: 600
center_mode: "auto"
center_x: -1.0   # -1 表示使用图像中心
center_y: -1.0
```

#### `center_mode` 参数说明

`center_mode` 控制如何解释 `center_x` 和 `center_y` 的值：

- `"auto"`（默认）：自动检测格式（向后兼容）
  - `-1`：自动使用图像中心
  - `0.0-1.0`：分数（相对于图像尺寸）
  - `0-100`：百分比
  - `>100`：像素坐标
- `"pixel"`：直接像素坐标
- `"percent"`：百分比（0-100）
- `"fraction"`：分数（0.0-1.0）

#### `center_x` / `center_y` 格式说明

根据 `center_mode` 的不同，`center_x` 与 `center_y` 的解释方式如下：

**auto 模式（默认）：**
- `-1`：自动使用图像中心（默认）。
- `0.0` 到 `1.0`：表示相对于图像尺寸的分数（例如 `0.5` 表示图像中心）。
- `0` 到 `100`：表示百分比（例如 `50` 表示图像中心）。
- 大于 `100`：表示像素坐标（直接使用）。

**pixel 模式：**
- 直接像素坐标值（例如 `320` 表示图像宽度320像素处）。

**percent 模式：**
- `0-100` 的百分比值（例如 `50` 表示图像中心）。

**fraction 模式：**
- `0.0-1.0` 的分数（例如 `0.5` 表示图像中心）。

节点会将计算出的中心坐标裁剪到图像范围内，并在必要时给出警告，同时在日志中输出解释信息。

#### 使用示例

```bash
# 使用百分比指定中心点（auto模式自动检测）
ros2 launch object_detection_openvino object_detection_openvino.launch.py \
    center_x:=50 center_y:=50 roi_mode:=center roi_width:=320 roi_height:=240

# 明确指定百分比模式
ros2 launch object_detection_openvino object_detection_openvino.launch.py \
    center_mode:=percent center_x:=50 center_y:=50 roi_mode:=center roi_width:=320 roi_height:=240

# 使用分数指定中心点
ros2 launch object_detection_openvino object_detection_openvino.launch.py \
    center_mode:=fraction center_x:=0.5 center_y:=0.5 roi_mode:=center roi_width:=320 roi_height:=240

# 使用像素坐标
ros2 launch object_detection_openvino object_detection_openvino.launch.py \
    center_mode:=pixel center_x:=320 center_y:=240 roi_mode:=center roi_width:=320 roi_height:=240
```

示例：指定自定义中心点（例如 `(100,150)`）：

```bash
ros2 launch object_detection_openvino object_detection_openvino.launch.py \
    center_mode:=pixel center_x:=100 center_y:=150 roi_mode:=center roi_width:=320 roi_height:=240
```


从输入图像的中心提取指定大小的区域进行检测。这在以下场景中很有用：

- 目标通常出现在图像中心
- 需要减少推理时间（较小的ROI可以提高处理速度）
- 想要忽略图像边缘的干扰信息

### ROI 可视化

当启用 `publish_debug_image` 时，调试图像会显示：

- **青色矩形框**：标识当前使用的ROI区域
- **ROI尺寸标签**：显示ROI的实际宽度和高度
- **检测框**：所有检测框的坐标都已映射回完整图像坐标系

### ROI 使用示例

```bash
# 使用中心裁剪模式启动节点
ros2 run object_detection_openvino object_detection_openvino_node \
    --ros-args \
    -p roi_mode:="center" \
    -p roi_width:=800 \
    -p roi_height:=600 \
    -p publish_debug_image:=true

# 或在 params.yaml 中配置
object_detection_openvino_node:
  ros__parameters:
    roi_mode: "center"
    roi_width: 1280
    roi_height: 1024
    publish_debug_image: true
```

### 注意事项

- 如果指定的ROI尺寸大于输入图像，ROI会自动裁剪到图像边界
- 所有发布的检测结果（包括在 `Target2DArray` 消息中的坐标）都已映射回完整图像的坐标系
- ROI裁剪不影响模型输入尺寸（`input_width` 和 `input_height`），ROI图像会被缩放到模型所需尺寸

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

## 重要修复说明

该包已针对多输出 YOLO 模型进行了以下关键修复：

1. **多输出模型支持**: 修改了 OpenVINO 预处理器以支持具有多个输出层的模型
2. **输入尺寸匹配**: 确保配置的输入尺寸与模型要求一致（本例中为 640x384）
3. **Warmup 阶段**: 在推理预热阶段添加了 `wait()` 调用以确保正确初始化
4. **Letterbox 优化**: 修改为创建图像副本而不是修改原始图像

## 故障排除

### 常见问题

1. **模型加载失败**
   - 检查模型文件路径是否正确（使用绝对路径）
   - 确认模型文件格式是否为OpenVINO格式（.xml 和 .bin 文件）
   - 检查文件权限
   - 验证路径中的用户名是否正确

2. **推理设备不可用**
   - 检查OpenVINO是否支持指定的设备
   - 尝试使用 "CPU" 作为备用设备
   - 对于 GPU，确保已安装相应的 OpenVINO GPU 插件

3. **图像订阅失败**
   - 检查图像话题名称是否正确：`ros2 topic list`
   - 确认相机节点是否正在发布图像：`ros2 topic hz /image_raw`
   - 检查图像编码格式是否为 BGR8

4. **模型输入尺寸不匹配**
   - 使用提供的 `check_model_outputs.cpp` 工具检查模型的实际输入尺寸
   - 更新 `params.yaml` 中的 `input_width` 和 `input_height` 以匹配模型

5. **节点运行一段时间后崩溃**
   - 这可能是由于视频源问题或内存相关问题
   - 尝试使用不同的视频源进行测试
   - 检查系统资源使用情况（内存、CPU）

### 调试信息

启用调试输出：

```bash
ros2 run object_detection_openvino object_detection_openvino_node --ros-args --log-level debug
```

### 检查模型信息

编译并运行模型检查工具：

```bash
cd src/object_detection_openvino/scripts
g++ -o check_model_outputs check_model_outputs.cpp -I/usr/include/openvino -lopenvino
./check_model_outputs
```

## 性能说明

- 当前版本可成功处理视频流并执行目标检测
- 节点已在 CPU 设备上测试，能够以约 30 FPS 的速度处理图像
- 使用 GPU 可以获得更好的性能

## 许可证

TODO: 添加许可证信息
