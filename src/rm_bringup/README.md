# RM Bringup Package

## 简介

`rm_bringup` 是用于启动整个系统的ROS2包，它按照预定义的顺序启动各个子系统。

## 启动顺序

系统按照以下三个阶段顺序启动：

### Stage 1: 相机驱动

- 启动双相机驱动节点
- 包: `ros2_mindvision_camera`
- Launch文件: `dual_camera_launch.py`

### Stage 2: 并行启动（等待Stage 1完成后）

- **目标检测**: OpenVINO目标检测节点
  - 包: `object_detection_openvino`
  - Launch文件: `object_detection_openvino.launch.py`
  
- **立体图像处理**: 立体图像处理节点
  - 包: `stereo_image_proc_wrapper`
  - Launch文件: `stereo_image_proc.launch.py`

### Stage 3: 距离估计（等待Stage 2完成后）

- 启动立体距离估计节点
- 包: `stereo_distance_estimator`
- Launch文件: `stereo_distance_estimator_config.launch.py`

## 使用方法

### 1. 编译包

```bash
cd /home/amatrix/Dart_2026_ws
colcon build --packages-select rm_bringup
source install/setup.bash
```

### 2. 启动系统

使用默认配置：

```bash
ros2 launch rm_bringup system_bringup.launch.py
```

使用自定义配置文件：

```bash
ros2 launch rm_bringup system_bringup.launch.py system_config_file:=/path/to/your/config.yaml
```

### 3. 调整启动延迟

你可以通过参数调整各个阶段之间的启动延迟（单位：秒）：

```bash
# 使用自定义延迟时间
ros2 launch rm_bringup system_bringup.launch.py stage2_delay:=8.0 stage3_delay:=15.0
```

参数说明：

- `stage1_delay`: Stage 1启动延迟（默认: 0.0秒）
- `stage2_delay`: Stage 2启动延迟，等待Stage 1启动完成（默认: 5.0秒）
- `stage3_delay`: Stage 3启动延迟，等待Stage 2启动完成（默认: 10.0秒）

**注意**: 延迟时间是从系统启动开始计算的绝对时间，不是相对于上一阶段的时间。因此 `stage3_delay` 应该大于 `stage2_delay`。

### 4. 使用快速启动脚本

为了方便使用，提供了一个启动脚本：

```bash
# 使用默认配置
./src/rm_bringup/scripts/start_system.sh

# 使用自定义配置文件
./src/rm_bringup/scripts/start_system.sh -c /path/to/config.yaml

# 自定义延迟时间
./src/rm_bringup/scripts/start_system.sh -2 8.0 -3 15.0

# 查看帮助
./src/rm_bringup/scripts/start_system.sh -h
```

## 配置文件

配置文件位于 `config/launch_config.yaml`，你可以通过修改此文件来：

1. **启用/禁用某个阶段的节点**：设置 `enabled: true/false`
2. **修改启动文件**：修改 `package` 和 `launch_file` 字段
3. **传递launch参数**：在 `launch_arguments` 下添加参数（✨新功能！）

### 传递Launch参数

现在你可以在配置文件中为每个节点指定launch参数，例如配置文件路径：

```yaml
stage1:
  camera:
    package: 'ros2_mindvision_camera'
    launch_file: 'dual_camera_launch.py'
    enabled: true
    launch_arguments:
      params_file: '/path/to/custom/camera_params.yaml'
      use_sensor_data_qos: 'false'
```

### 可用的Launch参数

#### Stage 1 - 相机驱动

- `params_file`: 相机参数配置文件路径
- `use_sensor_data_qos`: 是否使用SensorDataQoS（'true'/'false'）

#### Stage 2 - 目标检测

- `mode`: 检测模式（'armor', 'rune'等）
- `input_width`: 输入图像宽度
- `input_height`: 输入图像高度
- `score_threshold`: 置信度阈值
- `nms_threshold`: NMS阈值

#### Stage 2 - 立体图像处理

- `config_file`: 配置文件路径
- `left_namespace`: 左相机命名空间
- `right_namespace`: 右相机命名空间

#### Stage 3 - 距离估计

- `config_file`: 配置文件路径

### 配置文件示例

```yaml
# 阶段1：相机驱动
stage1:
  camera:
    package: 'ros2_mindvision_camera'
    launch_file: 'dual_camera_launch.py'
    enabled: true

# 阶段2：并行启动
stage2:
  object_detection:
    package: 'object_detection_openvino'
    launch_file: 'object_detection_openvino.launch.py'
    enabled: true
  
  stereo_image_proc:
    package: 'stereo_image_proc_wrapper'
    launch_file: 'stereo_image_proc.launch.py'
    enabled: true

# 阶段3：距离估计
stage3:
  distance_estimator:
    package: 'stereo_distance_estimator'
    launch_file: 'stereo_distance_estimator_config.launch.py'
    enabled: true
```

## 自定义配置

如果你想使用不同的launch文件，可以：

1. 复制配置文件到新位置：

    ```bash
    cp config/launch_config.yaml my_custom_config.yaml
    ```

2. 编辑配置文件，修改相应的包名和launch文件名

3. 使用自定义配置启动：

```bash
ros2 launch rm_bringup system_bringup.launch.py system_config_file:=/path/to/my_custom_config.yaml
```

## 调试

如果某个阶段的节点启动失败，可以：

1. 在配置文件中禁用该阶段的节点（设置 `enabled: false`）
2. 单独启动该节点进行调试
3. 检查日志输出，查看具体的错误信息

## 注意事项

- 确保所有依赖的包都已正确编译和安装
- 各个阶段的启动顺序是固定的，确保前序节点正常运行后才会启动后续节点
- Stage 2的两个节点会并行启动，但只有当所有Stage 2的节点都完成后，才会启动Stage 3

## 依赖包

- `ros2_mindvision_camera`
- `object_detection_openvino`
- `stereo_image_proc_wrapper`
- `stereo_distance_estimator`
- `launch`
- `launch_ros`
