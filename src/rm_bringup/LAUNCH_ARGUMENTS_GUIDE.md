# Launch参数传递功能使用指南

## 概述

现在 `rm_bringup` 包支持在配置文件中为每个启动节点指定launch参数！这意味着你可以轻松地自定义每个子系统的配置文件路径和其他参数。

## 🎯 主要功能

- ✅ 为每个阶段的节点传递自定义参数
- ✅ 支持配置文件路径自定义
- ✅ 支持各种launch参数（模式、阈值等）
- ✅ 参数会在启动时显示在日志中

## 📝 如何使用

### 1. 基本语法

在配置文件中添加 `launch_arguments` 字段：

```yaml
stage1:
  camera:
    package: 'ros2_mindvision_camera'
    launch_file: 'dual_camera_launch.py'
    enabled: true
    launch_arguments:              # 添加这个字段
      params_file: '/path/to/camera_config.yaml'
      use_sensor_data_qos: 'false'
```

### 2. 实际示例

#### 示例1：使用自定义相机配置

```yaml
stage1:
  camera:
    package: 'ros2_mindvision_camera'
    launch_file: 'dual_camera_launch.py'
    enabled: true
    launch_arguments:
      params_file: '/home/amatrix/Dart_2026_ws/my_custom_camera_config.yaml'
```

#### 示例2：自定义目标检测参数

```yaml
stage2:
  object_detection:
    package: 'object_detection_openvino'
    launch_file: 'object_detection_openvino.launch.py'
    enabled: true
    launch_arguments:
      mode: 'armor'
      input_width: '640'
      input_height: '640'
      score_threshold: '0.7'    # 提高置信度阈值
      nms_threshold: '0.4'
```

#### 示例3：为所有阶段指定配置文件

```yaml
stage1:
  camera:
    package: 'ros2_mindvision_camera'
    launch_file: 'dual_camera_launch.py'
    enabled: true
    launch_arguments:
      params_file: '/home/user/my_camera.yaml'

stage2:
  object_detection:
    package: 'object_detection_openvino'
    launch_file: 'object_detection_openvino.launch.py'
    enabled: true
    launch_arguments:
      params_file: '/home/user/my_detection.yaml'
  
  stereo_image_proc:
    package: 'stereo_image_proc_wrapper'
    launch_file: 'stereo_image_proc.launch.py'
    enabled: true
    launch_arguments:
      config_file: '/home/user/my_stereo.yaml'

stage3:
  distance_estimator:
    package: 'stereo_distance_estimator'
    launch_file: 'stereo_distance_estimator_config.launch.py'
    enabled: true
    launch_arguments:
      config_file: '/home/user/my_distance.yaml'
```

## 📋 可用参数列表

### Stage 1 - 相机驱动 (dual_camera_launch.py)

| 参数名 | 类型 | 说明 | 示例 |
|--------|------|------|------|
| `params_file` | string | 相机参数配置文件路径 | `'/path/to/camera_params.yaml'` |
| `use_sensor_data_qos` | string | 是否使用SensorDataQoS | `'true'` 或 `'false'` |

### Stage 2 - 目标检测 (object_detection_openvino.launch.py)

| 参数名 | 类型 | 说明 | 示例 |
|--------|------|------|------|
| `params_file` | string | ROS2参数文件路径 | `'/path/to/params.yaml'` |
| `mode` | string | 检测模式 | `'armor'`, `'rune'` |
| `input_width` | string | 输入图像宽度 | `'640'`, `'1280'` |
| `input_height` | string | 输入图像高度 | `'640'`, `'1280'` |
| `score_threshold` | string | 置信度阈值 | `'0.5'`, `'0.7'` |
| `nms_threshold` | string | NMS阈值 | `'0.4'`, `'0.5'` |

### Stage 2 - 立体图像处理 (stereo_image_proc.launch.py)

| 参数名 | 类型 | 说明 | 示例 |
|--------|------|------|------|
| `config_file` | string | 配置文件路径 | `'/path/to/stereo_params.yaml'` |
| `left_namespace` | string | 左相机命名空间 | `'camera_left'` |
| `right_namespace` | string | 右相机命名空间 | `'camera_right'` |

### Stage 3 - 距离估计 (stereo_distance_estimator_config.launch.py)

| 参数名 | 类型 | 说明 | 示例 |
|--------|------|------|------|
| `config_file` | string | 配置文件路径 | `'/path/to/estimator.yaml'` |

## 💡 使用技巧

### 技巧1：使用相对路径
如果配置文件在工作空间内，使用绝对路径：
```yaml
launch_arguments:
  config_file: '/home/amatrix/Dart_2026_ws/src/my_pkg/config/my_config.yaml'
```

### 技巧2：创建多个配置文件
为不同场景创建不同的配置文件：
```bash
config/
├── launch_config.yaml              # 默认配置
├── launch_config_debug.yaml        # 调试配置
├── launch_config_competition.yaml  # 比赛配置
└── launch_config_test.yaml         # 测试配置
```

使用时指定：
```bash
ros2 launch rm_bringup system_bringup.launch.py config_file:=/path/to/launch_config_competition.yaml
```

### 技巧3：参数验证
启动时查看日志输出，确认参数正确传递：
```
[INFO] 为 object_detection_openvino/object_detection_openvino.launch.py 传递参数: [('mode', 'armor'), ('score_threshold', '0.7')]
```

### 技巧4：注释不用的参数
不需要的参数可以注释掉：
```yaml
launch_arguments:
  params_file: '/path/to/config.yaml'
  # mode: 'armor'              # 注释掉，使用默认值
  # score_threshold: '0.5'     # 注释掉，使用默认值
```

## 🔍 调试

### 查看传递的参数
启动时会打印传递的参数：
```
[INFO] 为 ros2_mindvision_camera/dual_camera_launch.py 传递参数: [('params_file', '/path/to/config.yaml')]
```

### 常见问题

**Q: 参数没有生效？**
A: 检查：
1. 参数名是否正确（区分大小写）
2. 参数值是否为字符串格式（用引号）
3. 路径是否存在且正确

**Q: 如何知道某个launch文件支持哪些参数？**
A: 查看该launch文件的源代码，查找 `DeclareLaunchArgument` 声明

**Q: 所有参数都必须提供吗？**
A: 不是，只提供需要自定义的参数即可，其他使用默认值

## 📚 示例文件

包中提供了以下示例配置文件：

1. `config/launch_config.yaml` - 默认配置（参数被注释）
2. `config/launch_config_example.yaml` - 详细示例和说明
3. `config/launch_config_with_params.yaml` - 带参数的实际示例

## 🚀 快速开始

1. 复制示例配置：
```bash
cp src/rm_bringup/config/launch_config_with_params.yaml my_config.yaml
```

2. 编辑配置文件，修改参数

3. 使用自定义配置启动：
```bash
ros2 launch rm_bringup system_bringup.launch.py config_file:=my_config.yaml
```

---

**更新日期**: 2025-10-19
**功能版本**: v1.1
