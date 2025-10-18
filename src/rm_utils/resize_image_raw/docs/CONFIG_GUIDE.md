# 配置文件使用指南

## 配置文件说明

本包支持通过 YAML 配置文件来设置参数，使得参数管理更加方便和可维护。

## 预设配置文件

### 1. resize_params.yaml（默认配置）
```yaml
resize_node:
  ros__parameters:
    scale_width: 0.5
    scale_height: 0.5
    interpolation: 1  # LINEAR
    input_image_topic: "image"
    input_camera_info_topic: "camera_info"
    output_image_topic: "resized/image"
    output_camera_info_topic: "resized/camera_info"
```
**用途**: 通用场景，平衡速度和质量

### 2. downscale_half.yaml（高质量缩小）
```yaml
resize_node:
  ros__parameters:
    scale_width: 0.5
    scale_height: 0.5
    interpolation: 3  # AREA
```
**用途**: 缩小图像时获得最佳质量

### 3. upscale_double.yaml（高质量放大）
```yaml
resize_node:
  ros__parameters:
    scale_width: 2.0
    scale_height: 2.0
    interpolation: 2  # CUBIC
```
**用途**: 放大图像时获得最佳质量

### 4. fast_resize.yaml（快速处理）
```yaml
resize_node:
  ros__parameters:
    scale_width: 0.5
    scale_height: 0.5
    interpolation: 0  # NEAREST
```
**用途**: 需要最快速度时使用，质量较低

### 5. custom_aspect.yaml（自定义宽高比）
```yaml
resize_node:
  ros__parameters:
    scale_width: 0.75
    scale_height: 0.5
    interpolation: 1  # LINEAR
```
**用途**: 改变图像宽高比

## 使用配置文件的方法

### 方法 1: 使用预设配置文件

```bash
# 使用默认配置
ros2 launch resize_image_raw resize_with_config.launch.py

# 使用缩小一半配置
ros2 launch resize_image_raw downscale_half.launch.py

# 使用放大两倍配置
ros2 launch resize_image_raw upscale_double.launch.py
```

### 方法 2: 指定自定义配置文件

```bash
ros2 launch resize_image_raw resize_with_config.launch.py \
    config_file:=/path/to/your/custom_config.yaml
```

### 方法 3: 使用配置文件并覆盖话题

```bash
ros2 launch resize_image_raw resize_with_config.launch.py \
    config_file:=/path/to/config.yaml \
    image_topic:=/my_camera/image_raw \
    resized_image_topic:=/my_resized/image
```

### 方法 4: 不使用配置文件（使用命令行参数）

```bash
ros2 launch resize_image_raw resize_with_config.launch.py \
    use_config_file:=false \
    scale_width:=0.75 \
    scale_height:=0.75
```

## 创建自己的配置文件

1. 创建新的 YAML 文件，例如 `my_config.yaml`:

```yaml
resize_node:
  ros__parameters:
    # 缩放比例
    scale_width: 0.3    # 宽度缩放到 30%
    scale_height: 0.3   # 高度缩放到 30%
    
    # 插值方法
    # 0: NEAREST - 最快，质量最低
    # 1: LINEAR  - 平衡速度和质量（推荐）
    # 2: CUBIC   - 较慢，质量高（适合放大）
    # 3: AREA    - 适中，质量最高（适合缩小）
    interpolation: 3
    
    # 话题名称配置（新功能！）
    input_image_topic: "/my_camera/image_raw"
    input_camera_info_topic: "/my_camera/camera_info"
    output_image_topic: "/my_camera/resized/image_raw"
    output_camera_info_topic: "/my_camera/resized/camera_info"
```

2. 使用你的配置文件：

```bash
ros2 launch resize_image_raw resize_with_config.launch.py \
    config_file:=/path/to/my_config.yaml
```

## 插值方法选择指南

| 插值方法 | 代码 | 速度 | 质量 | 适用场景 |
|---------|------|------|------|---------|
| NEAREST | 0 | ⭐⭐⭐⭐⭐ | ⭐ | 需要最快速度，对质量要求不高 |
| LINEAR  | 1 | ⭐⭐⭐⭐ | ⭐⭐⭐ | 通用场景，平衡速度和质量 |
| CUBIC   | 2 | ⭐⭐ | ⭐⭐⭐⭐ | 放大图像，需要高质量 |
| AREA    | 3 | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | 缩小图像，需要最佳质量 |

## 常见配置场景

### 场景 1: 降低分辨率以提高处理速度

用于深度学习推理前的预处理：

```yaml
resize_node:
  ros__parameters:
    scale_width: 0.5
    scale_height: 0.5
    interpolation: 3  # AREA 提供最佳质量
```

### 场景 2: 实时视频流快速处理

```yaml
resize_node:
  ros__parameters:
    scale_width: 0.25
    scale_height: 0.25
    interpolation: 0  # NEAREST 最快速度
```

### 场景 3: 图像显示优化

```yaml
resize_node:
  ros__parameters:
    scale_width: 0.5
    scale_height: 0.5
    interpolation: 1  # LINEAR 平衡速度和质量
```

### 场景 4: 不同宽高比的屏幕适配

```yaml
resize_node:
  ros__parameters:
    scale_width: 0.8
    scale_height: 0.6
    interpolation: 1  # LINEAR
```

## 配置文件路径

安装后的配置文件位于：
```
/home/amatrix/Dart_2026_ws/install/resize_image_raw/share/resize_image_raw/config/
```

源代码中的配置文件位于：
```
/home/amatrix/Dart_2026_ws/src/rm_utils/resize_image_raw/config/
```

## 参数验证

节点启动时会验证参数：

- `scale_width` 和 `scale_height` 必须 > 0
- `interpolation` 必须在 [0, 3] 范围内
- 如果参数无效，节点会报错并退出

## 动态参数更新

ROS2 支持运行时更新参数（未来可以添加）：

```bash
ros2 param set /resize_node scale_width 0.75
ros2 param set /resize_node scale_height 0.75
```

注意：当前版本需要重启节点才能应用新参数。

## 话题名称配置（新功能）

### 为什么需要在配置文件中设置话题名称？

在 ROS2 中，有两种方式配置话题名称：

1. **使用 remapping（重映射）**: 在 launch 文件中使用 `-r` 参数
2. **使用参数**: 在配置文件或命令行中设置参数

使用参数的优势：
- ✅ 所有配置集中在一个文件中，易于管理
- ✅ 不需要修改 launch 文件
- ✅ 可以为不同的相机创建不同的配置文件
- ✅ 配置文件可以版本控制和共享

### 话题名称参数说明

```yaml
resize_node:
  ros__parameters:
    # 输入话题
    input_image_topic: "/camera_left/image_raw"           # 输入图像
    input_camera_info_topic: "/camera_left/camera_info"   # 输入相机信息
    
    # 输出话题
    output_image_topic: "/camera_left/resized/image_raw"        # 输出图像
    output_camera_info_topic: "/camera_left/resized/camera_info" # 输出相机信息
```

### 预设相机配置

#### camera_left.yaml - 左相机
```yaml
resize_node:
  ros__parameters:
    scale_width: 0.5
    scale_height: 0.5
    interpolation: 3
    input_image_topic: "/camera_left/image_raw"
    input_camera_info_topic: "/camera_left/camera_info"
    output_image_topic: "/camera_left/resized/image_raw"
    output_camera_info_topic: "/camera_left/resized/camera_info"
```

#### camera_right.yaml - 右相机
```yaml
resize_node:
  ros__parameters:
    scale_width: 0.5
    scale_height: 0.5
    interpolation: 3
    input_image_topic: "/camera_right/image_raw"
    input_camera_info_topic: "/camera_right/camera_info"
    output_image_topic: "/camera_right/resized/image_raw"
    output_camera_info_topic: "/camera_right/resized/camera_info"
```

### 使用示例

#### 单个相机
```bash
# 使用左相机配置
ros2 launch resize_image_raw resize_with_config.launch.py \
    config_file:=$(ros2 pkg prefix resize_image_raw)/share/resize_image_raw/config/camera_left.yaml
```

#### 双目相机（两个节点）
```bash
# 使用预设的双目相机 launch 文件
ros2 launch resize_image_raw stereo_cameras_with_config.launch.py
```

这将启动两个节点：
- `resize_node_left`: 处理左相机图像
- `resize_node_right`: 处理右相机图像

### 话题配置最佳实践

1. **使用绝对路径**: 话题名称以 `/` 开头，避免命名空间问题
2. **保持一致性**: 输出话题应该反映输入话题的结构
3. **描述性命名**: 使用清晰的名称，如 `resized`、`scaled` 等
4. **避免冲突**: 确保不同节点的输出话题不重复

### 常见话题配置场景

#### 场景 1: 单个相机
```yaml
input_image_topic: "/camera/image_raw"
output_image_topic: "/camera/resized/image_raw"
```

#### 场景 2: 左右双目相机
```yaml
# 左相机
input_image_topic: "/stereo/left/image_raw"
output_image_topic: "/stereo/left/resized/image_raw"

# 右相机
input_image_topic: "/stereo/right/image_raw"
output_image_topic: "/stereo/right/resized/image_raw"
```

#### 场景 3: 多个处理阶段
```yaml
# 第一阶段：原始图像 -> 缩小
input_image_topic: "/camera/image_raw"
output_image_topic: "/camera/small/image_raw"

# 第二阶段：缩小图像 -> 更小（链式处理）
input_image_topic: "/camera/small/image_raw"
output_image_topic: "/camera/tiny/image_raw"
```

#### 场景 4: 不同分辨率输出
```yaml
# 节点1：生成中等分辨率
scale_width: 0.5
output_image_topic: "/camera/medium/image_raw"

# 节点2：生成低分辨率
scale_width: 0.25
output_image_topic: "/camera/low/image_raw"
```

### 检查话题配置

节点启动后，会打印话题配置信息：

```
[INFO] [resize_node]: Topic configuration:
  Input image: /camera_left/image_raw
  Input camera_info: /camera_left/camera_info
  Output image: /camera_left/resized/image_raw
  Output camera_info: /camera_left/resized/camera_info
```

你也可以使用 ROS2 命令检查：

```bash
# 查看节点参数
ros2 param list /resize_node

# 查看特定参数
ros2 param get /resize_node input_image_topic
ros2 param get /resize_node output_image_topic

# 查看所有话题
ros2 topic list
```
