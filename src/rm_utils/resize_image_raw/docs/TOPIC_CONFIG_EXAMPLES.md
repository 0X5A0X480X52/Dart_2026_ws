# 话题配置使用示例

本文档提供了使用配置文件配置话题名称的详细示例。

## 快速开始

### 1. 使用默认话题名称

默认话题使用相对名称，可以通过 remapping 或 namespace 调整：

```bash
ros2 run resize_image_raw resize_node_exe
```

订阅：
- `image`
- `camera_info`

发布：
- `resized/image`
- `resized/camera_info`

### 2. 使用配置文件指定完整话题路径

创建配置文件 `my_camera.yaml`:

```yaml
resize_node:
  ros__parameters:
    scale_width: 0.5
    scale_height: 0.5
    interpolation: 3
    input_image_topic: "/camera/image_raw"
    input_camera_info_topic: "/camera/camera_info"
    output_image_topic: "/camera/resized/image_raw"
    output_camera_info_topic: "/camera/resized/camera_info"
```

运行：

```bash
ros2 run resize_image_raw resize_node_exe --ros-args \
    --params-file my_camera.yaml
```

## 实际应用场景

### 场景 1: 双目视觉系统

为左右相机分别创建配置文件并启动节点。

**left_camera.yaml:**
```yaml
resize_node:
  ros__parameters:
    scale_width: 0.5
    scale_height: 0.5
    interpolation: 3
    input_image_topic: "/stereo/left/image_raw"
    input_camera_info_topic: "/stereo/left/camera_info"
    output_image_topic: "/stereo/left/resized/image_raw"
    output_camera_info_topic: "/stereo/left/resized/camera_info"
```

**right_camera.yaml:**
```yaml
resize_node:
  ros__parameters:
    scale_width: 0.5
    scale_height: 0.5
    interpolation: 3
    input_image_topic: "/stereo/right/image_raw"
    input_camera_info_topic: "/stereo/right/camera_info"
    output_image_topic: "/stereo/right/resized/image_raw"
    output_camera_info_topic: "/stereo/right/resized/camera_info"
```

**启动方式 1 - 分别启动:**
```bash
# 终端 1: 左相机
ros2 run resize_image_raw resize_node_exe \
    --ros-args --params-file left_camera.yaml

# 终端 2: 右相机
ros2 run resize_image_raw resize_node_exe \
    --ros-args --params-file right_camera.yaml
```

**启动方式 2 - 使用预设 launch 文件:**
```bash
ros2 launch resize_image_raw stereo_cameras_with_config.launch.py
```

### 场景 2: 多分辨率输出

为同一个相机生成多个不同分辨率的输出。

**medium_res.yaml:**
```yaml
resize_node:
  ros__parameters:
    scale_width: 0.5
    scale_height: 0.5
    interpolation: 1
    input_image_topic: "/camera/image_raw"
    input_camera_info_topic: "/camera/camera_info"
    output_image_topic: "/camera/medium/image_raw"
    output_camera_info_topic: "/camera/medium/camera_info"
```

**low_res.yaml:**
```yaml
resize_node:
  ros__parameters:
    scale_width: 0.25
    scale_height: 0.25
    interpolation: 0  # 快速处理
    input_image_topic: "/camera/image_raw"
    input_camera_info_topic: "/camera/camera_info"
    output_image_topic: "/camera/low/image_raw"
    output_camera_info_topic: "/camera/low/camera_info"
```

启动：
```bash
# 终端 1: 中等分辨率
ros2 run resize_image_raw resize_node_exe \
    --ros-args --params-file medium_res.yaml

# 终端 2: 低分辨率
ros2 run resize_image_raw resize_node_exe \
    --ros-args --params-file low_res.yaml
```

### 场景 3: 集成到现有系统

假设你的系统已经有以下话题：
- `/hik_camera/image_raw`
- `/hik_camera/camera_info`

想要输出到：
- `/vision/input/image_raw`
- `/vision/input/camera_info`

**hik_to_vision.yaml:**
```yaml
resize_node:
  ros__parameters:
    scale_width: 0.5
    scale_height: 0.5
    interpolation: 3
    input_image_topic: "/hik_camera/image_raw"
    input_camera_info_topic: "/hik_camera/camera_info"
    output_image_topic: "/vision/input/image_raw"
    output_camera_info_topic: "/vision/input/camera_info"
```

启动：
```bash
ros2 run resize_image_raw resize_node_exe \
    --ros-args --params-file hik_to_vision.yaml
```

### 场景 4: 开发和生产环境切换

开发环境使用录制的 bag 文件，生产环境使用真实相机。

**dev_config.yaml:**
```yaml
resize_node:
  ros__parameters:
    scale_width: 0.5
    scale_height: 0.5
    interpolation: 1
    input_image_topic: "/playback/camera/image_raw"
    input_camera_info_topic: "/playback/camera/camera_info"
    output_image_topic: "/processed/image_raw"
    output_camera_info_topic: "/processed/camera_info"
```

**prod_config.yaml:**
```yaml
resize_node:
  ros__parameters:
    scale_width: 0.5
    scale_height: 0.5
    interpolation: 3
    input_image_topic: "/camera/image_raw"
    input_camera_info_topic: "/camera/camera_info"
    output_image_topic: "/processed/image_raw"
    output_camera_info_topic: "/processed/camera_info"
```

切换环境只需要更改配置文件：
```bash
# 开发环境
ros2 run resize_image_raw resize_node_exe \
    --ros-args --params-file dev_config.yaml

# 生产环境
ros2 run resize_image_raw resize_node_exe \
    --ros-args --params-file prod_config.yaml
```

## 对比：参数 vs Remapping

### 使用参数（配置文件）
```yaml
# config.yaml
resize_node:
  ros__parameters:
    input_image_topic: "/camera/image_raw"
    output_image_topic: "/camera/resized/image_raw"
```

```bash
ros2 run resize_image_raw resize_node_exe \
    --ros-args --params-file config.yaml
```

**优点:**
- ✅ 配置集中
- ✅ 易于版本控制
- ✅ 可读性强
- ✅ 易于分享

### 使用 Remapping
```bash
ros2 run resize_image_raw resize_node_exe --ros-args \
    -r image:=/camera/image_raw \
    -r camera_info:=/camera/camera_info \
    -r resized/image:=/camera/resized/image_raw \
    -r resized/camera_info:=/camera/resized/camera_info
```

**优点:**
- ✅ ROS2 标准方式
- ✅ 灵活，无需修改配置文件
- ✅ 适合临时测试

**缺点:**
- ❌ 命令行很长
- ❌ 难以维护
- ❌ 容易出错

## 推荐做法

1. **为每个相机/传感器创建独立配置文件**
   ```
   config/
   ├── camera_left.yaml
   ├── camera_right.yaml
   ├── camera_front.yaml
   └── camera_back.yaml
   ```

2. **使用描述性的话题名称**
   ```yaml
   input_image_topic: "/camera_left/image_raw"  # ✅ 清晰
   input_image_topic: "/cam1/img"              # ❌ 不清晰
   ```

3. **保持输入输出话题的一致性**
   ```yaml
   input_image_topic: "/camera/image_raw"
   output_image_topic: "/camera/resized/image_raw"  # ✅ 结构一致
   ```

4. **使用绝对路径避免命名空间问题**
   ```yaml
   input_image_topic: "/camera/image_raw"   # ✅ 绝对路径
   input_image_topic: "camera/image_raw"    # ❌ 相对路径（可能有问题）
   ```

## 验证配置

启动节点后验证话题配置：

```bash
# 检查节点是否运行
ros2 node list

# 检查话题
ros2 topic list

# 检查节点参数
ros2 param list /resize_node

# 查看具体参数值
ros2 param get /resize_node input_image_topic
ros2 param get /resize_node output_image_topic

# 检查话题信息
ros2 topic info /camera/image_raw
ros2 topic info /camera/resized/image_raw

# 监听话题数据
ros2 topic echo /camera/resized/image_raw --no-arr
```

## 故障排查

### 问题 1: 节点启动但没有输出

**检查输入话题:**
```bash
ros2 topic list | grep image
ros2 topic hz /camera/image_raw  # 检查是否有数据
```

**检查节点日志:**
节点会打印话题配置信息，确认配置是否正确。

### 问题 2: 话题名称不匹配

**查看节点实际订阅的话题:**
```bash
ros2 node info /resize_node
```

**检查参数:**
```bash
ros2 param get /resize_node input_image_topic
```

### 问题 3: 多个节点冲突

确保每个节点有唯一的名称和输出话题：

```bash
# 节点 1
ros2 run resize_image_raw resize_node_exe \
    --ros-args \
    -r __node:=resize_left \
    --params-file camera_left.yaml

# 节点 2
ros2 run resize_image_raw resize_node_exe \
    --ros-args \
    -r __node:=resize_right \
    --params-file camera_right.yaml
```
