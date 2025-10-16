# 图像缩放功能说明

## 概述

该功能在双目立体匹配处理前对输入图像进行缩放，可以显著减少计算量，提高处理速度。

## 功能特性

- 自动对左右相机图像同时进行缩放
- 自动更新相机内参（camera_info）以匹配缩放后的图像
- 可通过配置文件或启动参数灵活调整缩放率
- 使用 ROS2 的 `image_proc::ResizeNode` 组件实现

## 配置方法

### 方法1：修改配置文件（推荐）

编辑配置文件 `config/stereo_params.yaml`：

```yaml
/**:
  ros__parameters:
    # 图像缩放参数
    # 输入图像在进行双目匹配前的缩放因子
    # 较小的值会减少计算量但也会降低精度
    # 典型值: 0.25, 0.5, 0.75, 1.0 (不缩放)
    # 设置为 1.0 可禁用图像缩放
    image_scale: 0.5  # 将图像缩放到原始尺寸的50%
```

### 方法2：通过启动参数覆盖

启动时使用参数覆盖配置文件中的值：

```bash
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py image_scale:=0.5
```

## 缩放率选择建议

| 缩放率 | 计算量 | 精度 | 适用场景 |
|--------|--------|------|----------|
| 1.0    | 最高   | 最高 | 需要最高精度，计算资源充足 |
| 0.75   | 较高   | 较高 | 平衡精度和性能 |
| 0.5    | 中等   | 中等 | 实时性要求高，可接受精度损失 |
| 0.25   | 最低   | 较低 | 计算资源极其有限 |

## 数据流说明

原始数据流：

```text
相机话题 → 双目匹配 → 视差图 → 点云
```

启用缩放后的数据流：

```text
相机话题 → Resize节点 → 缩放后图像 → 双目匹配 → 视差图 → 点云
```

具体话题映射：

- 左相机输入: `left_image_topic` → Resize → `/stereo/left/image_resized`
- 右相机输入: `right_image_topic` → Resize → `/stereo/right/image_resized`
- 左相机信息: `left_camera_info_topic` → Resize → `/stereo/left/camera_info_resized`
- 右相机信息: `right_camera_info_topic` → Resize → `/stereo/right/camera_info_resized`

## 性能影响

### 计算量

图像缩放后，双目匹配的计算量大约按缩放率的平方减少：

- 0.5倍缩放 ≈ 25%的原始计算量
- 0.25倍缩放 ≈ 6.25%的原始计算量

### 注意事项

1. **精度损失**: 缩放会导致细节丢失，影响视差图和点云的精度
2. **相机标定**: 缩放后的相机内参会自动调整，保持几何关系的正确性
3. **视差范围**: 可能需要相应调整配置文件中的 `disparity_range` 参数

## 完整启动示例

```bash
# 使用配置文件中的默认缩放率 (0.5)
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py

# 使用自定义缩放率
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py image_scale:=0.75

# 禁用缩放（使用原始分辨率）
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py image_scale:=1.0

# 同时指定其他参数
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py \
    image_scale:=0.5 \
    left_image_topic:=/camera/left/image_raw \
    right_image_topic:=/camera/right/image_raw
```

## 调试和查看

查看缩放后的图像：

```bash
# 查看缩放后的左图像
ros2 run rqt_image_view rqt_image_view /stereo/left/image_resized

# 查看缩放后的右图像
ros2 run rqt_image_view rqt_image_view /stereo/right/image_resized

# 查看所有话题
ros2 topic list | grep stereo
```

## 故障排除

### 问题1: 找不到 image_proc 包

确保已安装 image_proc：

```bash
sudo apt-get install ros-<your-ros-distro>-image-proc
```

### 问题2: 缩放率不生效

检查启动时的输出日志，确认 `Image scale default:` 的值是否正确。

### 问题3: 视差图质量下降

尝试增大缩放率（如从0.25改为0.5），或调整双目匹配参数以适应新的图像分辨率。
