# Stereo Image Proc Wrapper

基于 ROS2 官方 `stereo_image_proc` 库的立体视觉处理包装器。该包提供了完整的立体视觉处理管道，包括图像校正、视差计算和点云生成。

## 概述

`stereo_image_proc_wrapper` 是对 ROS2 官方 `stereo_image_proc` 包的高层封装，简化了立体视觉系统的配置和使用。与自定义实现的 `stereo_processor` 相比，该包直接使用 ROS2 生态系统中经过充分测试和优化的官方实现。

## 功能特性

- ✅ **官方实现**: 使用 ROS2 官方维护的 `stereo_image_proc` 库
- ✅ **完整管道**: 自动处理图像校正、视差计算、点云生成
- ✅ **高性能**: 使用组合节点容器 (Composable Nodes) 提高性能
- ✅ **易于配置**: 提供预设的配置文件（标准、高质量、快速模式）
- ✅ **兼容性**: 与 ROS2 生态系统完全兼容

## 系统要求

- ROS2 Humble 或更高版本
- `stereo_image_proc` 包
- `image_proc` 包

### 安装依赖

```bash
sudo apt install ros-humble-stereo-image-proc ros-humble-image-proc
```

## Topics

### 订阅 (Subscribed Topics)

| Topic | 类型 | 描述 |
|-------|------|------|
| `/camera/left/image_rect` | `sensor_msgs/msg/Image` | 校正后的左相机图像 |
| `/camera/right/image_rect` | `sensor_msgs/msg/Image` | 校正后的右相机图像 |
| `/camera/left/camera_info` | `sensor_msgs/msg/CameraInfo` | 左相机标定信息 |
| `/camera/right/camera_info` | `sensor_msgs/msg/CameraInfo` | 右相机标定信息 |

**注意**: 该包需要**已经校正过的图像**作为输入。如果你的相机输出的是原始图像，需要先使用 `image_proc` 进行图像校正。

配置说明：话题名现在可以在配置文件中指定（`config/*.yaml`）。添加了以下可配置参数到 `ros__parameters`：

- `left_image_topic` (string) - 左相机校正后图像的 topic，默认 `/camera/left/image_rect`
- `right_image_topic` (string) - 右相机校正后图像的 topic，默认 `/camera/right/image_rect`
- `left_camera_info_topic` (string) - 左相机 camera_info topic，默认 `/camera/left/camera_info`
- `right_camera_info_topic` (string) - 右相机 camera_info topic，默认 `/camera/right/camera_info`
- `disparity_topic` (string) - 输出视差图 topic，默认 `/stereo/disparity`
- `points_topic` (string) - 输出点云 topic，默认 `/stereo/points2`

这些参数在 `config/stereo_params.yaml`、`config/fast_mode.yaml`、`config/high_quality.yaml` 中已有默认值。你也可以在启动时通过 launch 参数覆盖，例如：

```bash
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py \
  left_image_topic:=/my_cam/left/image_rect \
  right_image_topic:=/my_cam/right/image_rect \
  disparity_topic:=/my_stereo/disparity
```

### 发布 (Published Topics)

| Topic | 类型 | 描述 |
|-------|------|------|
| `/stereo/disparity` | `stereo_msgs/msg/DisparityImage` | 视差图 |
| `/stereo/points2` | `sensor_msgs/msg/PointCloud2` | 彩色 3D 点云 |

## 使用方法

### 基本使用

启动立体视觉处理管道：

```bash
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py
```

### 使用自定义配置文件

```bash
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py \
    config_file:=/path/to/your/config.yaml
```

### 预设配置模式

包含三种预设配置文件：

#### 1. 标准模式（默认）

平衡质量和性能的配置：

```bash
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py
```

#### 2. 高质量模式

优先考虑准确性和细节：

```bash
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py \
    config_file:=$(ros2 pkg prefix stereo_image_proc_wrapper)/share/stereo_image_proc_wrapper/config/high_quality.yaml
```

特点：
- 更大的视差搜索范围 (256)
- 更小的相关窗口大小 (9) 以获得更多细节
- 启用完整 DP 优化
- 更严格的唯一性检查

#### 3. 快速模式

优先考虑处理速度：

```bash
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py \
    config_file:=$(ros2 pkg prefix stereo_image_proc_wrapper)/share/stereo_image_proc_wrapper/config/fast_mode.yaml
```

特点：
- 较小的视差搜索范围 (64)
- 较大的相关窗口大小 (21)
- 禁用完整 DP 优化
- 更宽松的匹配参数

### 完整管道（包含相机驱动）

```bash
ros2 launch stereo_image_proc_wrapper stereo_pipeline.launch.py
```

## 参数配置

### 立体匹配参数

详细的参数说明请参考 `config/stereo_params.yaml` 文件。主要参数包括：

#### 算法选择

- `stereo_algorithm`: 立体匹配算法
  - `0`: Block Matching (BM)
  - `1`: Semi-Global Block Matching (SGBM) - **推荐**

#### 基本参数

- `min_disparity` (int, 默认: 0)
  - 最小视差值，通常为 0

- `disparity_range` (int, 默认: 128)
  - 视差搜索范围，必须是 16 的倍数
  - 更大的值提供更大的深度范围，但计算量更大
  - 典型值: 64, 128, 256

- `correlation_window_size` (int, 默认: 15)
  - 匹配块大小，必须是奇数
  - 较大的值产生更平滑但细节较少的视差图
  - 典型值: 5, 7, 9, 11, 15, 21

#### 质量控制参数

- `uniqueness_ratio` (int, 默认: 10)
  - 唯一性比率，范围 5-15
  - 较高的值产生更可靠但更稀疏的视差图

- `texture_threshold` (int, 默认: 10)
  - 纹理阈值，过滤纹理不足的区域
  - 典型值: 10-100

#### 后处理参数

- `speckle_size` (int, 默认: 100)
  - 斑点过滤大小，设置为 0 禁用
  - 典型值: 50-200

- `speckle_range` (int, 默认: 4)
  - 斑点过滤范围
  - 典型值: 1-2

#### 预滤波参数

- `prefilter_cap` (int, 默认: 31)
  - 预滤波截断值
  - 典型值: 1-63

- `prefilter_size` (int, 默认: 9)
  - 预滤波窗口大小，必须是奇数
  - 典型值: 5, 9, 13

#### SGBM 高级参数

- `full_dp` (bool, 默认: false)
  - 是否使用完整动态规划优化
  - `true`: 更高质量但更慢
  - `false`: 更快但质量略低

### 点云生成参数

- `use_color` (bool, 默认: true)
  - 是否在点云中包含颜色信息

- `approximate_sync` (bool, 默认: true)
  - 是否使用近似时间同步

- `queue_size` (int, 默认: 5)
  - 消息同步队列大小

## 与 stereo_processor 的区别

| 特性 | stereo_processor | stereo_image_proc_wrapper |
|------|------------------|---------------------------|
| 实现方式 | 自定义 OpenCV 封装 | ROS2 官方库 |
| 图像校正 | 内置 | 需要预先校正 |
| 维护 | 自行维护 | 官方维护 |
| 性能优化 | 基础 | 高度优化（组合节点） |
| 功能扩展 | 需要自己实现 | 可使用整个 image_pipeline |
| 学习曲线 | 需要了解 OpenCV | 标准 ROS2 接口 |

## 架构说明

### 处理流程

```
原始图像 → 图像校正 (image_proc) → 视差计算 (DisparityNode) → 点云生成 (PointCloudNode)
```

### 组件说明

1. **DisparityNode**: 从校正后的立体图像对计算视差图
2. **PointCloudNode**: 从视差图生成 3D 点云

这两个节点在一个组合节点容器中运行，以获得最佳性能。

## 性能优化

### 使用组合节点容器

该包默认使用 `ComposableNodeContainer`，这样可以：
- 减少节点间通信开销
- 启用零拷贝传输（intra-process communication）
- 降低 CPU 使用率
- 提高整体吞吐量

### 参数调优建议

对于**实时应用**：
1. 减小 `disparity_range` (例如 64 或 128)
2. 增大 `correlation_window_size` (例如 15 或 21)
3. 设置 `full_dp: false`
4. 减小 `speckle_size`

对于**高精度应用**：
1. 增大 `disparity_range` (例如 256)
2. 减小 `correlation_window_size` (例如 7 或 9)
3. 设置 `full_dp: true`
4. 增大 `uniqueness_ratio`

## 调试与可视化

### 查看视差图

```bash
ros2 run rqt_image_view rqt_image_view /stereo/disparity
```

### 查看点云

```bash
rviz2
```

在 RViz2 中：
1. 添加 PointCloud2 显示
2. 设置 Topic 为 `/stereo/points2`
3. 设置 Fixed Frame 为合适的坐标系

### 监控性能

```bash
ros2 topic hz /stereo/disparity
ros2 topic hz /stereo/points2
```

## 故障排除

### 问题：没有输出点云

**可能原因**：
1. 图像未正确校正
2. 相机标定信息不正确
3. 输入图像不同步

**解决方法**：
1. 确认输入的是已校正图像 (`image_rect`)
2. 检查 `camera_info` 是否包含有效的校正参数
3. 调整 `approximate_sync` 和 `queue_size` 参数

### 问题：视差图质量差

**可能原因**：
1. 场景缺乏纹理
2. 参数设置不当
3. 光照条件差

**解决方法**：
1. 增加场景光照和纹理
2. 调整匹配参数（参考上面的参数调优建议）
3. 尝试不同的预设配置文件

### 问题：处理速度慢

**解决方法**：
1. 使用 `fast_mode.yaml` 配置
2. 降低图像分辨率
3. 减小 `disparity_range`
4. 增大 `correlation_window_size`

## 开发与扩展

该包作为轻量级包装器，主要通过配置文件和 launch 文件来定制行为。如果需要更复杂的功能：

1. 可以修改 launch 文件添加更多节点
2. 可以创建自定义配置文件
3. 可以参考 `stereo_image_proc` 官方文档进行深度定制

## 参考资源

- [stereo_image_proc 官方文档](http://wiki.ros.org/stereo_image_proc)
- [ROS2 image_pipeline](https://github.com/ros-perception/image_pipeline)
- [OpenCV 立体视觉](https://docs.opencv.org/master/dd/d53/tutorial_py_depthmap.html)

## 许可证

Apache-2.0

## 维护者

- amatrix02 (3432900546@qq.com)

## 更新日志

### 0.0.0 (2025-10-04)

- 初始版本
- 支持基本的立体视觉处理管道
- 提供三种预设配置文件
- 使用组合节点容器优化性能
