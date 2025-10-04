# Stereo Processor

双目立体视觉处理节点，使用 OpenCV 的立体匹配算法进行图像校正、视差计算和点云生成。

## 功能特性

- **立体图像校正**: 使用相机标定参数对左右图像进行校正
- **视差图计算**: 支持 SGBM (Semi-Global Block Matching) 算法
- **点云生成**: 从视差图生成带颜色信息的 3D 点云
- **可配置参数**: 灵活的立体匹配参数配置

## Topics

### 订阅 (Subscribed Topics)

| Topic | 类型 | 描述 |
|-------|------|------|
| `/camera/left/image_raw` | `sensor_msgs/Image` | 左相机原始图像 |
| `/camera/right/image_raw` | `sensor_msgs/Image` | 右相机原始图像 |
| `/camera/left/camera_info` | `sensor_msgs/CameraInfo` | 左相机标定信息 |
| `/camera/right/camera_info` | `sensor_msgs/CameraInfo` | 右相机标定信息 |

### 发布 (Published Topics)

| Topic | 类型 | 描述 |
|-------|------|------|
| `/camera/left/image_rect` | `sensor_msgs/Image` | 校正后的左图像 |
| `/camera/right/image_rect` | `sensor_msgs/Image` | 校正后的右图像 |
| `/stereo/disparity` | `stereo_msgs/DisparityImage` | 视差图 |
| `/stereo/points2` | `sensor_msgs/PointCloud2` | 3D 点云 |

## 参数配置

所有参数都可以在 `config/stereo_processor.yaml` 中配置：

### 立体匹配参数

- `stereo_algorithm` (string, default: "sgbm"): 立体匹配算法
- `min_disparity` (int, default: 0): 最小视差值
- `num_disparities` (int, default: 128): 视差搜索范围（必须是16的倍数）
- `block_size` (int, default: 15): 匹配块大小（必须是奇数）

### SGBM 特定参数

- `uniqueness_ratio` (int, default: 10): 唯一性比率 (5-15)
- `speckle_window_size` (int, default: 100): 斑点过滤窗口大小 (50-200)
- `speckle_range` (int, default: 4): 斑点过滤范围 (1-2)
- `disp12_max_diff` (int, default: 1): 左右视差检查的最大差异
- `prefilter_cap` (int, default: 31): 预滤波截断值 (1-63)
- `prefilter_size` (int, default: 5): 预滤波大小（必须是奇数）
- `texture_threshold` (int, default: 10): 纹理阈值

### 点云参数

- `use_color` (bool, default: true): 是否在点云中包含颜色信息

## 使用方法

### 1. 编译

```bash
cd ~/Dart_2026_ws
colcon build --packages-select stereo_processor
source install/setup.bash
```

### 2. 运行

```bash
ros2 launch stereo_processor stereo_processor.launch.py
```

### 3. 使用自定义配置

```bash
ros2 launch stereo_processor stereo_processor.launch.py config_file:=/path/to/your/config.yaml
```

### 4. 可视化

使用 RViz2 可视化点云和图像：

```bash
rviz2
```

在 RViz2 中：
- 添加 `PointCloud2` 显示，订阅 `/stereo/points2`
- 添加 `Image` 显示，订阅 `/camera/left/image_rect` 或 `/stereo/disparity`

## 系统集成

在您的双目测距系统中，此节点替代了原来的 `stereo_image_proc` 节点，提供相同的功能：

```
双目相机驱动 → stereo_processor → 目标检测 → 后续处理
```

## 依赖项

- ROS 2 Humble
- OpenCV (>= 4.0)
- image_geometry
- cv_bridge
- message_filters
- sensor_msgs
- stereo_msgs

## 性能调优建议

1. **提高速度**:
   - 减小 `num_disparities`
   - 减小 `block_size`
   - 降低输入图像分辨率

2. **提高质量**:
   - 增大 `num_disparities`（用于近距离物体）
   - 调整 `uniqueness_ratio`
   - 调整 `speckle_window_size` 和 `speckle_range`

3. **相机标定**:
   - 确保使用高质量的相机标定参数
   - 使用 `camera_calibration` 包重新标定相机

## 故障排除

### 没有点云输出

1. 检查相机标定参数是否正确发布
2. 验证左右图像是否正确同步
3. 检查视差图是否有效

### 视差图质量差

1. 调整 SGBM 参数
2. 改善场景光照条件
3. 确保相机之间有足够的基线距离
4. 增加场景纹理

### 性能问题

1. 降低图像分辨率
2. 减小视差搜索范围
3. 使用更快的硬件

## 相关链接

- [OpenCV Stereo Vision](https://docs.opencv.org/4.x/dd/d53/tutorial_py_depthmap.html)
- [ROS 2 Image Pipeline](https://github.com/ros-perception/image_pipeline)
- [Camera Calibration](http://wiki.ros.org/camera_calibration)

## 作者

amatrix02 (3432900546@qq.com)

## 许可证

Apache-2.0
