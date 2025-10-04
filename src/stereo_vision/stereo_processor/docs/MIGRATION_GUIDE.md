# Stereo Processor 使用指南

## 从 target_matcher 迁移到 stereo_processor

本文档说明如何将原有的 `target_matcher` 包更新为新的 `stereo_processor` 包，并集成到您的双目测距系统中。

## 更改摘要

### 包重命名
- **旧名称**: `target_matcher`
- **新名称**: `stereo_processor`

### 功能变化
- 新增双目图像校正功能
- 新增视差图计算（SGBM算法）
- 新增点云生成功能
- 可以替代原系统中的 `stereo_image_proc` 节点

## 系统架构对比

### 原架构
```
stereo_camera_driver → stereo_image_proc → object_detection → ...
```

### 新架构
```
stereo_camera_driver → stereo_processor → object_detection → ...
                          (替代 stereo_image_proc)
```

## 快速开始

### 1. 编译包

```bash
cd ~/Dart_2026_ws
colcon build --packages-select stereo_processor
source install/setup.bash
```

### 2. 启动节点

#### 方式 1: 单独启动 stereo_processor
```bash
ros2 launch stereo_processor stereo_processor.launch.py
```

#### 方式 2: 启动完整系统
```bash
ros2 launch stereo_processor full_stereo_system.launch.py
```

### 3. 验证节点运行

检查节点是否在运行：
```bash
ros2 node list | grep stereo
```

检查发布的 topics：
```bash
ros2 topic list | grep -E "(rect|disparity|points2)"
```

查看节点信息：
```bash
ros2 node info /stereo_processor
```

## Topic 映射

### 输入 Topics（需要相机驱动提供）

| Topic | 类型 | 说明 |
|-------|------|------|
| `/camera/left/image_raw` | `sensor_msgs/Image` | 左相机原始图像 |
| `/camera/right/image_raw` | `sensor_msgs/Image` | 右相机原始图像 |
| `/camera/left/camera_info` | `sensor_msgs/CameraInfo` | 左相机标定信息 |
| `/camera/right/camera_info` | `sensor_msgs/CameraInfo` | 右相机标定信息 |

### 输出 Topics

| Topic | 类型 | 下游节点 |
|-------|------|----------|
| `/camera/left/image_rect` | `sensor_msgs/Image` | object_detection_openvino |
| `/camera/right/image_rect` | `sensor_msgs/Image` | - |
| `/stereo/disparity` | `stereo_msgs/DisparityImage` | stereo_distance_estimator |
| `/stereo/points2` | `sensor_msgs/PointCloud2` | stereo_distance_estimator |

## 与现有系统集成

### 更新 object_detection_openvino

确保目标检测节点订阅校正后的图像：

```python
# 在 object_detection_openvino 的 launch 文件中
object_detection_node = Node(
    package='object_detection_openvino',
    executable='object_detection_node',
    name='object_detection',
    remappings=[
        ('/image_raw', '/camera/left/image_rect'),  # 使用校正后的图像
    ]
)
```

### 更新 stereo_distance_estimator

确保立体测距节点订阅正确的 topics：

```python
stereo_distance_node = Node(
    package='stereo_distance_estimator',
    executable='stereo_distance_estimator_node',
    name='stereo_distance_estimator',
    remappings=[
        ('/disparity', '/stereo/disparity'),
        ('/points2', '/stereo/points2'),
    ]
)
```

## 参数调优

### 常见场景参数建议

#### 近距离目标（< 2m）
```yaml
stereo_processor:
  ros__parameters:
    num_disparities: 256  # 增大视差范围
    block_size: 11
    uniqueness_ratio: 5
```

#### 远距离目标（> 5m）
```yaml
stereo_processor:
  ros__parameters:
    num_disparities: 64   # 减小视差范围
    block_size: 21
    uniqueness_ratio: 15
```

#### 高速场景（提高性能）
```yaml
stereo_processor:
  ros__parameters:
    num_disparities: 64   # 减小计算量
    block_size: 9
    use_color: false      # 不生成彩色点云
```

#### 高精度场景（提高质量）
```yaml
stereo_processor:
  ros__parameters:
    num_disparities: 128
    block_size: 15
    speckle_window_size: 200
    speckle_range: 2
```

## 可视化和调试

### 使用 RViz2 可视化

```bash
rviz2
```

在 RViz2 中添加：
1. **PointCloud2** - 订阅 `/stereo/points2`
2. **Image** - 订阅 `/camera/left/image_rect`
3. **Image** - 订阅 `/stereo/disparity/image`

### 使用 rqt_image_view 查看图像

```bash
# 查看校正后的左图
ros2 run rqt_image_view rqt_image_view /camera/left/image_rect

# 查看视差图
ros2 run rqt_image_view rqt_image_view /stereo/disparity/image
```

### 查看点云统计

```bash
ros2 topic echo /stereo/points2 --once
```

## 性能优化

### 1. 降低图像分辨率

在相机驱动配置中降低图像分辨率：
```yaml
camera:
  image_width: 640   # 从 1920 降到 640
  image_height: 480  # 从 1080 降到 480
```

### 2. 调整 QoS 策略

对于高频率数据流，使用 BEST_EFFORT QoS：
```cpp
// 在节点代码中
auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
    .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
```

### 3. 多线程处理

考虑使用 MultiThreadedExecutor：
```cpp
rclcpp::executors::MultiThreadedExecutor executor;
executor.add_node(node);
executor.spin();
```

## 故障排除

### 问题 1: 没有视差图输出

**可能原因**:
- 相机标定参数未正确发布
- 左右图像未同步

**解决方案**:
```bash
# 检查 camera_info 是否发布
ros2 topic echo /camera/left/camera_info --once
ros2 topic echo /camera/right/camera_info --once

# 检查图像时间戳
ros2 topic echo /camera/left/image_raw --once | grep stamp
ros2 topic echo /camera/right/image_raw --once | grep stamp
```

### 问题 2: 视差图质量差

**可能原因**:
- 参数设置不当
- 场景纹理不足
- 光照条件差

**解决方案**:
- 参考"参数调优"章节调整参数
- 改善场景光照
- 增加场景纹理（投影随机点阵）

### 问题 3: 点云有很多噪点

**可能原因**:
- 视差图质量差
- speckle_filter 参数太宽松

**解决方案**:
```yaml
stereo_processor:
  ros__parameters:
    speckle_window_size: 50   # 减小
    speckle_range: 2          # 减小
    uniqueness_ratio: 15      # 增大
```

### 问题 4: 处理速度慢

**可能原因**:
- 图像分辨率太高
- num_disparities 太大

**解决方案**:
- 降低输入图像分辨率
- 减小 num_disparities
- 减小 block_size

## 相机标定

如果视差图质量不佳，可能需要重新标定相机：

### 使用 camera_calibration 包

```bash
# 安装标定工具
sudo apt install ros-humble-camera-calibration

# 运行标定（使用棋盘格）
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.108 \
  --ros-args \
  -r right:=/camera/right/image_raw \
  -r left:=/camera/left/image_raw \
  -r left_camera:=/camera/left \
  -r right_camera:=/camera/right
```

标定完成后，保存标定文件并更新相机驱动配置。

## 进阶配置

### 使用自定义相机 frame

```python
stereo_processor_node = Node(
    package='stereo_processor',
    executable='stereo_processor_node',
    name='stereo_processor',
    parameters=[{
        'frame_id': 'custom_stereo_frame',
    }]
)
```

### 动态参数调整

```bash
# 实时调整参数
ros2 param set /stereo_processor num_disparities 192
ros2 param set /stereo_processor block_size 21

# 查看当前参数
ros2 param list /stereo_processor
ros2 param get /stereo_processor num_disparities
```

## 相关资源

- [OpenCV Stereo Vision Tutorial](https://docs.opencv.org/4.x/dd/d53/tutorial_py_depthmap.html)
- [ROS 2 Image Pipeline](https://github.com/ros-perception/image_pipeline)
- [Camera Calibration Tutorial](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration)

## 技术支持

如有问题，请联系：
- 维护者: amatrix02
- Email: 3432900546@qq.com
- 仓库: Dart_2026_ws

## 更新日志

### v0.0.0 (2025-10-04)
- 初始版本
- 从 target_matcher 重命名为 stereo_processor
- 实现基于 SGBM 的立体匹配
- 实现点云生成功能
