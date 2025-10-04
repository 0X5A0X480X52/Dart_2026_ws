# Stereo Vision 完整流程指南

本文档描述如何使用 stereo_vision 包组构建完整的立体视觉系统。

## 系统架构

### 完整流程图

```
┌─────────────────────────────────────────────────────────────┐
│                    相机硬件层                                 │
│  ┌────────────────┐              ┌────────────────┐         │
│  │  Left Camera   │              │  Right Camera  │         │
│  │  (HIK/MindVis) │              │  (HIK/MindVis) │         │
│  └────────┬───────┘              └────────┬───────┘         │
└───────────┼──────────────────────────────┼─────────────────┘
            │                               │
            │ Raw Images                    │ Raw Images
            ▼                               ▼
┌─────────────────────────────────────────────────────────────┐
│              stereo_rectifier (图像校正层)                   │
│  ┌──────────────────────────────────────────────────────┐   │
│  │          image_proc::RectifyNode (左)                │   │
│  │          - 畸变校正                                  │   │
│  │          - 立体校正                                  │   │
│  └─────────────────┬────────────────────────────────────┘   │
│                    │ Rectified Images                       │
│  ┌──────────────────────────────────────────────────────┐   │
│  │          image_proc::RectifyNode (右)                │   │
│  │          - 畸变校正                                  │   │
│  │          - 立体校正                                  │   │
│  └─────────────────┬────────────────────────────────────┘   │
└───────────────────┼──────────────────────────────────────────┘
                    │
                    │ Rectified Stereo Pair
                    ▼
┌─────────────────────────────────────────────────────────────┐
│        stereo_image_proc_wrapper (立体视觉处理层)            │
│  ┌──────────────────────────────────────────────────────┐   │
│  │     stereo_image_proc::DisparityNode                 │   │
│  │     - 立体匹配 (SGBM/BM)                             │   │
│  │     - 视差图生成                                     │   │
│  └─────────────────┬────────────────────────────────────┘   │
│                    │ Disparity Image                        │
│  ┌──────────────────────────────────────────────────────┐   │
│  │     stereo_image_proc::PointCloudNode                │   │
│  │     - 三角测量                                       │   │
│  │     - 点云生成                                       │   │
│  └─────────────────┬────────────────────────────────────┘   │
└───────────────────┼──────────────────────────────────────────┘
                    │
                    │ 3D Point Cloud
                    ▼
         ┌────────────────────────┐
         │    应用层               │
         │  - 目标检测             │
         │  - 距离估计             │
         │  - 导航避障             │
         │  - 3D 重建              │
         └────────────────────────┘
```

## 一键启动完整流程

### 方法 1: 使用集成 Launch 文件

```bash
ros2 launch stereo_rectifier complete_stereo_pipeline.launch.py
```

这会启动：
1. 海康威视左右相机驱动
2. 左右图像校正
3. 视差计算
4. 点云生成

### 方法 2: 分步启动

**终端 1**: 启动相机和图像校正
```bash
ros2 launch stereo_rectifier hik_stereo_rectify.launch.py
```

**终端 2**: 启动立体视觉处理
```bash
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py
```

## 使用不同相机

### 海康威视相机

```bash
ros2 launch stereo_rectifier complete_stereo_pipeline.launch.py
```

或使用序列号指定：

```bash
ros2 launch stereo_rectifier hik_stereo_rectify.launch.py \
    left_camera_sn:=00J12345678 \
    right_camera_sn:=00J12345679
```

### 迈德威视相机

修改 `complete_stereo_pipeline.launch.py` 中的：
```python
'hik_stereo_rectify.launch.py'
```
改为：
```python
'mindvision_stereo_rectify.launch.py'
```

或直接使用：
```bash
ros2 launch stereo_rectifier mindvision_stereo_rectify.launch.py
```

## 配置和优化

### 1. 相机标定

**必须步骤！** 在使用系统前必须完成相机标定。

```bash
ros2 run camera_calibration cameracalibrator \
    --size 8x6 --square 0.025 --approximate 0.1 \
    --ros-args -r left:=/camera/left/image_raw \
                -r right:=/camera/right/image_raw \
                -r left_camera:=/camera/left \
                -r right_camera:=/camera/right
```

标定完成后，将文件保存到：
- `stereo_rectifier/config/left_camera_info.yaml`
- `stereo_rectifier/config/right_camera_info.yaml`

### 2. 立体匹配参数调整

根据应用场景选择配置：

**标准模式**（默认）:
```bash
ros2 launch stereo_rectifier complete_stereo_pipeline.launch.py
```

**高质量模式**（更精确，更慢）:
```bash
ros2 launch stereo_rectifier complete_stereo_pipeline.launch.py \
    stereo_config:=$(ros2 pkg prefix stereo_image_proc_wrapper)/share/stereo_image_proc_wrapper/config/high_quality.yaml
```

**快速模式**（更快，精度略低）:
```bash
ros2 launch stereo_rectifier complete_stereo_pipeline.launch.py \
    stereo_config:=$(ros2 pkg prefix stereo_image_proc_wrapper)/share/stereo_image_proc_wrapper/config/fast_mode.yaml
```

### 3. 运行时参数调整

```bash
# 查看参数
ros2 param list

# 调整视差范围
ros2 param set /stereo_container/disparity_node disparity_range 256

# 调整匹配窗口大小
ros2 param set /stereo_container/disparity_node correlation_window_size 11
```

## 话题监控

### 检查所有话题

```bash
ros2 topic list
```

应该看到：
```
/camera/left/image_raw
/camera/left/image_rect
/camera/left/image_rect_color
/camera/left/camera_info
/camera/right/image_raw
/camera/right/image_rect
/camera/right/image_rect_color
/camera/right/camera_info
/stereo/disparity
/stereo/points2
```

### 监控频率

```bash
# 原始图像
ros2 topic hz /camera/left/image_raw

# 校正图像
ros2 topic hz /camera/left/image_rect

# 视差图
ros2 topic hz /stereo/disparity

# 点云
ros2 topic hz /stereo/points2
```

### 检查数据

```bash
# 查看图像尺寸
ros2 topic echo /camera/left/image_raw --no-arr | grep -E "height|width"

# 查看相机标定信息
ros2 topic echo /camera/left/camera_info

# 查看视差图范围
ros2 topic echo /stereo/disparity --no-arr | grep -E "min_disparity|max_disparity"

# 查看点云点数
ros2 topic echo /stereo/points2 --no-arr | grep -E "width|height"
```

## 可视化

### 使用 rqt_image_view

```bash
# 原始左图像
ros2 run rqt_image_view rqt_image_view /camera/left/image_raw

# 校正左图像
ros2 run rqt_image_view rqt_image_view /camera/left/image_rect

# 视差图
ros2 run rqt_image_view rqt_image_view /stereo/disparity/image
```

### 使用 RViz2

```bash
rviz2
```

配置步骤：
1. 设置 Fixed Frame 为 `camera_left_optical_frame`
2. 添加 Image 显示
   - Topic: `/camera/left/image_rect`
3. 添加 Image 显示
   - Topic: `/stereo/disparity/image`
4. 添加 PointCloud2 显示
   - Topic: `/stereo/points2`
   - Style: Points 或 Flat Squares
   - Size: 0.01
   - Color: RGB8

### 并排对比

使用 RViz2 的多视图功能同时显示：
- 原始图像
- 校正图像
- 视差图
- 点云

## 性能基准

### 测试配置

- CPU: Intel Core i7
- 图像: 1920x1080
- 视差范围: 128
- 匹配窗口: 15

### 预期性能

| 指标 | 标准模式 | 高质量模式 | 快速模式 |
|------|----------|-----------|---------|
| 帧率 | ~15 FPS | ~8 FPS | ~25 FPS |
| CPU 使用 | ~50% | ~70% | ~30% |
| 延迟 | ~70ms | ~130ms | ~40ms |
| 点云密度 | 中等 | 高 | 低 |

*实际性能取决于硬件配置和图像内容*

## 常见问题

### 1. 没有点云输出

**检查清单**:
- [ ] 相机是否正常工作？`ros2 topic hz /camera/left/image_raw`
- [ ] 图像是否已校正？`ros2 topic hz /camera/left/image_rect`
- [ ] 标定文件是否正确？`ros2 topic echo /camera/left/camera_info`
- [ ] 场景是否有足够纹理？

### 2. 点云质量差

**优化建议**:
1. 使用高质量配置模式
2. 改善照明条件
3. 增加场景纹理
4. 重新进行高质量标定
5. 调整匹配参数

### 3. 处理速度慢

**优化建议**:
1. 使用快速模式配置
2. 降低图像分辨率
3. 减小视差搜索范围
4. 检查 CPU 负载
5. 确保使用组合节点容器（已默认启用）

### 4. 相机启动失败

**检查**:
- 相机连接和电源
- USB/网络连接
- 设备权限
- 没有其他程序占用相机

## 集成到您的应用

### 订阅点云

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class StereoVisionConsumer(Node):
    def __init__(self):
        super().__init__('stereo_vision_consumer')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/stereo/points2',
            self.point_cloud_callback,
            10)
    
    def point_cloud_callback(self, msg):
        # 处理点云数据
        self.get_logger().info(f'Received point cloud with {msg.width * msg.height} points')

def main():
    rclpy.init()
    node = StereoVisionConsumer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

### 订阅视差图

```python
from stereo_msgs.msg import DisparityImage

class DisparityConsumer(Node):
    def __init__(self):
        super().__init__('disparity_consumer')
        self.subscription = self.create_subscription(
            DisparityImage,
            '/stereo/disparity',
            self.disparity_callback,
            10)
    
    def disparity_callback(self, msg):
        # 处理视差图
        min_disp = msg.min_disparity
        max_disp = msg.max_disparity
        self.get_logger().info(f'Disparity range: {min_disp} to {max_disp}')
```

## 下一步

1. **学习参数调优**: 阅读 `stereo_image_proc_wrapper/README.md`
2. **高级标定技术**: 学习立体标定最佳实践
3. **性能优化**: 根据应用需求优化系统参数
4. **应用开发**: 使用点云数据进行目标检测、距离估计等

## 相关文档

- [stereo_rectifier README](stereo_rectifier/README.md)
- [stereo_image_proc_wrapper README](stereo_image_proc_wrapper/README.md)
- [stereo_processor README](stereo_processor/README.md)
- [包对比说明](COMPARISON.md)
- [ROS2 Camera Calibration Tutorial](https://navigation.ros.org/tutorials/docs/camera_calibration.html)
