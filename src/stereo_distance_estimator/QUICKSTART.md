# 快速开始指南

本文档将帮助你快速开始使用 `stereo_distance_estimator` 包。

## 前置条件

确保你已经：
1. 安装了 ROS2（Humble 或更高版本）
2. 编译了 `rm_interfaces` 包（包含 Target2D、Target3D 等消息定义）
3. 编译了 `stereo_distance_estimator` 包

## 快速测试

### 步骤 1: 编译工作空间

```bash
cd ~/Dart_2026_ws
colcon build --packages-select stereo_distance_estimator
source install/setup.bash
```

### 步骤 2: 启动节点（使用点云模式）

在第一个终端启动 stereo_distance_estimator：

```bash
source ~/Dart_2026_ws/install/setup.bash
ros2 launch stereo_distance_estimator stereo_distance_estimator.launch.py
```

### 步骤 3: 发布测试数据

在第二个终端运行测试发布器：

```bash
source ~/Dart_2026_ws/install/setup.bash
ros2 run stereo_distance_estimator test_publisher.py
```

### 步骤 4: 查看输出

在第三个终端查看 3D 目标输出：

```bash
source ~/Dart_2026_ws/install/setup.bash
ros2 topic echo /stereo/target3d_array_raw
```

你应该能看到类似这样的输出：

```yaml
header:
  stamp:
    sec: 1696435200
    nanosec: 0
  frame_id: camera_link
targets:
- header:
    stamp:
      sec: 1696435200
      nanosec: 0
    frame_id: camera_link
  position:
    x: 0.0
    y: 0.0
    z: 2.0
  distance: 2.0
  confidence: 0.95
  class_name: 'test_target'
  id: 1
  is_filtered: false
```

## 使用真实数据

### 方法 1: 使用配置文件

1. 复制配置文件模板：

```bash
cd ~/Dart_2026_ws/src/stereo_distance_estimator/config
cp stereo_distance_estimator.yaml my_config.yaml
```

2. 编辑 `my_config.yaml`，根据你的相机参数调整：

```yaml
stereo_distance_estimator:
  ros__parameters:
    # 根据你的系统修改话题名称
    target2d_topic: "/your/target2d_topic"
    disparity_topic: "/your/disparity_topic"
    pointcloud_topic: "/your/pointcloud_topic"
    
    # 如果使用视差图模式，需要更新相机内参
    use_pointcloud: false
    fx: 700.0
    fy: 700.0
    cx: 640.0
    cy: 360.0
    baseline: 0.15
```

3. 使用自定义配置启动：

```bash
ros2 launch stereo_distance_estimator stereo_distance_estimator_config.launch.py \
  config_file:=/path/to/my_config.yaml
```

### 方法 2: 使用命令行参数

```bash
ros2 launch stereo_distance_estimator stereo_distance_estimator.launch.py \
  target2d_topic:=/your/target2d_topic \
  disparity_topic:=/your/disparity_topic \
  pointcloud_topic:=/your/pointcloud_topic \
  use_pointcloud:=true \
  max_distance:=15.0
```

## 调试技巧

### 1. 检查话题连接

```bash
# 查看节点订阅的话题
ros2 node info /stereo_distance_estimator

# 查看话题发布频率
ros2 topic hz /stereo/target3d_array_raw
```

### 2. 查看日志

```bash
# 实时查看日志
ros2 run rqt_console rqt_console

# 或在启动时设置日志级别
ros2 launch stereo_distance_estimator stereo_distance_estimator.launch.py \
  --ros-args --log-level debug
```

### 3. 可视化

使用 RViz2 可视化点云和目标：

```bash
rviz2
```

在 RViz2 中：
1. 添加 PointCloud2 显示，订阅 `/stereo/points2`
2. 添加 MarkerArray 显示（需要额外的可视化节点将 Target3DArray 转换为 Marker）

## 常见问题

### Q: 没有输出 3D 目标

**A:** 检查以下几点：
- 输入话题是否有数据：`ros2 topic list`
- 消息时间戳是否同步
- 增加日志级别查看详细信息：`--log-level debug`

### Q: 3D 坐标不准确

**A:** 如果使用视差图模式：
- 验证相机内参（fx, fy, cx, cy）是否正确
- 检查基线距离（baseline）是否准确
- 建议使用点云模式以获得更好的精度

### Q: 性能问题

**A:** 优化建议：
- 使用点云模式而非视差图模式
- 减少输入目标数量（在上游过滤）
- 降低话题发布频率

## 集成到系统

在完整系统中使用时，通常会与以下节点配合：

```
相机驱动 -> stereo_image_proc -> 目标检测 -> coordinate_filter 
                                                     ↓
串口驱动 <- distance_filter <- stereo_distance_estimator
```

创建一个完整的 launch 文件示例：

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ... 其他节点 ...
        
        Node(
            package='stereo_distance_estimator',
            executable='stereo_distance_estimator_node',
            name='stereo_distance_estimator',
            parameters=[{
                'use_pointcloud': True,
                'max_distance': 10.0,
                'min_distance': 0.1,
            }]
        ),
        
        # ... 下游节点 ...
    ])
```

## 下一步

- 调整参数以适应你的应用场景
- 集成到完整的视觉处理管道
- 添加可视化工具以便调试
- 根据需要扩展功能（例如添加卡尔曼滤波）

如有问题，请查看完整的 README.md 文档。
