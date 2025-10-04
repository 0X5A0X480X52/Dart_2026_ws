# Stereo Rectifier

立体相机图像校正包，基于 ROS2 官方的 `image_proc` 库进行图像校正。该包集成了海康威视（HIK）和迈德威视（MindVision）相机驱动，提供完整的立体相机图像采集和校正流程。

## 概述

`stereo_rectifier` 包使用 ROS2 的 `image_proc` 包对双目相机的原始图像进行校正（rectification），消除镜头畸变并进行立体校正，为后续的立体视觉处理（如视差计算、点云生成）提供标准化的输入。

## 功能特性

- ✅ **集成相机驱动**: 支持海康威视和迈德威视工业相机
- ✅ **图像校正**: 使用 ROS2 官方 `image_proc` 进行畸变校正和立体校正
- ✅ **高性能**: 使用组合节点容器（Composable Nodes）实现零拷贝传输
- ✅ **灵活配置**: 支持独立使用图像校正或与相机驱动集成使用
- ✅ **标准接口**: 输出符合 ROS2 标准的校正图像和相机信息

## 系统要求

- ROS2 Humble 或更高版本
- `image_proc` 包
- `ros2_hik_camera` 包（使用海康威视相机时）
- `ros2_mindvision_camera` 包（使用迈德威视相机时）

### 安装依赖

```bash
# 安装 image_proc
sudo apt install ros-humble-image-proc

# 相机驱动包应该已经在工作空间中编译
cd ~/Dart_2026_ws
colcon build --packages-select ros2_hik_camera ros2_mindvision_camera
```

## 工作原理

### 图像校正流程

```
原始图像 (image_raw) + 相机标定信息 (camera_info)
                    ↓
            image_proc::RectifyNode
                    ↓
        校正后的图像 (image_rect)
```

### 完整立体相机流程

```
相机驱动 → 原始图像 → 图像校正 → 校正图像 → 立体视觉处理
```

## Topics

### 订阅 (Subscribed Topics)

| Topic | 类型 | 描述 |
|-------|------|------|
| `/camera/left/image_raw` | `sensor_msgs/msg/Image` | 左相机原始图像 |
| `/camera/right/image_raw` | `sensor_msgs/msg/Image` | 右相机原始图像 |
| `/camera/left/camera_info` | `sensor_msgs/msg/CameraInfo` | 左相机标定信息 |
| `/camera/right/camera_info` | `sensor_msgs/msg/CameraInfo` | 右相机标定信息 |

### 发布 (Published Topics)

| Topic | 类型 | 描述 |
|-------|------|------|
| `/camera/left/image_rect` | `sensor_msgs/msg/Image` | 校正后的左相机图像（单通道） |
| `/camera/right/image_rect` | `sensor_msgs/msg/Image` | 校正后的右相机图像（单通道） |
| `/camera/left/image_rect_color` | `sensor_msgs/msg/Image` | 校正后的左相机彩色图像 |
| `/camera/right/image_rect_color` | `sensor_msgs/msg/Image` | 校正后的右相机彩色图像 |

## 使用方法

### 1. 仅图像校正（假设已有图像源）

如果你已经有相机驱动在运行，只需要图像校正：

```bash
ros2 launch stereo_rectifier stereo_rectify.launch.py
```

### 2. 海康威视相机 + 图像校正

完整的海康威视立体相机流程：

```bash
ros2 launch stereo_rectifier hik_stereo_rectify.launch.py
```

#### 指定相机序列号

```bash
ros2 launch stereo_rectifier hik_stereo_rectify.launch.py \
    left_camera_sn:=00J12345678 \
    right_camera_sn:=00J12345679
```

#### 指定标定文件

```bash
ros2 launch stereo_rectifier hik_stereo_rectify.launch.py \
    left_camera_info_url:=file:///path/to/left_calibration.yaml \
    right_camera_info_url:=file:///path/to/right_calibration.yaml
```

### 3. 迈德威视相机 + 图像校正

完整的迈德威视立体相机流程：

```bash
ros2 launch stereo_rectifier mindvision_stereo_rectify.launch.py
```

#### 指定标定文件

```bash
ros2 launch stereo_rectifier mindvision_stereo_rectify.launch.py \
    left_camera_info_url:=file:///path/to/left_calibration.yaml \
    right_camera_info_url:=file:///path/to/right_calibration.yaml
```

## 相机标定

在使用图像校正之前，必须先对相机进行标定以获取标定参数。

### 使用 ROS2 标定工具

#### 单目相机标定

```bash
# 左相机
ros2 run camera_calibration cameracalibrator \
    --size 8x6 \
    --square 0.025 \
    --ros-args -r image:=/camera/left/image_raw \
                -r camera:=/camera/left

# 右相机
ros2 run camera_calibration cameracalibrator \
    --size 8x6 \
    --square 0.025 \
    --ros-args -r image:=/camera/right/image_raw \
                -r camera:=/camera/right
```

参数说明：
- `--size 8x6`: 棋盘格内角点数量（宽x高）
- `--square 0.025`: 棋盘格方格边长（米）
- 调整这些参数以匹配你的标定板

#### 立体相机标定

```bash
ros2 run camera_calibration cameracalibrator \
    --size 8x6 \
    --square 0.025 \
    --approximate 0.1 \
    --ros-args -r left:=/camera/left/image_raw \
                -r right:=/camera/right/image_raw \
                -r left_camera:=/camera/left \
                -r right_camera:=/camera/right
```

标定步骤：
1. 准备棋盘格标定板
2. 启动相机驱动
3. 运行标定工具
4. 在不同位置、角度、距离移动标定板
5. 确保标定进度条中的 X、Y、Size、Skew 都变绿
6. 点击 "Calibrate" 按钮进行标定
7. 标定完成后点击 "Save" 保存标定文件
8. 将标定文件复制到 `config/` 目录

### 标定文件格式

标定文件应该是 YAML 格式，示例见 `config/left_camera_info.yaml` 和 `config/right_camera_info.yaml`。

关键参数：
- **camera_matrix**: 相机内参矩阵（焦距、主点）
- **distortion_coefficients**: 畸变系数
- **rectification_matrix**: 立体校正旋转矩阵
- **projection_matrix**: 投影矩阵

对于立体相机，右相机的 `projection_matrix` 最后一列第一行包含基线信息（Tx = -baseline * fx）。

## 配置文件

### config/left_camera_info.yaml

左相机标定参数模板，包含：
- 图像尺寸
- 相机内参
- 畸变模型和系数
- 校正矩阵
- 投影矩阵

### config/right_camera_info.yaml

右相机标定参数模板，格式同左相机。

### config/stereo_calibration.yaml

完整的立体标定参数示例，包含左右相机的所有参数和基线信息。

## 架构说明

### 组合节点容器

该包使用 `ComposableNodeContainer` 将多个节点组合在一起运行：

**优势**：
- 零拷贝数据传输（进程内通信）
- 降低 CPU 使用率
- 减少延迟
- 提高整体吞吐量

### 节点组成

以 `hik_stereo_rectify.launch.py` 为例：

```
ComposableNodeContainer
├── left_camera_node (HIK 相机驱动)
├── right_camera_node (HIK 相机驱动)
├── left_rectify_node (图像校正)
└── right_rectify_node (图像校正)
```

所有节点在同一个进程中运行，数据传输无需序列化和网络通信。

## 可视化与调试

### 查看原始图像

```bash
ros2 run rqt_image_view rqt_image_view /camera/left/image_raw
ros2 run rqt_image_view rqt_image_view /camera/right/image_raw
```

### 查看校正后的图像

```bash
ros2 run rqt_image_view rqt_image_view /camera/left/image_rect
ros2 run rqt_image_view rqt_image_view /camera/right/image_rect
```

### 对比原始和校正图像

使用 RViz2 同时显示多个图像：

```bash
rviz2
```

添加多个 Image 显示：
1. 添加 Image 显示，Topic 设置为 `/camera/left/image_raw`
2. 再添加 Image 显示，Topic 设置为 `/camera/left/image_rect`
3. 对比畸变校正效果

### 监控话题

```bash
# 检查话题列表
ros2 topic list

# 查看图像发布频率
ros2 topic hz /camera/left/image_raw
ros2 topic hz /camera/left/image_rect

# 查看相机信息
ros2 topic echo /camera/left/camera_info
```

### 检查节点状态

```bash
# 列出所有节点
ros2 node list

# 查看容器中的节点
ros2 component list
```

## 与其他包集成

### 与 stereo_image_proc_wrapper 集成

校正后的图像可以直接用于立体视觉处理：

**终端 1**: 启动相机和图像校正
```bash
ros2 launch stereo_rectifier hik_stereo_rectify.launch.py
```

**终端 2**: 启动立体视觉处理
```bash
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py
```

这样就构建了完整的立体视觉流程：
```
相机驱动 → 图像校正 → 视差计算 → 点云生成
```

### 创建统一的 Launch 文件

你可以创建一个统一的 launch 文件来启动所有节点：

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # 相机 + 图像校正
    rectifier = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('stereo_rectifier'),
                'launch',
                'hik_stereo_rectify.launch.py'
            ])
        ])
    )
    
    # 立体视觉处理
    stereo_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('stereo_image_proc_wrapper'),
                'launch',
                'stereo_image_proc.launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        rectifier,
        stereo_proc,
    ])
```

## 故障排除

### 问题：没有校正后的图像输出

**可能原因**：
1. 未提供有效的标定文件
2. 标定文件路径错误
3. 原始图像没有发布

**解决方法**：
1. 检查标定文件是否存在：`ls -la config/`
2. 确认相机驱动正在运行：`ros2 topic list | grep image_raw`
3. 检查 camera_info 是否发布：`ros2 topic echo /camera/left/camera_info`
4. 查看节点日志：`ros2 node list` 和检查终端输出

### 问题：校正后的图像有黑边

**原因**：这是正常现象。图像校正后，由于畸变校正和立体校正，图像边缘会出现无效区域。

**解决方法**：
- 在后续处理中忽略黑边区域
- 使用更大的视场角拍摄
- 在标定时确保标定板覆盖整个视野

### 问题：相机无法打开

**可能原因**：
1. 相机未连接或未供电
2. 相机驱动权限问题
3. 相机已被其他程序占用

**解决方法**：
1. 检查相机连接和电源
2. 检查相机驱动是否有权限访问设备
3. 确保没有其他程序在使用相机
4. 查看相机驱动的日志输出

### 问题：图像延迟高

**解决方法**：
1. 确认使用了组合节点容器（已经在 launch 文件中配置）
2. 降低图像分辨率
3. 优化相机曝光和增益设置
4. 检查系统 CPU 和内存使用情况

## 性能优化

### 使用组合节点

本包已经使用组合节点容器，无需额外配置。

### 图像分辨率

根据应用需求调整图像分辨率：
- 高精度应用：使用高分辨率（如 1920x1080）
- 实时应用：使用中等分辨率（如 1280x720）
- 快速原型：使用低分辨率（如 640x480）

### 相机参数

在相机驱动配置中调整：
- **曝光时间**: 根据环境光照调整
- **增益**: 低光照环境可适当增加
- **帧率**: 根据应用需求设置（10-60 FPS）

## 开发扩展

### 添加其他相机驱动

要添加其他类型的相机支持，创建新的 launch 文件：

1. 复制现有的 launch 文件作为模板
2. 修改相机驱动节点的 package 和 plugin
3. 调整参数以匹配新相机的接口
4. 保持 image_proc 节点配置不变

### 自定义话题名称

在 launch 文件的 `remappings` 中修改话题映射：

```python
remappings=[
    ('image', '/my_custom/left/raw'),
    ('camera_info', '/my_custom/left/info'),
    ('image_rect', '/my_custom/left/rect'),
],
```

## 参考资源

- [image_proc 官方文档](http://wiki.ros.org/image_proc)
- [ROS2 Camera Calibration](https://navigation.ros.org/tutorials/docs/camera_calibration.html)
- [OpenCV Camera Calibration](https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html)
- [ros2_hik_camera](../../../rm_hardware_driver/ros2_hik_camera/)
- [ros2_mindvision_camera](../../../rm_hardware_driver/ros2_mindvision_camera/)

## 许可证

Apache-2.0

## 维护者

- amatrix02 (3432900546@qq.com)

## 更新日志

### 0.0.0 (2025-10-04)

- 初始版本
- 支持海康威视相机集成
- 支持迈德威视相机集成
- 支持独立图像校正模式
- 使用组合节点容器优化性能
- 提供标定文件模板
