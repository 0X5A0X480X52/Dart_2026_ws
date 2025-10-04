# Stereo Image Proc Wrapper - 快速入门

## 安装

### 1. 安装 ROS2 依赖

```bash
sudo apt install ros-humble-stereo-image-proc ros-humble-image-proc
```

### 2. 编译包

```bash
cd /home/amatrix/Dart_2026_ws
colcon build --packages-select stereo_image_proc_wrapper
source install/setup.bash
```

## 快速使用

### 基本使用（默认配置）

假设你已经有了校正后的立体图像：

```bash
# 启动立体视觉处理
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py
```

### 使用高质量模式

```bash
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py \
    config_file:=$(ros2 pkg prefix stereo_image_proc_wrapper)/share/stereo_image_proc_wrapper/config/high_quality.yaml
```

### 使用快速模式

```bash
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py \
    config_file:=$(ros2 pkg prefix stereo_image_proc_wrapper)/share/stereo_image_proc_wrapper/config/fast_mode.yaml
```

## 完整示例：从原始图像到点云

如果你的相机输出的是原始图像（未校正），需要完整的处理流程：

### 创建自定义 Launch 文件

创建 `complete_stereo_pipeline.launch.py`:

```python
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 配置文件路径
    config_file = PathJoinSubstitution([
        FindPackageShare('stereo_image_proc_wrapper'),
        'config',
        'stereo_params.yaml'
    ])
    
    # 创建组合节点容器
    container = ComposableNodeContainer(
        name='stereo_complete_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # 左相机图像校正
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='left_rectify_node',
                remappings=[
                    ('image', '/camera/left/image_raw'),
                    ('camera_info', '/camera/left/camera_info'),
                    ('image_rect', '/camera/left/image_rect'),
                ],
            ),
            # 右相机图像校正
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='right_rectify_node',
                remappings=[
                    ('image', '/camera/right/image_raw'),
                    ('camera_info', '/camera/right/camera_info'),
                    ('image_rect', '/camera/right/image_rect'),
                ],
            ),
            # 视差计算
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::DisparityNode',
                name='disparity_node',
                parameters=[config_file],
                remappings=[
                    ('left/image_rect', '/camera/left/image_rect'),
                    ('left/camera_info', '/camera/left/camera_info'),
                    ('right/image_rect', '/camera/right/image_rect'),
                    ('right/camera_info', '/camera/right/camera_info'),
                    ('disparity', '/stereo/disparity'),
                ],
            ),
            # 点云生成
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::PointCloudNode',
                name='point_cloud_node',
                parameters=[config_file],
                remappings=[
                    ('left/camera_info', '/camera/left/camera_info'),
                    ('right/camera_info', '/camera/right/camera_info'),
                    ('left/image_rect_color', '/camera/left/image_rect'),
                    ('disparity', '/stereo/disparity'),
                    ('points2', '/stereo/points2'),
                ],
            ),
        ],
        output='screen',
    )
    
    return LaunchDescription([container])
```

### 使用完整流程

```bash
ros2 launch your_package complete_stereo_pipeline.launch.py
```

## 可视化输出

### 查看视差图

```bash
# 使用 rqt_image_view
ros2 run rqt_image_view rqt_image_view /stereo/disparity/image

# 或使用 rviz2
rviz2
```

### 查看点云

```bash
rviz2
```

在 RViz2 中：
1. 点击 "Add"
2. 选择 "PointCloud2"
3. Topic 设置为 `/stereo/points2`
4. Fixed Frame 设置为相机坐标系（例如 `camera_left_optical_frame`）

## 性能监控

```bash
# 查看话题频率
ros2 topic hz /stereo/disparity
ros2 topic hz /stereo/points2

# 查看话题信息
ros2 topic info /stereo/disparity
ros2 topic info /stereo/points2

# 查看节点信息
ros2 node list
ros2 node info /stereo_container
```

## 参数调整

### 实时调整参数

可以使用 `ros2 param` 命令实时调整参数：

```bash
# 列出所有参数
ros2 param list /stereo_container/disparity_node

# 设置参数
ros2 param set /stereo_container/disparity_node disparity_range 256

# 获取参数值
ros2 param get /stereo_container/disparity_node disparity_range
```

### 保存参数配置

```bash
ros2 param dump /stereo_container/disparity_node > my_custom_config.yaml
```

## 故障排除

### 检查输入话题

```bash
# 确认输入话题是否存在
ros2 topic list | grep camera

# 查看图像消息
ros2 topic echo /camera/left/image_rect --no-arr

# 查看相机信息
ros2 topic echo /camera/left/camera_info
```

### 检查输出

```bash
# 查看是否有输出
ros2 topic echo /stereo/disparity --no-arr
ros2 topic echo /stereo/points2 --no-arr
```

### 常见问题

1. **没有输出**: 检查输入话题是否正确，确保图像已校正
2. **点云质量差**: 调整 `stereo_params.yaml` 中的参数
3. **处理速度慢**: 使用 `fast_mode.yaml` 配置

## 与相机集成示例

### 与 USB 相机集成

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # 启动左相机
    left_camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='left_camera',
        parameters=[{
            'video_device': '/dev/video0',
            'camera_name': 'left_camera',
            # ... 其他参数
        }],
        remappings=[
            ('image_raw', '/camera/left/image_raw'),
            ('camera_info', '/camera/left/camera_info'),
        ]
    )
    
    # 启动右相机
    right_camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='right_camera',
        parameters=[{
            'video_device': '/dev/video1',
            'camera_name': 'right_camera',
            # ... 其他参数
        }],
        remappings=[
            ('image_raw', '/camera/right/image_raw'),
            ('camera_info', '/camera/right/camera_info'),
        ]
    )
    
    # 启动立体视觉处理
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
        left_camera,
        right_camera,
        stereo_proc,
    ])
```

## 进阶配置

### 自定义话题映射

修改 launch 文件中的 `remappings` 参数：

```python
remappings=[
    ('left/image_rect', '/my_custom/left/image'),
    ('right/image_rect', '/my_custom/right/image'),
    # ... 其他映射
]
```

### 命名空间

```bash
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py \
    namespace:=/robot1
```

## 下一步

- 阅读完整的 [README.md](README.md) 了解详细参数说明
- 查看 [COMPARISON.md](../COMPARISON.md) 了解与 stereo_processor 的区别
- 参考 ROS2 官方文档学习更多图像处理技术

## 参考资源

- [配置文件说明](config/stereo_params.yaml)
- [ROS2 stereo_image_proc](http://wiki.ros.org/stereo_image_proc)
- [ROS2 image_pipeline](https://github.com/ros-perception/image_pipeline)
