# Dart 2026 Workspace

本项目基于 [深圳大学电控与视觉实现镖架制导代码](https://github.com/Elucidater01/2024_Dart_Algorithm.git) 进行开发。

## 工作空间结构

``` markdown
Dart_2026_ws/
└── src/
    ├── filter/                          # 滤波算法模块
    │   ├── basic_models/                   # 基础滤波模型（KF、EKF 等）
    │   ├── combined_models/                # 组合滤波模型（IMM 等）
    │   ├── coordinate_filter/              # 坐标滤波器
    │   ├── distance_filter/                # 距离滤波器
    │   └── models/                         # 通用滤波模型定义
    ├── object_detection_openvino/       # 基于 OpenVINO 的目标检测
    ├── rm_bringup/                      # 系统启动配置
    ├── rm_hardware_driver/              # 硬件驱动模块
    │   ├── rm_serial_driver/               # 串口通信驱动
    │   └── stereo_camera_driver/           # 双目相机驱动
    ├── rm_robot_description/            # 机器人模型描述
    ├── stereo_distance_estimator/       # 双目距离估算
    ├── stereo_msgs/                     # 双目视觉相关消息定义
    └── target_matcher/                  # 目标匹配模块
```

## 依赖项

本项目在 Ubuntu 22.04 和 ROS2 Humble 环境下开发，主要依赖以下库：

### ROS2

本项目的环境为 ROS2 Humble，安装请参考 [ROS2 官方安装网站](https://docs.ros.org/en/humble/Installation.html)。

### opencv

```bash
sudo apt install libopencv-dev
```

### openvino

请参考 [OpenVINO 官方安装网站](https://www.intel.com/content/www/us/en/developer/tools/openvino-toolkit/download.html) 进行安装。

### 相机驱动

在 ``src/rm_hardware_driver/`` 下需要安装相应的相机驱动包。本项目提供了海康相机、大恒相机、mindvision 相机的驱动，直接编译即可。

具体使用哪个相机驱动，请在 ``src/rm_bringup/launch/bringup.launch.py`` 中进行配置。

### stereo_image_proc

`stereo_image_proc` 用于双目图像的处理和视差计算。其安装命令如下：

```bash
sudo apt install ros-humble-image-pipeline
```

## 构建并运行

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch rm_bringup bringup.launch.py
```

## 使用 `foxglove` 可视化

下载安装 foxglove_bridge：

```bash
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
```

启动 foxglove_bridge：

```bash
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

启动后可使用 foxglove_bridge 相应接收终端进行可视化调试

## Quick Start

在不同终端中分别启动：

主检测ros2脚本：
```bash
source install/setup.bash
ros2 launch rm_bringup stereo_yolo_system.launch.py
```

串口ros2脚本：
```bash
source install/setup.bash
ros2 launch rm_serial_driver serial_driver.launch.py
```

foxglove后端：
```bash
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

更改瞄准中心：修改配置文件 `src/rm_hardware_driver/rm_serial_driver/config/serial_driver_params.yaml`

```yaml
    # 理想目标点在图像坐标系中的横坐标，应根据实际情况调整
    # 注意：该参数需要手动修改，串口节点暂时不会自动修改这个参数
    desiredPosX: 658 
```

保存yaml配置文件后重新启动串口ros2脚本。

调整 `src/rm_bringup/launch/stereo_yolo_system.launch.py` 中 `parameters` ：

```python
'roi_width': 320,   # ROI 框宽度
'roi_height': 240,  # ROI 框高度
'center_x': 700.0,  # ROI 框中心点像素x坐标
'center_y': -1.0,   # ROI 框中心点像素y坐标
```

可更改左右检测 ROI 位置及大小

```python
# ========== Stage 2: 左右双路YOLO检测 ==========
    
    # 获取YOLO配置目录
    yolo_pkg_dir = get_package_share_directory('object_detection_openvino')
    yolo_config = os.path.join(yolo_pkg_dir, 'config', 'params.yaml')
    
    # 左相机YOLO检测节点
    left_yolo_node = Node(
        package='object_detection_openvino',
        executable='object_detection_openvino_node',
        name='object_detection_left',
        namespace='detector_left',
        output='screen',
        parameters=[
            yolo_config,
            {
                'image_topic': '/camera_left/image_raw',
                'detection_topic': '/detector/left/target2d_array',
                'debug_image_topic': '/detector/left/debug_image',
                'roi_mode': 'center',
                'roi_width': 320,
                'roi_height': 240,
                'center_x': 700.0, 
                'center_y': -1.0,
            }
        ],
        remappings=[
            ('image_raw', '/camera_left/image_raw'),
            ('/detector/target2d_array', '/detector/left/target2d_array'),
            ('/detector/debug_image', '/detector/left/debug_image'),
        ]
    )
    
    # 右相机YOLO检测节点
    right_yolo_node = Node(
        package='object_detection_openvino',
        executable='object_detection_openvino_node',
        name='object_detection_right',
        namespace='detector_right',
        output='screen',
        parameters=[
            yolo_config,
            {
                'image_topic': '/camera_right/image_raw',
                'detection_topic': '/detector/right/target2d_array',
                'debug_image_topic': '/detector/right/debug_image',
                'roi_mode': 'center',
                'roi_width': 320,
                'roi_height': 240,
                'center_x': 500.0,
                'center_y': -1.0,
            }
        ],
        remappings=[
            ('image_raw', '/camera_right/image_raw'),
            ('/detector/target2d_array', '/detector/right/target2d_array'),
            ('/detector/debug_image', '/detector/right/debug_image'),
        ]
    )
```

保存后重启主检测ros2脚本source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml