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
