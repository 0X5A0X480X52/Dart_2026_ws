# ros2_mindvision_camera

ROS2 MindVision 相机包，提供了 MindVision 相机的 ROS API。

Only tested under Ubuntu 22.04 with ROS2 Humble

![Build Status](https://github.com/chenjunnn/ros2_mindvision_camera/actions/workflows/ros_ci.yml/badge.svg)

## 使用说明

### Build from source

#### Dependencies

- [Robot Operating System 2 (ROS2)](https://docs.ros.org/en/humble/) (middleware for robotics),

#### Building

To build from source, clone the latest version from this repository into your colcon workspace and compile the package using

 mkdir -p ros_ws/src
 cd ros_ws/src
 git clone <https://github.com/chenjunnn/ros2_mindvision_camera.git>
 cd ..
 rosdep install --from-paths src --ignore-src -r -y
 colcon build --symlink-install --packages-up-to mindvision_camera

### 标定

标定教程可参考 <https://navigation.ros.org/tutorials/docs/camera_calibration.html>

参数意义请参考 <http://wiki.ros.org/camera_calibration>

标定后的相机参数会被存放在 `/tmp/calibrationdata.tar.gz`

### 启动相机节点

#### 单相机模式

    ros2 launch mindvision_camera mv_launch.py

支持的参数：

1. params_file： 相机参数文件的路径
2. camera_info_url： 相机内参文件的路径
3. use_sensor_data_qos： 相机 Publisher 是否使用 SensorDataQoS (default: `false`)

#### 多相机模式

**1. 查看所有连接的相机信息和序列号**:

```bash
ros2 launch mindvision_camera list_cameras_launch.py
```

这将列出所有检测到的相机及其序列号（SN）。

**2. 配置多相机参数**：

编辑 `config/dual_camera_params.yaml` 文件，为每个相机指定序列号：

```yaml
/camera_left:
  ros__parameters:
    camera_name: camera_left
    camera_sn: "YOUR_LEFT_CAMERA_SN"  # 替换为实际序列号
    ...

/camera_right:
  ros__parameters:
    camera_name: camera_right
    camera_sn: "YOUR_RIGHT_CAMERA_SN"  # 替换为实际序列号
    ...
```

**3. 启动多个相机**：

```bash
ros2 launch mindvision_camera dual_camera_launch.py
```

这将同时启动两个相机节点，分别在 `/camera_left` 和 `/camera_right` 命名空间下。

**注意事项：**

- `camera_sn` 参数留空时，节点将使用第一个检测到的相机
- 每个相机必须指定唯一的序列号以避免冲突
- 可以通过修改 launch 文件添加更多相机节点

### 通过 rqt 动态调节相机参数

打开 rqt，在 Plugins 中添加 `Configuration -> Dynamic Reconfigure` 及 `Visualization -> Image View`

<!-- ![](docs/rqt.png) -->
