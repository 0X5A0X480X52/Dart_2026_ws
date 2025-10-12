# MindVision 多相机快速开始指南

## 🚀 快速开始

### 步骤 1: 查看相机序列号

```bash
# Source 你的工作空间
source /home/amatrix/Dart_2026_ws/install/setup.bash

# 列出所有连接的相机
ros2 launch mindvision_camera list_cameras_launch.py
```

你会看到类似这样的输出：
```
[INFO] [mv_camera]: Found camera count = 2
[INFO] [mv_camera]: Camera 0: MV-U300 (SN: A1B2C3D4E5F6)
[INFO] [mv_camera]: Camera 1: MV-U300 (SN: F6E5D4C3B2A1)
```

记下这些序列号！

### 步骤 2: 配置相机参数

编辑配置文件：
```bash
nano ~/Dart_2026_ws/src/rm_hardware_driver/ros2_mindvision_camera/config/dual_camera_params.yaml
```

将序列号替换为你的实际序列号：
```yaml
/camera_left:
  ros__parameters:
    camera_sn: "A1B2C3D4E5F6"  # ← 替换这里
    ...

/camera_right:
  ros__parameters:
    camera_sn: "F6E5D4C3B2A1"  # ← 替换这里
    ...
```

### 步骤 3: 启动双相机

```bash
ros2 launch mindvision_camera dual_camera_launch.py
```

### 步骤 4: 查看图像

在新终端中：
```bash
# 查看左相机
rqt_image_view /camera_left/image_raw

# 或查看右相机
rqt_image_view /camera_right/image_raw
```

## 📋 常用命令

### 启动单个相机（指定序列号）

```bash
ros2 launch mindvision_camera mv_launch_with_sn.py camera_sn:=A1B2C3D4E5F6
```

### 启动单个相机（默认第一个）

```bash
ros2 launch mindvision_camera mv_launch.py
```

### 手动启动特定相机到自定义命名空间

```bash
ros2 run mindvision_camera mindvision_camera_node --ros-args \
  -r __ns:=/my_camera \
  -p camera_sn:="YOUR_SN" \
  -p camera_name:=my_camera \
  -p exposure_time:=3500
```

## 🔍 检查话题

```bash
# 列出所有图像话题
ros2 topic list | grep image

# 查看相机信息
ros2 topic echo /camera_left/camera_info

# 查看图像话题信息
ros2 topic info /camera_left/image_raw
```

## ⚙️ 动态调整参数

```bash
# 查看可用参数
ros2 param list /camera_left/mv_camera

# 修改曝光时间
ros2 param set /camera_left/mv_camera exposure_time 5000

# 修改增益
ros2 param set /camera_left/mv_camera analog_gain 80
```

## 📊 性能监控

```bash
# 查看帧率
ros2 topic hz /camera_left/image_raw

# 查看带宽使用
ros2 topic bw /camera_left/image_raw
```

## 🛠️ 故障排查

### 相机未检测到
```bash
# 检查 USB 设备
lsusb | grep Mind

# 检查权限
ls -l /dev/video*
```

### 序列号不匹配
```bash
# 重新扫描相机
ros2 launch mindvision_camera list_cameras_launch.py
```

### 节点启动失败
```bash
# 查看详细日志
ros2 launch mindvision_camera dual_camera_launch.py --ros-args --log-level debug
```

## 📚 更多信息

- 详细配置指南: `docs/multi_camera_guide.md`
- 更新日志: `docs/CHANGELOG_multi_camera.md`
- 主 README: `README.md`

## 💡 提示

1. **USB 带宽**: 如果使用高分辨率或高帧率，确保将相机连接到不同的 USB 控制器
2. **命名空间**: 使用有意义的命名空间名称，如 `/camera_front`, `/camera_back`
3. **参数调整**: 可以在运行时通过 rqt 的 Dynamic Reconfigure 插件调整参数
4. **标定**: 每个相机需要单独进行标定
