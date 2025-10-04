# ros2_hik_camera - HIKVision相机ROS2驱动

## ✅ 项目状态

本项目已完成HIKVision工业相机的ROS2封装，参考ros2_mindvision_camera的架构实现。

**当前版本**: v0.1.0  
**测试相机**: MV-CS016-10UC  
**ROS2版本**: Humble/Foxy兼容

## 🎯 主要特性

- ✅ 完全的ROS2组件化架构 (rclcpp_components)
- ✅ 动态参数调整（曝光、增益、帧率）
- ✅ image_transport集成，支持图像压缩
- ✅ camera_info_manager支持，完整相机标定
- ✅ 独立图像采集线程，不阻塞ROS2主循环
- ✅ 完善的错误处理和诊断工具
- ✅ 详细的文档和故障排查指南

## 📦 快速开始

### 1. 编译

```bash
cd ~/Dart_2026_ws
colcon build --packages-select ros2_hik_camera
source install/setup.bash
```

### 2. 运行

```bash
# 使用launch文件（推荐）
ros2 launch ros2_hik_camera hik_camera_launch.py

# 或直接运行节点
ros2 run ros2_hik_camera ros2_hik_camera_node
```

### 3. 查看图像

```bash
# 方法1: rqt_image_view
ros2 run rqt_image_view rqt_image_view

# 方法2: rviz2
rviz2
# 然后添加Image显示，选择/image_raw话题
```

## ⚙️ 配置

### 默认配置 (config/camera_params.yaml)

```yaml
/hik_camera:
  ros__parameters:
    camera_name: hik_camera
    exposure_time: 5000.0  # 曝光时间(微秒)
    gain: 8.0              # 增益(0~16)
    frame_rate: 10.0       # 帧率(fps) - 建议从10开始
    flip_image: false      # 是否翻转图像
```

### 运行时调整参数

```bash
# 调整曝光
ros2 param set /hik_camera exposure_time 8000.0

# 调整增益
ros2 param set /hik_camera gain 12.0

# 调整帧率
ros2 param set /hik_camera frame_rate 20.0
```

## 🔧 故障排查

### 常见问题

#### 问题1: MV_E_NODATA (0x80000007) 错误

**症状**：
```
[WARN] MV_E_NODATA (0x80000007): No data available
```

**原因**：相机帧率设置过高，或者取流超时时间过短

**解决**：
1. 降低帧率：
```bash
ros2 run ros2_hik_camera ros2_hik_camera_node --ros-args -p frame_rate:=10.0
```

2. 或修改 `config/camera_params.yaml`:
```yaml
frame_rate: 10.0  # 从30降到10
```

详细说明请查看 [TROUBLESHOOTING.md](TROUBLESHOOTING.md)

#### 问题2: 找不到相机

**解决步骤**：
```bash
# 运行诊断脚本
python3 scripts/diagnose.py

# 检查USB连接
lsusb | grep -i hik

# 检查权限
groups | grep -E "video|plugdev"
```

#### 问题3: 编译错误

确保HIKVision SDK已正确安装：
```bash
ls /opt/MVS/lib/64/libMvCameraControl.so
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/MVS/lib/64
```

## 📖 文档

- [README.md](README.md) - 项目概述和基本使用
- [USAGE.md](USAGE.md) - 详细使用指南
- [TROUBLESHOOTING.md](TROUBLESHOOTING.md) - 问题排查和解决方案

## 🛠️ 工具脚本

### 诊断工具
```bash
python3 scripts/diagnose.py
```
检查USB设备、SDK安装、权限等

### 测试脚本
```bash
./scripts/test_camera.sh
```
测试不同帧率下的相机性能

### 构建脚本
```bash
./build.sh
```
快速编译包

## 📝 发布话题

- `/image_raw` (sensor_msgs/Image) - 原始图像数据
- `/camera_info` (sensor_msgs/CameraInfo) - 相机标定信息

## 🔄 与ros2_mindvision_camera对比

| 特性 | ros2_mindvision_camera | ros2_hik_camera | 说明 |
|------|------------------------|-----------------|------|
| 架构模式 | rclcpp_components | rclcpp_components | ✓ 完全相同 |
| 动态参数 | ✓ | ✓ | 相同的实现方式 |
| image_transport | ✓ | ✓ | 相同的发布机制 |
| camera_info | ✓ | ✓ | 相同的标定管理 |
| 独立线程 | ✓ | ✓ | 相同的线程模型 |
| SDK API | MindVision | HIKVision | 仅底层API不同 |
| 像素转换 | CameraImageProcess | MV_CC_ConvertPixelType | SDK差异 |
| 图像获取 | CameraGetImageBuffer | MV_CC_GetOneFrameTimeout | SDK差异 |

**结论**: 除了底层SDK调用，架构完全一致！

## 🎓 学习资源

- [HIKVision MVS SDK文档](https://www.hikrobotics.com/)
- [ROS2 Image Pipeline](https://github.com/ros-perception/image_pipeline)
- [camera_calibration](http://wiki.ros.org/camera_calibration)

## 📊 测试结果

### 测试环境
- 相机: MV-CS016-10UC
- USB: USB 3.0
- 分辨率: 1280x1024
- 系统: Ubuntu 22.04 + ROS2 Humble

### 性能表现

| 帧率设置 | 实际帧率 | 稳定性 | 推荐场景 |
|---------|---------|--------|----------|
| 10 fps  | ~10 fps | ⭐⭐⭐⭐⭐ | 调试、开发 |
| 20 fps  | ~20 fps | ⭐⭐⭐⭐ | 一般应用 |
| 30 fps  | ~25-30 fps | ⭐⭐⭐ | 高速应用 |
| 50 fps  | 不稳定 | ⭐⭐ | 需要优化 |

## 🤝 贡献

欢迎提交Issues和Pull Requests！

## 📄 许可证

MIT License

## 👥 作者

- amatrix
- 参考项目: [ros2_mindvision_camera](https://github.com/chenjunnn/ros2_mindvision_camera)

## 🙏 致谢

感谢ros2_mindvision_camera项目提供的优秀架构参考！
