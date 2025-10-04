# Stereo Rectifier 包创建总结

## ✅ 任务完成

已成功在 `stereo_vision` 目录下创建 `stereo_rectifier` 包，该包基于 ROS2 官方的 `image_proc` 进行图像校正，并集成了海康威视（HIK）和迈德威视（MindVision）相机驱动。

## 📦 创建的文件

### 核心配置文件
1. **package.xml** - ROS2 包定义
2. **CMakeLists.txt** - 构建配置

### Launch 文件 (4个)
3. **launch/stereo_rectify.launch.py** - 仅图像校正（假设相机已运行）
4. **launch/hik_stereo_rectify.launch.py** - 海康威视相机 + 图像校正
5. **launch/mindvision_stereo_rectify.launch.py** - 迈德威视相机 + 图像校正
6. **launch/complete_stereo_pipeline.launch.py** - 完整流程（相机+校正+立体处理）

### 配置文件 (3个)
7. **config/left_camera_info.yaml** - 左相机标定参数模板
8. **config/right_camera_info.yaml** - 右相机标定参数模板
9. **config/stereo_calibration.yaml** - 完整立体标定示例

### 文档文件 (2个)
10. **README.md** - 完整使用文档
11. **QUICKSTART.md** - 快速入门指南

### 集成文档 (1个)
12. **../INTEGRATION_GUIDE.md** - 完整立体视觉流程集成指南

## 🎯 主要功能

### 1. 相机驱动集成
- ✅ 支持海康威视（HIK）工业相机
- ✅ 支持迈德威视（MindVision）工业相机
- ✅ 使用组合节点容器实现高性能

### 2. 图像校正
- ✅ 基于 ROS2 官方 `image_proc`
- ✅ 畸变校正（distortion correction）
- ✅ 立体校正（stereo rectification）
- ✅ 输出标准化的校正图像

### 3. 灵活使用
- ✅ 可独立使用（仅图像校正）
- ✅ 可与相机驱动集成使用
- ✅ 可与 `stereo_image_proc_wrapper` 配合构建完整流程

## 📋 Topic 接口

### 输入
- `/camera/left/image_raw` - 左相机原始图像
- `/camera/right/image_raw` - 右相机原始图像
- `/camera/left/camera_info` - 左相机标定信息
- `/camera/right/camera_info` - 右相机标定信息

### 输出
- `/camera/left/image_rect` - 校正后的左图像（单通道）
- `/camera/right/image_rect` - 校正后的右图像（单通道）
- `/camera/left/image_rect_color` - 校正后的左图像（彩色）
- `/camera/right/image_rect_color` - 校正后的右图像（彩色）

## 🚀 快速使用

### 1. 编译
```bash
cd ~/Dart_2026_ws
colcon build --packages-select stereo_rectifier
source install/setup.bash
```

### 2. 准备标定文件
```bash
# 使用 ROS2 标定工具进行立体标定
ros2 run camera_calibration cameracalibrator \
    --size 8x6 --square 0.025 --approximate 0.1 \
    --ros-args -r left:=/camera/left/image_raw \
                -r right:=/camera/right/image_raw
```

### 3. 启动系统

#### 选项 A: 仅图像校正
```bash
ros2 launch stereo_rectifier stereo_rectify.launch.py
```

#### 选项 B: 海康威视相机 + 校正
```bash
ros2 launch stereo_rectifier hik_stereo_rectify.launch.py
```

#### 选项 C: 迈德威视相机 + 校正
```bash
ros2 launch stereo_rectifier mindvision_stereo_rectify.launch.py
```

#### 选项 D: 完整立体视觉流程
```bash
ros2 launch stereo_rectifier complete_stereo_pipeline.launch.py
```

## 🔗 与其他包集成

### 完整立体视觉流程

**方法 1: 使用集成 launch 文件**
```bash
ros2 launch stereo_rectifier complete_stereo_pipeline.launch.py
```

**方法 2: 分步启动**
```bash
# 终端 1: 相机 + 图像校正
ros2 launch stereo_rectifier hik_stereo_rectify.launch.py

# 终端 2: 立体视觉处理
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py
```

### 数据流

```
相机硬件
    ↓ (raw images)
stereo_rectifier
    ↓ (rectified images)
stereo_image_proc_wrapper
    ↓ (disparity + point cloud)
应用层（目标检测、导航等）
```

## 🎨 可视化

### 查看图像
```bash
# 原始图像
ros2 run rqt_image_view rqt_image_view /camera/left/image_raw

# 校正图像
ros2 run rqt_image_view rqt_image_view /camera/left/image_rect
```

### 查看点云（需要先启动完整流程）
```bash
rviz2
# 添加 PointCloud2 显示，Topic: /stereo/points2
```

## 📊 技术特点

### 1. 高性能设计
- 使用 `ComposableNodeContainer` 组合节点容器
- 启用进程内通信（零拷贝）
- 降低 CPU 使用率和延迟

### 2. 标准化接口
- 基于 ROS2 官方 `image_proc`
- 输出符合 ROS2 标准的消息类型
- 与 ROS2 生态系统完全兼容

### 3. 相机驱动集成
- 直接集成 HIK 和 MindVision 相机驱动
- 统一的 launch 文件管理
- 简化系统部署和使用

## 🆚 与 stereo_processor 的区别

| 特性 | stereo_rectifier | stereo_processor |
|------|-----------------|------------------|
| 功能范围 | 仅图像校正 | 完整流程（校正+视差+点云） |
| 实现方式 | ROS2 官方 image_proc | 自定义 OpenCV 实现 |
| 相机集成 | ✅ 内置 | ❌ 需要单独启动 |
| 性能 | 高（零拷贝） | 中等 |
| 维护 | 官方维护 | 自行维护 |
| 适用场景 | 生产部署 | 学习研究 |

## 📁 目录结构

```
stereo_rectifier/
├── CMakeLists.txt
├── package.xml
├── README.md                              # 详细文档
├── QUICKSTART.md                          # 快速入门
├── config/
│   ├── left_camera_info.yaml             # 左相机标定
│   ├── right_camera_info.yaml            # 右相机标定
│   └── stereo_calibration.yaml           # 立体标定示例
└── launch/
    ├── stereo_rectify.launch.py          # 仅校正
    ├── hik_stereo_rectify.launch.py      # HIK相机+校正
    ├── mindvision_stereo_rectify.launch.py # MindVision相机+校正
    └── complete_stereo_pipeline.launch.py # 完整流程
```

## ⚙️ 配置参数

### 相机参数
- `camera_name`: 相机名称
- `camera_info_url`: 标定文件路径
- `frame_id`: 坐标系名称
- `serial_number`: 相机序列号（HIK）

### Launch 参数示例
```bash
ros2 launch stereo_rectifier hik_stereo_rectify.launch.py \
    left_camera_sn:=00J12345678 \
    right_camera_sn:=00J12345679 \
    left_camera_info_url:=file:///path/to/left_calib.yaml \
    right_camera_info_url:=file:///path/to/right_calib.yaml
```

## 🔧 故障排除

### 问题 1: 没有校正图像输出
**原因**: 标定文件缺失或无效
**解决**: 检查标定文件是否存在，使用 `ros2 topic echo /camera/left/camera_info` 验证

### 问题 2: 相机无法打开
**原因**: 相机连接或权限问题
**解决**: 检查相机连接、电源、驱动权限

### 问题 3: 图像有黑边
**原因**: 这是正常的畸变校正效果
**说明**: 校正后边缘会有无效区域，可在后续处理中裁剪

## 📚 相关文档

- [README.md](README.md) - 完整使用文档
- [QUICKSTART.md](QUICKSTART.md) - 快速入门
- [INTEGRATION_GUIDE.md](../INTEGRATION_GUIDE.md) - 集成指南
- [stereo_image_proc_wrapper](../stereo_image_proc_wrapper/) - 立体处理包
- [stereo_processor](../stereo_processor/) - 自定义实现包

## 🎓 下一步

1. ✅ 完成相机标定
2. ✅ 测试图像校正效果
3. ✅ 集成立体视觉处理
4. ✅ 开发应用程序使用点云数据

## 📝 更新日志

### 0.0.0 (2025-10-04)
- ✅ 初始版本
- ✅ 支持 HIK 和 MindVision 相机
- ✅ 基于 image_proc 的图像校正
- ✅ 组合节点容器优化
- ✅ 完整流程 launch 文件
- ✅ 详细文档和示例

## 🙏 致谢

- ROS2 官方 image_proc 团队
- HIK 和 MindVision 相机驱动开发者
- ROS2 社区

---

**维护者**: amatrix02 (3432900546@qq.com)
**许可证**: Apache-2.0
**创建日期**: 2025-10-04
