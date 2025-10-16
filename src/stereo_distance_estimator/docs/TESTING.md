# 完整系统测试

本目录包含用于测试完整立体视觉目标检测系统的启动文件和脚本。

## 系统架构

```
相机节点 (mindvision_camera)
    ↓
    ├─→ 左相机图像 (/camera_left/image_raw)
    └─→ 右相机图像 (/camera_right/image_raw)
         ↓
立体视觉处理 (stereo_image_proc_wrapper)
    ↓
    ├─→ 视差图 (/stereo/disparity)
    └─→ 点云 (/stereo/points2)
         ↓
目标检测 (object_detection_openvino)
    ↓
    └─→ 2D目标 (/detector/target2d_array)
         ↓
立体距离估计 (stereo_distance_estimator)
    ↓
    └─→ 3D目标 (/stereo/target3d_array_raw)
```

## 启动方式

### 方式1：使用 Launch 文件（推荐）

一键启动所有节点：

```bash
cd ~/Dart_2026_ws
source install/setup.bash
ros2 launch stereo_distance_estimator full_system_test.launch.py
```

### 方式2：使用 Bash 脚本

在独立终端中启动各个节点：

```bash
cd ~/Dart_2026_ws/src/stereo_distance_estimator/scripts
chmod +x test_full_system.sh
./test_full_system.sh
```

### 方式3：手动逐步启动

在不同终端中依次执行：

#### 终端 1: 双目相机
```bash
cd ~/Dart_2026_ws
source install/setup.bash
ros2 launch mindvision_camera dual_camera_launch.py
```

#### 终端 2: 立体视觉处理 (等待2-3秒)
```bash
cd ~/Dart_2026_ws
source install/setup.bash
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py
```

#### 终端 3: 目标检测 (等待2-3秒)
```bash
cd ~/Dart_2026_ws
source install/setup.bash
ros2 launch object_detection_openvino object_detection_openvino.launch.py
```

#### 终端 4: 立体距离估计 (等待2-3秒)
```bash
cd ~/Dart_2026_ws
source install/setup.bash
ros2 launch stereo_distance_estimator stereo_distance_estimator_config.launch.py
```

## 验证系统运行

### 检查所有话题
```bash
ros2 topic list
```

应该看到以下关键话题：
- `/camera_left/image_raw`
- `/camera_right/image_raw`
- `/stereo/disparity`
- `/stereo/points2`
- `/detector/target2d_array`
- `/stereo/target3d_array_raw`

### 查看话题频率
```bash
# 检查相机频率
ros2 topic hz /camera_left/image_raw

# 检查2D检测频率
ros2 topic hz /detector/target2d_array

# 检查3D目标频率
ros2 topic hz /stereo/target3d_array_raw
```

### 查看3D目标数据
```bash
ros2 topic echo /stereo/target3d_array_raw
```

### 使用 RViz2 可视化
```bash
rviz2
```

添加以下显示：
- **PointCloud2**: 订阅 `/stereo/points2`
- **Image**: 订阅 `/detector/debug_image`
- **MarkerArray**: 订阅 `/stereo/target3d_array_raw` (如果有可视化节点)

## 配置文件

主要配置文件位于：
- 距离估计器: `config/stereo_distance_estimator.yaml`

可以修改以下参数：
- `use_pointcloud`: 使用点云(true)或视差图(false)
- `max_distance`: 最大有效距离（米）
- `min_distance`: 最小有效距离（米）
- 相机内参（fx, fy, cx, cy, baseline）

## 故障排除

### 问题1: 没有3D目标输出

**可能原因**：
- 立体视觉处理未生成点云或视差图
- 目标检测没有检测到目标
- 相机图像不同步

**解决方案**：
```bash
# 检查点云是否发布
ros2 topic hz /stereo/points2

# 检查目标检测是否发布
ros2 topic hz /detector/target2d_array

# 查看节点日志
ros2 node list
ros2 node info /stereo_distance_estimator
```

### 问题2: 频率不匹配

节点现在使用最新缓存数据，不需要严格的时间同步。即使频率不同，只要有数据就能工作。

### 问题3: 空目标数组

当检测器发布空的 `target2d_array` 时，距离估计器也会发布空的 `target3d_array`。这是正常行为。

## 性能监控

查看系统资源使用：
```bash
# CPU 和内存使用
htop

# ROS 节点性能
ros2 run ros2_performance_test ros2_performance_test
```

## 日志级别

调整日志详细程度：
```bash
# 启动时设置日志级别
ros2 launch stereo_distance_estimator full_system_test.launch.py --log-level debug

# 运行时修改日志级别
ros2 service call /stereo_distance_estimator/set_log_level rcl_interfaces/srv/SetLogLevel "{level: 10}"
```

日志级别：
- DEBUG: 10
- INFO: 20
- WARN: 30
- ERROR: 40
- FATAL: 50

## 更多信息

详细文档请参考：
- [ARCHITECTURE.md](../ARCHITECTURE.md) - 系统架构
- [IMPLEMENTATION_SUMMARY.md](../IMPLEMENTATION_SUMMARY.md) - 实现细节
- [QUICKSTART.md](../QUICKSTART.md) - 快速开始指南
