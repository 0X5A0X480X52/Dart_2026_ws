# 测试脚本说明

此目录包含用于测试立体视觉距离估计系统的便捷脚本。

## 可用脚本

### 1. quick_test.sh (推荐)
**最简单的启动方式** - 使用 ROS2 launch 文件启动所有节点。

```bash
./quick_test.sh
```

**优点**：
- 一个命令启动所有节点
- 自动处理节点启动顺序
- 所有日志在一个终端中
- 容易停止（Ctrl+C 停止所有）

**缺点**：
- 所有日志混在一起，可能难以阅读

---

### 2. test_full_system.sh
在独立的终端窗口中启动每个节点。

```bash
./test_full_system.sh
```

**优点**：
- 每个节点在独立终端中
- 日志分离，易于调试
- 可以单独查看每个节点的输出

**缺点**：
- 需要手动关闭每个终端
- 仅在有图形界面时工作（gnome-terminal）

---

## 使用建议

### 开发和调试
使用 `test_full_system.sh`，可以清楚地看到每个节点的输出。

### 日常测试
使用 `quick_test.sh`，快速启动和停止。

### 生产部署
直接使用 launch 文件：
```bash
ros2 launch stereo_distance_estimator full_system_test.launch.py
```

---

## 手动测试

如果脚本不工作，可以手动在4个终端中依次运行：

**终端 1 - 相机:**
```bash
cd ~/Dart_2026_ws && source install/setup.bash
ros2 launch mindvision_camera dual_camera_launch.py
```

**终端 2 - 立体处理 (等待2秒):**
```bash
cd ~/Dart_2026_ws && source install/setup.bash
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py
```

**终端 3 - 目标检测 (等待2秒):**
```bash
cd ~/Dart_2026_ws && source install/setup.bash
ros2 launch object_detection_openvino object_detection_openvino.launch.py
```

**终端 4 - 距离估计 (等待2秒):**
```bash
cd ~/Dart_2026_ws && source install/setup.bash
ros2 launch stereo_distance_estimator stereo_distance_estimator_config.launch.py
```

---

## 验证系统运行

### 检查所有节点
```bash
ros2 node list
```

应该看到：
- `/camera_left`
- `/camera_right`
- `/stereo_image_proc`
- `/object_detection`
- `/stereo_distance_estimator`

### 检查关键话题
```bash
ros2 topic list | grep -E "(image_raw|disparity|points2|target)"
```

### 监控输出频率
```bash
# 2D 检测
ros2 topic hz /detector/target2d_array

# 3D 目标
ros2 topic hz /stereo/target3d_array_raw
```

### 查看实时数据
```bash
ros2 topic echo /stereo/target3d_array_raw
```

---

## 故障排除

### 脚本无法执行
```bash
chmod +x *.sh
```

### 找不到包
```bash
cd ~/Dart_2026_ws
source install/setup.bash
```

### 节点没有数据输出
检查每个节点是否正常运行：
```bash
ros2 node list
ros2 topic list
```

---

## 更多信息

详细测试文档: [../docs/TESTING.md](../docs/TESTING.md)
