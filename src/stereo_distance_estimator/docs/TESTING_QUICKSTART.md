# 完整系统测试 - 快速开始

## 🚀 最快启动方式

```bash
cd ~/Dart_2026_ws/src/stereo_distance_estimator/scripts
./quick_test.sh
```

## 📋 系统组件

| 序号 | 组件 | 功能 | 输出话题 |
|------|------|------|---------|
| 1 | mindvision_camera | 双目相机采集 | `/camera_left/image_raw`<br>`/camera_right/image_raw` |
| 2 | stereo_image_proc_wrapper | 立体视觉处理 | `/stereo/disparity`<br>`/stereo/points2` |
| 3 | object_detection_openvino | 目标检测 | `/detector/target2d_array` |
| 4 | stereo_distance_estimator | 3D距离估计 | `/stereo/target3d_array_raw` |

## 📁 文件结构

```
stereo_distance_estimator/
├── launch/
│   ├── full_system_test.launch.py          # 完整系统启动文件
│   ├── stereo_distance_estimator.launch.py
│   └── stereo_distance_estimator_config.launch.py
├── scripts/
│   ├── quick_test.sh                        # ⭐ 推荐：快速测试
│   ├── test_full_system.sh                  # 多终端测试
│   └── README.md                            # 脚本说明
├── docs/
│   └── TESTING.md                           # 详细测试文档
└── config/
    └── stereo_distance_estimator.yaml       # 配置文件
```

## 🎯 三种启动方式

### 方式 1: 一键启动（最简单）✨

```bash
cd ~/Dart_2026_ws/src/stereo_distance_estimator/scripts
./quick_test.sh
```

### 方式 2: 多终端启动（便于调试）

```bash
cd ~/Dart_2026_ws/src/stereo_distance_estimator/scripts
./test_full_system.sh
```

### 方式 3: Launch 文件启动

```bash
cd ~/Dart_2026_ws
source install/setup.bash
ros2 launch stereo_distance_estimator full_system_test.launch.py
```

## ✅ 验证系统

### 检查节点运行
```bash
ros2 node list
```

### 检查话题发布
```bash
ros2 topic list
```

### 查看3D目标频率
```bash
ros2 topic hz /stereo/target3d_array_raw
```

### 查看3D目标数据
```bash
ros2 topic echo /stereo/target3d_array_raw
```

## 🔧 配置修改

编辑配置文件：
```bash
nano ~/Dart_2026_ws/src/stereo_distance_estimator/config/stereo_distance_estimator.yaml
```

关键参数：
- `target2d_topic`: 2D目标输入话题
- `use_pointcloud`: true=使用点云, false=使用视差图
- `max_distance`: 最大有效距离（米）
- `min_distance`: 最小有效距离（米）

## 📊 系统数据流

```
📷 相机 (38 Hz)
    ↓
🔲 立体处理 (4 Hz)
    ↓
🎯 目标检测 (38 Hz)
    ↓
📏 距离估计 (38 Hz)
    ↓
📦 3D目标输出
```

## 🐛 常见问题

### Q: 没有3D目标输出？
```bash
# 检查输入话题
ros2 topic hz /detector/target2d_array
ros2 topic hz /stereo/points2

# 查看节点日志
ros2 node info /stereo_distance_estimator
```

### Q: targets 数组为空？
这是正常的！当检测器没有检测到目标时，会发布空数组，距离估计器也会输出空数组。

### Q: 频率不匹配？
新版本使用缓存机制，不需要严格的时间同步。只要话题有数据就能工作。

## 📖 更多文档

- 详细测试指南: [docs/TESTING.md](docs/TESTING.md)
- 脚本说明: [scripts/README.md](scripts/README.md)
- 系统架构: [ARCHITECTURE.md](ARCHITECTURE.md)
- 实现总结: [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md)

## 🎉 快速测试步骤

1. **启动系统**
   ```bash
   cd ~/Dart_2026_ws/src/stereo_distance_estimator/scripts
   ./quick_test.sh
   ```

2. **验证运行**
   ```bash
   # 新开一个终端
   ros2 topic hz /stereo/target3d_array_raw
   ```

3. **查看数据**
   ```bash
   ros2 topic echo /stereo/target3d_array_raw
   ```

4. **停止系统**
   在运行 quick_test.sh 的终端按 `Ctrl+C`

---

**就这么简单！** 🎊
