# RM Bringup 包 - 使用演示

## 已完成！ ✅

我已经为你在 `src/rm_bringup` 包中创建了一个完整的系统启动解决方案。

## 📁 创建的文件

```
src/rm_bringup/
├── CMakeLists.txt              # 更新了安装配置
├── package.xml                 # 添加了依赖
├── README.md                   # 详细文档
├── QUICKSTART.md              # 快速参考
├── SUMMARY.md                 # 完成总结
├── config/
│   ├── launch_config.yaml            # 默认配置
│   └── launch_config_example.yaml    # 配置示例
├── docs/
│   └── system_architecture.dot       # 系统架构图
├── launch/
│   └── system_bringup.launch.py      # 主启动文件
└── scripts/
    └── start_system.sh                # 快速启动脚本
```

## 🎯 功能特性

### 1. 按顺序启动
- **Stage 1** (t=0s): 启动双相机驱动
- **Stage 2** (t=5s): 并行启动目标检测和立体图像处理
- **Stage 3** (t=10s): 启动距离估计

### 2. 灵活配置
- 通过 YAML 配置文件控制启用/禁用节点
- 可以指定不同的 launch 文件
- 可以调整启动延迟时间

### 3. 易于使用
- 提供快速启动脚本
- 支持命令行参数
- 详细的文档和示例

## 🚀 立即开始使用

### 第一步：编译（已完成）
```bash
cd /home/amatrix/Dart_2026_ws
colcon build --packages-select rm_bringup
source install/setup.bash
```

### 第二步：启动系统

**方法1：使用启动脚本（推荐）**
```bash
cd /home/amatrix/Dart_2026_ws
./src/rm_bringup/scripts/start_system.sh
```

**方法2：使用 ros2 launch**
```bash
cd /home/amatrix/Dart_2026_ws
source install/setup.bash
ros2 launch rm_bringup system_bringup.launch.py
```

## 📝 快速命令参考

```bash
# 1. 基本启动
ros2 launch rm_bringup system_bringup.launch.py

# 2. 自定义延迟（如果节点需要更多启动时间）
ros2 launch rm_bringup system_bringup.launch.py stage2_delay:=8.0 stage3_delay:=15.0

# 3. 使用自定义配置文件
ros2 launch rm_bringup system_bringup.launch.py config_file:=/path/to/config.yaml

# 4. 使用启动脚本
./src/rm_bringup/scripts/start_system.sh              # 默认
./src/rm_bringup/scripts/start_system.sh -2 8.0      # 自定义Stage 2延迟
./src/rm_bringup/scripts/start_system.sh -c my.yaml  # 自定义配置
./src/rm_bringup/scripts/start_system.sh -h          # 查看帮助
```

## ⚙️ 自定义配置

### 修改启动的 launch 文件

编辑 `src/rm_bringup/config/launch_config.yaml`：

```yaml
stage1:
  camera:
    package: 'ros2_mindvision_camera'
    launch_file: 'dual_camera_launch.py'  # 可以改成其他launch文件
    enabled: true

stage2:
  object_detection:
    package: 'object_detection_openvino'
    launch_file: 'object_detection_openvino.launch.py'
    enabled: true  # 设为false可禁用
  
  stereo_image_proc:
    package: 'stereo_image_proc_wrapper'
    launch_file: 'stereo_image_proc.launch.py'
    enabled: true

stage3:
  distance_estimator:
    package: 'stereo_distance_estimator'
    launch_file: 'stereo_distance_estimator_config.launch.py'
    enabled: true
```

### 创建多个配置文件

```bash
# 复制默认配置
cp src/rm_bringup/config/launch_config.yaml my_test_config.yaml

# 编辑配置...
nano my_test_config.yaml

# 使用自定义配置启动
ros2 launch rm_bringup system_bringup.launch.py config_file:=my_test_config.yaml
```

## 🔍 调试技巧

### 1. 测试单个阶段
禁用其他阶段来单独测试某个阶段：
```yaml
stage1:
  camera:
    enabled: true  # 只测试相机
stage2:
  object_detection:
    enabled: false
  stereo_image_proc:
    enabled: false
stage3:
  distance_estimator:
    enabled: false
```

### 2. 增加启动延迟
如果节点启动太快导致问题：
```bash
ros2 launch rm_bringup system_bringup.launch.py stage2_delay:=10.0 stage3_delay:=20.0
```

### 3. 查看运行状态
```bash
# 查看所有节点
ros2 node list

# 查看话题
ros2 topic list

# 查看节点信息
ros2 node info /node_name
```

## 📚 文档

- **README.md** - 完整的使用说明和配置指南
- **QUICKSTART.md** - 快速参考卡
- **SUMMARY.md** - 项目完成总结
- **config/launch_config_example.yaml** - 配置文件示例

## ❓ 常见问题

### Q: 如何更改启动顺序？
A: 启动顺序是固定的（Stage 1 → Stage 2 → Stage 3），但你可以通过配置文件禁用某些阶段。

### Q: 如何调整延迟时间？
A: 使用 `stage2_delay` 和 `stage3_delay` 参数，或在配置文件中设置。

### Q: 如何使用不同的 launch 文件？
A: 在配置文件中修改 `launch_file` 字段。

### Q: 修改配置文件后需要重新编译吗？
A: 不需要！配置文件的修改会立即生效。只有修改 launch Python 文件时才需要重新编译。

## ✨ 下一步

1. ✅ 编译已完成
2. 📝 查看 README.md 了解详细信息
3. 🚀 运行启动命令测试系统
4. ⚙️ 根据需要调整配置文件
5. 🎯 享受便捷的系统启动！

---

**提示**: 如果遇到问题，请查看 README.md 中的调试部分，或查看启动时的日志输出。

**祝使用愉快！** 🎉
