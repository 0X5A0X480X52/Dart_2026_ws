# RM Bringup 系统启动包 - 完成总结

## ✅ 已完成的工作

### 1. 创建的文件

#### Launch 文件
- `launch/system_bringup.launch.py` - 主启动文件，实现顺序启动

#### 配置文件
- `config/launch_config.yaml` - 默认配置文件
- `config/launch_config_example.yaml` - 配置示例文件

#### 脚本文件
- `scripts/start_system.sh` - 快速启动脚本

#### 文档文件
- `README.md` - 详细说明文档
- `QUICKSTART.md` - 快速参考指南
- `SUMMARY.md` - 本文件，完成总结

### 2. 包配置
- 更新了 `package.xml`，添加了必要的依赖
- 更新了 `CMakeLists.txt`，配置了文件安装

## 📋 启动顺序

系统按照以下顺序启动：

```
[Stage 1] t=0s
    └─ ros2_mindvision_camera/dual_camera_launch.py
       (双相机驱动)
           ↓
       等待 5 秒
           ↓
[Stage 2] t=5s (并行启动)
    ├─ object_detection_openvino/object_detection_openvino.launch.py
    │  (目标检测)
    │
    └─ stereo_image_proc_wrapper/stereo_image_proc.launch.py
       (立体图像处理)
           ↓
       等待 5 秒
           ↓
[Stage 3] t=10s
    └─ stereo_distance_estimator/stereo_distance_estimator_config.launch.py
       (距离估计)
```

## 🚀 使用方法

### 快速启动（推荐）

```bash
cd /home/amatrix/Dart_2026_ws
source install/setup.bash

# 方法1：使用启动脚本
./src/rm_bringup/scripts/start_system.sh

# 方法2：使用ros2 launch
ros2 launch rm_bringup system_bringup.launch.py
```

### 自定义配置

```bash
# 自定义延迟时间
ros2 launch rm_bringup system_bringup.launch.py stage2_delay:=8.0 stage3_delay:=15.0

# 使用自定义配置文件
ros2 launch rm_bringup system_bringup.launch.py config_file:=/path/to/config.yaml

# 使用脚本的自定义选项
./src/rm_bringup/scripts/start_system.sh -2 8.0 -3 15.0
./src/rm_bringup/scripts/start_system.sh -c my_config.yaml
```

## ⚙️ 配置文件说明

配置文件 `config/launch_config.yaml` 可以配置：

1. **启用/禁用节点**: 设置 `enabled: true/false`
2. **指定launch文件**: 修改 `package` 和 `launch_file`
3. **三个阶段的独立配置**: stage1, stage2, stage3

示例配置：
```yaml
stage1:
  camera:
    package: 'ros2_mindvision_camera'
    launch_file: 'dual_camera_launch.py'
    enabled: true

stage2:
  object_detection:
    package: 'object_detection_openvino'
    launch_file: 'object_detection_openvino.launch.py'
    enabled: true
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

## 🎯 特性

1. **顺序启动**: 按照预定义的顺序启动节点，确保依赖关系正确
2. **并行启动**: Stage 2的节点并行启动，提高效率
3. **可配置延迟**: 可以调整各阶段之间的启动延迟
4. **灵活配置**: 通过YAML配置文件可以启用/禁用节点和切换launch文件
5. **易于使用**: 提供了便捷的启动脚本

## 📝 参数说明

### Launch 参数

- `config_file`: 配置文件路径（默认：使用包内的 launch_config.yaml）
- `stage1_delay`: Stage 1启动延迟，默认 0.0 秒
- `stage2_delay`: Stage 2启动延迟，默认 5.0 秒
- `stage3_delay`: Stage 3启动延迟，默认 10.0 秒

### 启动脚本选项

- `-c, --config FILE`: 指定配置文件路径
- `-2, --stage2-delay SEC`: 设置Stage 2启动延迟
- `-3, --stage3-delay SEC`: 设置Stage 3启动延迟
- `-h, --help`: 显示帮助信息

## 🔧 调试建议

### 1. 测试单个阶段
修改配置文件，禁用其他阶段：
```yaml
stage1:
  camera:
    enabled: true
stage2:
  object_detection:
    enabled: false  # 禁用
  stereo_image_proc:
    enabled: false  # 禁用
stage3:
  distance_estimator:
    enabled: false  # 禁用
```

### 2. 调整延迟时间
如果节点启动失败，尝试增加延迟：
```bash
ros2 launch rm_bringup system_bringup.launch.py stage2_delay:=10.0 stage3_delay:=20.0
```

### 3. 查看日志
启动过程中会有日志输出，显示各阶段的启动状态

## 📦 依赖包

确保以下包已正确编译：
- `ros2_mindvision_camera` (相机驱动)
- `object_detection_openvino` (目标检测)
- `stereo_image_proc_wrapper` (立体图像处理)
- `stereo_distance_estimator` (距离估计)

## 🔄 重新编译

修改launch文件后需要重新编译：
```bash
cd /home/amatrix/Dart_2026_ws
colcon build --packages-select rm_bringup
source install/setup.bash
```

修改配置文件（*.yaml）后不需要重新编译，直接使用即可。

## 📚 文档参考

- `README.md` - 详细使用说明
- `QUICKSTART.md` - 快速参考
- `config/launch_config_example.yaml` - 配置示例

## ✨ 下一步

1. 根据实际需要调整 `config/launch_config.yaml`
2. 测试启动流程
3. 根据需要调整各阶段的延迟时间
4. 如有需要，可以创建多个配置文件用于不同场景

---

创建日期: 2025年10月19日
包版本: 0.0.0
