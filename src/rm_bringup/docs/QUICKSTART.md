# RM Bringup 快速参考

## 快速启动

### 方法1：使用ros2 launch
```bash
cd /home/amatrix/Dart_2026_ws
source install/setup.bash
ros2 launch rm_bringup system_bringup.launch.py
```

### 方法2：使用启动脚本
```bash
cd /home/amatrix/Dart_2026_ws
./src/rm_bringup/scripts/start_system.sh
```

## 常用命令

### 基本启动
```bash
# 默认配置
ros2 launch rm_bringup system_bringup.launch.py

# 自定义延迟
ros2 launch rm_bringup system_bringup.launch.py stage2_delay:=8.0 stage3_delay:=15.0

# 自定义配置文件
ros2 launch rm_bringup system_bringup.launch.py config_file:=/path/to/config.yaml
```

### 使用启动脚本
```bash
# 默认
./src/rm_bringup/scripts/start_system.sh

# 自定义延迟
./src/rm_bringup/scripts/start_system.sh -2 8.0 -3 15.0

# 自定义配置
./src/rm_bringup/scripts/start_system.sh -c my_config.yaml

# 组合使用
./src/rm_bringup/scripts/start_system.sh -c my_config.yaml -2 8.0 -3 15.0
```

## 启动顺序

```
Stage 1 (t=0s)
  └─ 双相机驱动
      ↓
      等待 5秒 (可配置)
      ↓
Stage 2 (t=5s)
  ├─ 目标检测 (并行)
  └─ 立体图像处理 (并行)
      ↓
      等待 5秒 (可配置)
      ↓
Stage 3 (t=10s)
  └─ 距离估计
```

## 配置文件位置

- 默认配置: `src/rm_bringup/config/launch_config.yaml`
- 示例配置: `src/rm_bringup/config/launch_config_example.yaml`

## 修改配置

1. 复制示例配置:
```bash
cp src/rm_bringup/config/launch_config.yaml my_config.yaml
```

2. 编辑配置文件:
```yaml
stage1:
  camera:
    enabled: true  # 改为 false 可禁用

stage2:
  object_detection:
    enabled: true
  stereo_image_proc:
    enabled: false  # 禁用立体图像处理

stage3:
  distance_estimator:
    enabled: true
```

3. 使用自定义配置启动:
```bash
ros2 launch rm_bringup system_bringup.launch.py config_file:=my_config.yaml
```

## 调试技巧

### 单独测试某个阶段
通过禁用其他阶段来单独测试:
```yaml
stage1:
  camera:
    enabled: true  # 只启动相机
stage2:
  object_detection:
    enabled: false
  stereo_image_proc:
    enabled: false
stage3:
  distance_estimator:
    enabled: false
```

### 调整启动延迟
如果节点启动太快导致问题:
```bash
ros2 launch rm_bringup system_bringup.launch.py stage2_delay:=10.0 stage3_delay:=20.0
```

### 查看日志
```bash
# 查看所有节点
ros2 node list

# 查看特定节点日志
ros2 run rqt_console rqt_console
```

## 停止系统

按 `Ctrl+C` 停止所有节点

## 重新编译

修改配置后不需要重新编译，直接启动即可。
修改launch文件后需要重新编译:

```bash
cd /home/amatrix/Dart_2026_ws
colcon build --packages-select rm_bringup
source install/setup.bash
```
