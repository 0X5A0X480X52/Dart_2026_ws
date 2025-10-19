# 参数冲突问题修复说明

## 问题描述

在启动系统时遇到以下错误：

```bash
[ERROR] [stereo_distance_estimator_node-5]: process has died [pid 12358, exit code -6, cmd '/home/hfut-nuc/Dart_2026_ws/install/stereo_distance_estimator/lib/stereo_distance_estimator/stereo_distance_estimator_node --ros-args -r __node:=stereo_distance_estimator --params-file src/rm_bringup/config/launch_config_with_params.yaml'].

failed to initialize rcl: Couldn't parse params file: '--params-file src/rm_bringup/config/launch_config_with_params.yaml'
```

## 问题根因

当通过命令行传递参数时：

```bash
ros2 launch rm_bringup system_bringup.launch.py config_file:="src/rm_bringup/config/launch_config_with_params.yaml"
```

这个 `config_file` 参数会被 ROS 2 的 launch 系统全局传递给所有子启动文件。

`stereo_distance_estimator_config.launch.py` 也定义了一个名为 `config_file` 的参数，用于指定节点的参数配置文件。结果导致：

- 系统启动配置文件 `launch_config_with_params.yaml` 被错误地传递给了 `stereo_distance_estimator_node` 作为节点参数文件
- 但该文件的格式不符合 ROS 2 节点参数文件的要求（它是 launch 配置文件，不是节点参数文件）
- 导致节点初始化失败

## 解决方案

将 `system_bringup.launch.py` 中的参数名从 `config_file` 改为 `system_config_file`，避免与子启动文件的参数名冲突。

### 修改的文件

1. **`src/rm_bringup/launch/system_bringup.launch.py`**
   - 参数名: `config_file` → `system_config_file`
   - 参数描述更明确: "系统启动配置文件路径"

2. **`src/rm_bringup/README.md`**
   - 更新所有示例命令，使用新的参数名

3. **`src/rm_bringup/scripts/start_system.sh`**
   - 更新脚本以使用新的参数名

## 新的使用方法

### 1. 使用默认配置启动

```bash
cd ~/Dart_2026_ws
source install/setup.bash
ros2 launch rm_bringup system_bringup.launch.py
```

### 2. 使用自定义系统配置文件启动

```bash
ros2 launch rm_bringup system_bringup.launch.py system_config_file:="src/rm_bringup/config/launch_config_with_params.yaml"
```

注意：现在参数名是 `system_config_file` 而不是 `config_file`！

### 3. 使用启动脚本

```bash
# 使用默认配置
./src/rm_bringup/scripts/start_system.sh

# 使用自定义配置文件
./src/rm_bringup/scripts/start_system.sh -c src/rm_bringup/config/launch_config_with_params.yaml

# 自定义延迟时间
./src/rm_bringup/scripts/start_system.sh -2 8.0 -3 15.0
```

## 参数说明

### system_config_file (系统配置文件)

- **作用**: 指定系统启动的配置文件
- **格式**: YAML 文件，包含各个 stage 的启动配置
- **默认值**: `<rm_bringup_share>/config/launch_config.yaml`
- **示例文件**: `src/rm_bringup/config/launch_config_with_params.yaml`

### 子启动文件的 config_file 参数

各个子系统的 launch 文件仍然使用 `config_file` 参数，用于指定各自节点的参数文件：

- `stereo_distance_estimator_config.launch.py`: 距离估计器的参数文件
- `stereo_image_proc.launch.py`: 立体图像处理的参数文件
- 其他子系统的启动文件...

这些参数可以在系统配置文件的 `launch_arguments` 中指定，例如：

```yaml
stage3:
  distance_estimator:
    package: 'stereo_distance_estimator'
    launch_file: 'stereo_distance_estimator_config.launch.py'
    enabled: true
    launch_arguments:
      config_file: '/home/amatrix/Dart_2026_ws/src/stereo_distance_estimator/config/stereo_distance_estimator.yaml'
```

## 验证修复

重新构建并启动系统：

```bash
cd ~/Dart_2026_ws
colcon build --packages-select rm_bringup
source install/setup.bash
ros2 launch rm_bringup system_bringup.launch.py system_config_file:="src/rm_bringup/config/launch_config_with_params.yaml"
```

现在应该不会再出现参数文件解析错误了。

## 技术细节

### ROS 2 Launch 参数传递机制

ROS 2 的 launch 系统在处理参数时：

1. 命令行参数会被添加到全局命名空间
2. 子启动文件可以访问父启动文件的所有参数
3. 如果子启动文件定义了同名参数，命令行值会覆盖子启动文件的默认值
4. 这可能导致意外的参数传递

### 最佳实践

为避免类似问题：

1. **使用唯一的参数名**: 顶层启动文件使用描述性的、不太可能冲突的参数名
2. **参数命名规范**: 考虑使用前缀，如 `system_*`、`global_*` 等
3. **文档化参数**: 在文档中清楚说明各个参数的作用域和用途

## 日期

- 修复日期: 2025年10月19日
- 修复者: GitHub Copilot
