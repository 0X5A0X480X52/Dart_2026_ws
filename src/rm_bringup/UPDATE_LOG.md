# RM Bringup 包 - 功能更新总结

## 🎉 新功能：Launch参数传递支持

**更新日期**: 2025-10-19
**版本**: v1.1

### ✨ 新增功能

现在 `rm_bringup` 包的配置文件支持为每个启动节点传递launch参数！

#### 主要改进：
1. ✅ **配置文件支持launch参数** - 可以在YAML配置文件中为每个节点指定参数
2. ✅ **自定义配置文件路径** - 轻松指定各个子系统的配置文件
3. ✅ **灵活的参数配置** - 支持各种参数类型（路径、模式、阈值等）
4. ✅ **参数日志输出** - 启动时显示传递的参数，便于调试

### 📝 更新的文件

#### 1. 主launch文件
- **文件**: `launch/system_bringup.launch.py`
- **更新**: 添加了 `create_include_launch()` 函数来处理launch参数传递
- **功能**: 从配置文件读取 `launch_arguments` 并传递给子launch文件

#### 2. 配置文件
- **文件**: `config/launch_config.yaml`
- **更新**: 添加了 `launch_arguments` 字段（注释形式，显示可用参数）

#### 3. 示例配置文件
- **文件**: `config/launch_config_example.yaml`
- **更新**: 添加详细的参数示例和说明

#### 4. 新增配置文件
- **文件**: `config/launch_config_with_params.yaml` ⭐新增
- **内容**: 带实际参数值的完整示例

#### 5. 文档更新
- **文件**: `README.md`
- **更新**: 添加了launch参数传递的说明
- **文件**: `LAUNCH_ARGUMENTS_GUIDE.md` ⭐新增
- **内容**: 完整的参数使用指南

### 🚀 使用示例

#### 基本用法（默认配置）
```bash
ros2 launch rm_bringup system_bringup.launch.py
```

#### 传递参数的配置文件示例
```yaml
stage1:
  camera:
    package: 'ros2_mindvision_camera'
    launch_file: 'dual_camera_launch.py'
    enabled: true
    launch_arguments:
      params_file: '/path/to/camera_config.yaml'
      use_sensor_data_qos: 'false'

stage2:
  object_detection:
    package: 'object_detection_openvino'
    launch_file: 'object_detection_openvino.launch.py'
    enabled: true
    launch_arguments:
      mode: 'armor'
      score_threshold: '0.7'

stage3:
  distance_estimator:
    package: 'stereo_distance_estimator'
    launch_file: 'stereo_distance_estimator_config.launch.py'
    enabled: true
    launch_arguments:
      config_file: '/path/to/estimator_config.yaml'
```

#### 使用带参数的配置启动
```bash
ros2 launch rm_bringup system_bringup.launch.py \
  config_file:=/path/to/launch_config_with_params.yaml
```

### 📋 支持的参数

#### Stage 1 - 相机驱动
- `params_file` - 相机参数配置文件路径
- `use_sensor_data_qos` - 是否使用SensorDataQoS

#### Stage 2 - 目标检测
- `params_file` - ROS2参数文件路径
- `mode` - 检测模式（'armor', 'rune'等）
- `input_width` - 输入图像宽度
- `input_height` - 输入图像高度
- `score_threshold` - 置信度阈值
- `nms_threshold` - NMS阈值

#### Stage 2 - 立体图像处理
- `config_file` - 配置文件路径
- `left_namespace` - 左相机命名空间
- `right_namespace` - 右相机命名空间

#### Stage 3 - 距离估计
- `config_file` - 配置文件路径

### 💡 使用场景

1. **测试环境** - 使用不同的配置文件进行测试
2. **比赛模式** - 切换到优化的比赛参数
3. **调试模式** - 使用调试参数便于问题排查
4. **多机器部署** - 每台机器使用自己的配置文件

### 📚 文档

- `README.md` - 完整使用说明
- `QUICKSTART.md` - 快速参考
- `LAUNCH_ARGUMENTS_GUIDE.md` ⭐新增 - 参数使用详细指南
- `DEMO.md` - 使用演示
- `SUMMARY.md` - 项目总结

### 🔍 参数验证

启动时会在日志中显示传递的参数：
```
[INFO] 为 object_detection_openvino/object_detection_openvino.launch.py 传递参数: 
       [('mode', 'armor'), ('score_threshold', '0.7')]
```

### ✅ 测试状态

- ✅ 编译成功
- ✅ 配置文件格式正确
- ✅ 参数传递逻辑实现
- ✅ 文档完整

### 📦 项目结构

```
src/rm_bringup/
├── CMakeLists.txt
├── package.xml
├── README.md                           # 更新
├── QUICKSTART.md
├── SUMMARY.md
├── DEMO.md
├── LAUNCH_ARGUMENTS_GUIDE.md          # ⭐新增
├── config/
│   ├── launch_config.yaml             # 更新
│   ├── launch_config_example.yaml     # 更新
│   └── launch_config_with_params.yaml # ⭐新增
├── docs/
│   └── system_architecture.dot
├── launch/
│   └── system_bringup.launch.py       # 更新
└── scripts/
    └── start_system.sh
```

### 🎯 后续建议

1. 根据实际需要创建自定义配置文件
2. 为不同场景准备多个配置（测试、比赛、调试）
3. 定期备份工作良好的配置文件
4. 记录各个参数的最佳值

### 🚀 立即开始

1. 查看示例配置：
```bash
cat src/rm_bringup/config/launch_config_with_params.yaml
```

2. 创建自定义配置：
```bash
cp src/rm_bringup/config/launch_config_with_params.yaml my_config.yaml
# 编辑 my_config.yaml
```

3. 使用自定义配置启动：
```bash
ros2 launch rm_bringup system_bringup.launch.py config_file:=my_config.yaml
```

4. 查看详细文档：
```bash
cat src/rm_bringup/LAUNCH_ARGUMENTS_GUIDE.md
```

---

**功能已完成并测试！** 🎉

如有问题，请查看 `LAUNCH_ARGUMENTS_GUIDE.md` 获取详细使用说明。
