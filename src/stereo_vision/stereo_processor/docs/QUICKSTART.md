# Stereo Processor - 快速开始

## 🚀 5分钟快速入门

### 1. 编译项目 (1分钟)

```bash
cd ~/Dart_2026_ws
colcon build --packages-select stereo_processor --symlink-install
source install/setup.bash
```

### 2. 验证安装 (30秒)

```bash
# 运行测试脚本
./src/target_matcher/test_node.sh

# 或手动检查
ros2 pkg list | grep stereo_processor
ros2 pkg executables stereo_processor
```

### 3. 启动节点 (30秒)

```bash
# 方式1: 使用 launch 文件（推荐）
ros2 launch stereo_processor stereo_processor.launch.py

# 方式2: 直接运行
ros2 run stereo_processor stereo_processor_node

# 方式3: 启动完整系统
ros2 launch stereo_processor full_stereo_system.launch.py
```

### 4. 验证运行 (1分钟)

打开新终端：

```bash
# 检查节点
ros2 node list
# 应该看到: /stereo_processor

# 检查 topics
ros2 topic list | grep -E "(rect|disparity|points2)"
# 应该看到:
#   /camera/left/image_rect
#   /camera/right/image_rect
#   /stereo/disparity
#   /stereo/points2

# 检查节点信息
ros2 node info /stereo_processor
```

### 5. 可视化 (2分钟)

```bash
# 启动 RViz2
rviz2

# 在 RViz2 中:
# 1. 添加 PointCloud2 -> Topic: /stereo/points2
# 2. 添加 Image -> Topic: /camera/left/image_rect
# 3. 添加 Image -> Topic: /stereo/disparity/image
# 4. 设置 Fixed Frame: stereo_camera_frame
```

## 📊 验证输出

### 检查图像输出

```bash
# 查看校正后的左图
ros2 run rqt_image_view rqt_image_view /camera/left/image_rect

# 查看视差图
ros2 run rqt_image_view rqt_image_view /stereo/disparity/image
```

### 检查点云输出

```bash
# 查看点云消息
ros2 topic echo /stereo/points2 --once

# 查看发布频率
ros2 topic hz /stereo/points2
```

## ⚙️ 调整参数

### 实时调整

```bash
# 调整视差范围
ros2 param set /stereo_processor num_disparities 192

# 调整匹配块大小
ros2 param set /stereo_processor block_size 21

# 查看所有参数
ros2 param list /stereo_processor

# 查看当前参数值
ros2 param get /stereo_processor num_disparities
```

### 永久修改

编辑配置文件：
```bash
nano ~/Dart_2026_ws/src/target_matcher/config/stereo_processor.yaml
```

然后重启节点。

## 🔧 常见问题

### Q1: 没有输出怎么办？

**检查输入:**
```bash
# 确保相机驱动在运行
ros2 topic list | grep camera

# 检查是否有数据
ros2 topic hz /camera/left/image_raw
ros2 topic hz /camera/right/image_raw
```

### Q2: 视差图全黑/质量差？

**调整参数:**
```yaml
stereo_processor:
  ros__parameters:
    num_disparities: 128    # 尝试增加/减少
    block_size: 15          # 尝试调整 (奇数)
    uniqueness_ratio: 10    # 增大以过滤噪声
```

### Q3: 处理速度慢？

**优化建议:**
```yaml
stereo_processor:
  ros__parameters:
    num_disparities: 64     # 减小视差范围
    block_size: 9           # 减小匹配块
    use_color: false        # 不生成彩色点云
```

并考虑降低输入图像分辨率。

## 📁 重要文件位置

```
~/Dart_2026_ws/src/target_matcher/
├── config/stereo_processor.yaml        # 参数配置
├── launch/stereo_processor.launch.py   # 启动文件
├── README.md                            # 详细说明
├── MIGRATION_GUIDE.md                   # 使用指南
└── test_node.sh                         # 测试脚本
```

## 🔗 相关命令速查

```bash
# 编译
colcon build --packages-select stereo_processor

# 启动
ros2 launch stereo_processor stereo_processor.launch.py

# 查看节点
ros2 node list

# 查看 topics
ros2 topic list

# 查看参数
ros2 param list /stereo_processor

# 设置参数
ros2 param set /stereo_processor <param_name> <value>

# 查看帮助
ros2 run stereo_processor stereo_processor_node --ros-args --help
```

## 📖 进一步学习

- 详细使用指南: `MIGRATION_GUIDE.md`
- 实现细节: `IMPLEMENTATION_SUMMARY.md`
- 系统架构: `docs/system_architecture.dot`

## 🆘 需要帮助？

- 查看日志: 节点输出会显示详细的状态信息
- 运行测试: `./src/target_matcher/test_node.sh`
- 联系维护者: amatrix02 (3432900546@qq.com)

---

**提示**: 确保您的相机驱动正确发布 `camera_info` 消息，这对立体校正至关重要！
