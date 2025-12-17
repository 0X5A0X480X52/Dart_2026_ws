# 双目YOLO测距系统 - 快速启动指南

## 系统简介

本系统实现了基于双目YOLO检测的立体测距方案，替代传统的SGBM方法。通过在左右相机图像上分别运行YOLO检测，匹配检测结果，计算视差并估算3D距离。

## 快速启动

### 1. 编译系统

```bash
cd ~/Dart_2026_ws
colcon build --packages-select stereo_yolo_distance rm_bringup
source install/setup.bash
```

### 2. 配置相机标定参数

编辑标定文件（如果还没有标定，请先完成双目标定）：
```bash
# 查看现有标定参数
cat src/rm_hardware_driver/ros2_mindvision_camera/config/camera_info.yaml
```

**重要**：测量并记录基线距离（两个相机镜头中心的距离，单位：米）

### 3. 配置YOLO模型

编辑YOLO配置文件：
```bash
nano src/object_detection_openvino/config/params.yaml
```

确保设置正确的模型路径：
```yaml
object_detection_openvino_node:
  ros__parameters:
    xml_path: "/path/to/your/model.xml"
    bin_path: "/path/to/your/model.bin"
```

### 4. 配置立体测距参数

编辑测距配置：
```bash
nano src/stereo_yolo_distance/config/stereo_yolo_distance.yaml
```

**关键参数**：
```yaml
stereo_yolo_distance:
  ros__parameters:
    min_height_iou: 0.5          # 匹配阈值，可以降低到0.3-0.4
    baseline: 0.120              # ⚠️ 必须准确测量！
    use_manual_calibration: true # 建议先用手动模式测试
    fx: 1550.59436               # 从camera_info.yaml获取
```

### 5. 启动系统

```bash
cd ~/Dart_2026_ws
source install/setup.bash
ros2 launch rm_bringup stereo_yolo_system.launch.py
```

## 系统验证

### 检查节点状态

打开新终端：
```bash
source ~/Dart_2026_ws/install/setup.bash

# 查看所有运行的节点
ros2 node list

# 应该看到：
# /camera_left/mv_camera
# /camera_right/mv_camera
# /detector_left/object_detection_left
# /detector_right/object_detection_right
# /stereo_yolo_distance
# /serial_driver
```

### 检查话题数据

```bash
# 检查相机图像
ros2 topic hz /camera/left/image_raw
ros2 topic hz /camera/right/image_raw

# 检查左右检测结果
ros2 topic echo /detector/left/target2d_array --once
ros2 topic echo /detector/right/target2d_array --once

# 检查3D输出（这是最重要的）
ros2 topic echo /stereo/target3d_array
```

### 查看实时日志

```bash
# 查看立体测距节点的详细信息
ros2 node info /stereo_yolo_distance

# 如果需要调试信息
ros2 run rqt_console rqt_console
```

## 常见问题排查

### 问题1: 没有3D目标输出

**症状**：`/stereo/target3d_array` 话题没有数据

**排查步骤**：
1. 检查左右检测是否都有输出：
   ```bash
   ros2 topic echo /detector/left/target2d_array --once
   ros2 topic echo /detector/right/target2d_array --once
   ```

2. 检查节点日志：
   ```bash
   ros2 node info /stereo_yolo_distance
   ```
   查看是否有"Targets do not match"或"Invalid disparity"警告

3. 降低IOU阈值：
   ```yaml
   min_height_iou: 0.3  # 从0.5降低
   ```

4. 检查视差范围：
   ```yaml
   min_disparity: 5.0   # 降低最小值
   max_disparity: 500.0 # 增加最大值
   ```

### 问题2: 距离明显不准确

**可能原因**：
1. **基线距离错误**（最常见！）
   - 重新测量基线距离
   - 使用游标卡尺精确测量镜头中心距离

2. 焦距fx不正确
   - 检查camera_info.yaml中的projection_matrix[0,0]值

3. 相机未正确标定
   - 重新进行双目标定

### 问题3: 匹配失败

**症状**：日志中频繁出现"Targets do not match"

**解决方案**：
1. 降低IOU阈值到0.3或更低
2. 检查左右检测框的类别是否一致
3. 确保左右相机时间同步

### 问题4: 串口发送失败

**症状**：测距正常但下位机没有收到数据

**解决方案**：
```bash
# 检查串口设备
ls -l /dev/ttyUSB*

# 添加权限
sudo chmod 666 /dev/ttyUSB0

# 查看串口驱动日志
ros2 node info /serial_driver
```

## 参数调优建议

### 初始测试配置（宽松）

```yaml
stereo_yolo_distance:
  ros__parameters:
    min_height_iou: 0.3          # 宽松匹配
    max_distance: 15.0           # 大范围
    min_distance: 0.3            # 小范围
    max_disparity: 500.0         # 大视差
    min_disparity: 5.0           # 小视差
```

### 生产环境配置（严格）

```yaml
stereo_yolo_distance:
  ros__parameters:
    min_height_iou: 0.6          # 严格匹配
    max_distance: 8.0            # 实际需求范围
    min_distance: 0.5
    max_disparity: 300.0
    min_disparity: 15.0
```

## 性能监控

```bash
# 检查系统延迟
ros2 topic delay /stereo/target3d_array

# 检查CPU使用率
top -p $(pgrep -f stereo_yolo_distance_node)

# 检查发布频率
ros2 topic hz /stereo/target3d_array
```

## 下一步

1. **标定基线距离**：这是最关键的步骤！
2. **调整IOU阈值**：根据实际匹配效果
3. **优化YOLO参数**：在object_detection_openvino中调整置信度阈值
4. **集成滤波**：在串口驱动中启用卡尔曼滤波

## 技术支持

- 查看详细文档：`src/rm_bringup/docs/STEREO_YOLO_SYSTEM.md`
- 查看测距包文档：`src/stereo_yolo_distance/README.md`
- 查看YOLO检测文档：`src/object_detection_openvino/README_ROS2.md`

## 项目结构

```
Dart_2026_ws/src/
├── stereo_yolo_distance/        # 新建的立体测距包
│   ├── include/                 # 头文件
│   ├── src/                     # 源文件
│   ├── launch/                  # 启动文件
│   ├── config/                  # 配置文件
│   └── README.md
├── rm_bringup/
│   ├── launch/
│   │   └── stereo_yolo_system.launch.py  # 系统启动文件
│   └── docs/
│       └── STEREO_YOLO_SYSTEM.md         # 系统文档
├── object_detection_openvino/   # YOLO检测包
└── rm_hardware_driver/
    └── rm_serial_driver/        # 串口驱动
```

祝使用顺利！
