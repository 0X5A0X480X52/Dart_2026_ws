# 双目YOLO测距系统配置

## 系统架构说明

本配置实现了基于双目YOLO检测的立体测距系统，替代传统的SGBM方法。

### 数据流程

```
双目相机 → 左右YOLO检测 → ROI匹配 → 视差计算 → 3D坐标 → 串口发送
```

### 节点连接

1. **相机驱动** (`mindvision_camera` 或 `ros2_hik_camera`)
   - 输出: `/camera/left/image_raw`, `/camera/right/image_raw`
   - 输出: `/camera/left/camera_info`, `/camera/right/camera_info`

2. **左相机YOLO检测** (`object_detection_openvino_node`)
   - 输入: `/camera/left/image_raw`
   - 输出: `/detector/left/target2d_array`

3. **右相机YOLO检测** (`object_detection_openvino_node`)
   - 输入: `/camera/right/image_raw`
   - 输出: `/detector/right/target2d_array`

4. **立体YOLO测距** (`stereo_yolo_distance_node`)
   - 输入: `/detector/left/target2d_array`
   - 输入: `/detector/right/target2d_array`
   - 输入: `/camera/left/camera_info`
   - 输入: `/camera/right/camera_info`
   - 输出: `/stereo/target3d_array`

5. **串口驱动** (`rm_serial_driver_node`)
   - 输入: `/stereo/target3d_array`
   - 输出: 串口数据到下位机

## 启动系统

### 完整系统启动

```bash
cd ~/Dart_2026_ws
source install/setup.bash
ros2 launch rm_bringup stereo_yolo_system.launch.py
```

### 自定义启动参数

```bash
ros2 launch rm_bringup stereo_yolo_system.launch.py \
  camera_params:=/path/to/dual_camera_params.yaml \
  yolo_model_xml:=/path/to/model.xml \
  yolo_model_bin:=/path/to/model.bin \
  stereo_config:=/path/to/stereo_config.yaml
```

## 配置要求

### 1. 相机标定

必须先完成双目相机标定，标定文件位于：
- `src/rm_hardware_driver/ros2_mindvision_camera/config/camera_info.yaml`

关键参数：
- **camera_matrix**: 相机内参（包含焦距fx）
- **baseline**: 基线距离（需要实际测量）

### 2. YOLO模型

确保YOLO模型已正确配置：
- 模型路径在 `object_detection_openvino/config/params.yaml` 中设置
- 模型应该是OpenVINO格式（.xml + .bin）

### 3. 立体测距参数

编辑 `stereo_yolo_distance/config/stereo_yolo_distance.yaml`：

```yaml
stereo_yolo_distance:
  ros__parameters:
    # 匹配参数
    min_height_iou: 0.5          # 高度IOU阈值
    
    # 距离范围
    max_distance: 10.0           # 最大距离（米）
    min_distance: 0.5            # 最小距离（米）
    
    # 相机参数（从camera_info自动获取或手动设置）
    use_manual_calibration: false
    fx: 1550.59436               # 焦距
    baseline: 0.120              # 基线距离（米）- 需要实际测量！
```

## 测试和验证

### 1. 检查所有节点是否运行

```bash
ros2 node list
```

应该看到：
- `/camera_left/mv_camera` (或HIK相机节点)
- `/camera_right/mv_camera`
- `/detector_left/object_detection_left`
- `/detector_right/object_detection_right`
- `/stereo_yolo_distance`
- `/serial_driver`

### 2. 监控话题数据

```bash
# 查看相机图像
ros2 topic hz /camera/left/image_raw
ros2 topic hz /camera/right/image_raw

# 查看检测结果
ros2 topic echo /detector/left/target2d_array
ros2 topic echo /detector/right/target2d_array

# 查看3D目标
ros2 topic echo /stereo/target3d_array
```

### 3. 查看节点日志

```bash
# 查看立体测距节点日志
ros2 node info /stereo_yolo_distance

# 实时日志
ros2 run stereo_yolo_distance stereo_yolo_distance_node --ros-args --log-level debug
```

### 4. 可视化调试

```bash
# 查看检测框可视化（如果启用了debug_image）
ros2 run rqt_image_view rqt_image_view /detector/left/debug_image
ros2 run rqt_image_view rqt_image_view /detector/right/debug_image
```

## 参数调优

### 高度IOU阈值

如果匹配失败较多，可以降低IOU阈值：

```yaml
min_height_iou: 0.3  # 从0.5降低到0.3
```

### 视差范围

根据目标距离调整视差范围：

```yaml
max_disparity: 300.0  # 近距离目标
min_disparity: 10.0   # 远距离目标
```

### 基线距离测量

**重要**：精确测量双目相机的基线距离！

使用游标卡尺测量两个镜头光心之间的距离（单位：米）。

或从立体标定的右相机投影矩阵计算：
```
baseline = -projection_matrix[0,3] / fx
```

## 常见问题

### 1. 没有3D输出

- 检查左右相机是否都有检测输出
- 降低 `min_height_iou` 阈值
- 检查视差是否在有效范围内
- 查看节点日志了解详细信息

### 2. 距离不准确

- 验证相机标定是否正确
- 确认焦距fx参数准确
- **重新测量基线距离**（这是最常见的错误来源）
- 检查视差计算是否正确

### 3. 检测不稳定

- 检查YOLO模型的置信度阈值
- 优化光照条件
- 调整YOLO的score_threshold参数

### 4. 串口发送失败

- 检查串口设备权限：`sudo chmod 666 /dev/ttyUSB0`
- 验证串口配置文件
- 查看串口驱动日志

## 性能优化

### 1. 降低延迟

- 减小图像分辨率
- 优化YOLO模型（使用更轻量的模型）
- 使用GPU加速（设置device: "GPU"）

### 2. 提高准确度

- 使用更高分辨率的相机
- 增加基线距离（如果硬件允许）
- 使用更精确的标定方法

### 3. 增强鲁棒性

- 启用卡尔曼滤波（在串口驱动中）
- 添加时间戳同步检查
- 实现多帧平滑

## 与原系统的对比

### SGBM方法（原系统）
- ✅ 密集视差图，提供全局深度信息
- ❌ 对纹理依赖高，长焦效果差
- ❌ 计算量大
- ❌ 对光照敏感

### 双目YOLO方法（新系统）
- ✅ 仅在目标位置计算深度，效率高
- ✅ 对长焦相机友好
- ✅ 对纹理依赖低
- ✅ 可利用YOLO的语义信息
- ❌ 只能测量检测到的目标
- ❌ 需要左右都检测到目标

## 技术支持

如有问题，请检查：
1. 节点日志：`ros2 node info <node_name>`
2. 话题数据：`ros2 topic echo <topic_name>`
3. README文档：各包的README.md

联系：amatrix02 <3432900546@qq.com>
