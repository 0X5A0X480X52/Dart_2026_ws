# Stereo YOLO Distance Estimator

基于双目YOLO检测的立体测距包，通过匹配左右相机的检测结果并计算视差来估算目标的3D位置。

## 功能特点

- **双目YOLO检测匹配**：订阅左右相机的YOLO检测结果，通过高度IOU进行匹配
- **视差计算**：基于左右检测框中心点的水平位置差计算视差
- **3D位置估算**：使用相机标定参数和视差计算目标的3D坐标
- **自动标定参数获取**：可从camera_info话题自动提取焦距和基线参数
- **单目标优化**：当前版本针对单目标场景优化，选择置信度最高的检测结果

## 依赖项

- ROS2 (Humble 或更高版本)
- rclcpp
- rm_interfaces
- sensor_msgs
- geometry_msgs
- cv_bridge
- OpenCV

## 编译

```bash
cd ~/Dart_2026_ws
colcon build --packages-select stereo_yolo_distance
source install/setup.bash
```

## 使用方法

### 1. 基本启动

```bash
ros2 launch stereo_yolo_distance stereo_yolo_distance.launch.py
```

### 2. 自定义参数启动

```bash
ros2 launch stereo_yolo_distance stereo_yolo_distance.launch.py \
  left_target_topic:=/custom/left/detections \
  right_target_topic:=/custom/right/detections \
  target3d_topic:=/custom/target3d
```

### 3. 使用自定义配置文件

```bash
ros2 launch stereo_yolo_distance stereo_yolo_distance.launch.py \
  config_file:=/path/to/your/config.yaml
```

## 配置参数

编辑 `config/stereo_yolo_distance.yaml` 以调整参数：

### 话题配置

- `left_target_topic`: 左相机检测结果话题
- `right_target_topic`: 右相机检测结果话题
- `left_camera_info_topic`: 左相机camera_info话题
- `right_camera_info_topic`: 右相机camera_info话题
- `target3d_topic`: 输出的3D目标话题

### 匹配参数

- `min_height_iou`: 最小高度IOU阈值（默认：0.5）
  - 控制左右检测框垂直方向的重叠度要求
  - 值越大，匹配越严格
- `debug_invalid_reason`: 输出无效检测框原因（默认：false）
  - true: 当存在检测框但无法生成3D目标时，输出具体原因（IOU、类别、视差、距离范围等）
  - 建议仅在排障时开启
- `empty_publish_hz`: 空3D消息周期发布频率（默认：10.0）
  - 当最近一轮匹配未产生有效3D目标时，按该频率发布空的 `/stereo/target3d_array`
  - 设置为 <= 0 可关闭该功能

### 距离和视差范围

- `max_distance`: 最大有效距离，单位：米（默认：10.0）
- `min_distance`: 最小有效距离，单位：米（默认：0.5）
- `max_disparity`: 最大视差，单位：像素（默认：300.0）
- `min_disparity`: 最小视差，单位：像素（默认：10.0）

### 相机标定参数

- `use_manual_calibration`: 是否使用手动标定参数（默认：false）
  - false: 从camera_info话题自动获取
  - true: 使用配置文件中的fx和baseline值

- `fx`: 焦距，单位：像素（默认：1550.59436）
- `baseline`: 基线距离，单位：米（默认：0.120）

**重要**：baseline是两个相机光心之间的实际物理距离，需要精确测量！

## 工作原理

### 1. 检测结果订阅

节点订阅左右相机的Target2DArray消息，每个消息包含：
- 检测框的中心坐标 (x, y)
- 检测框的宽高 (width, height)
- 置信度 (confidence)
- 类别 (class_name)

### 2. 目标匹配

使用高度IOU（Intersection Over Union）算法匹配左右检测结果：

```
height_IOU = intersection_height / union_height
```

匹配条件：
- 高度IOU ≥ min_height_iou
- 类别名称相同
- 视差在有效范围内

### 3. 视差计算

```
disparity = left_x - right_x
```

### 4. 深度计算

使用立体视觉的基本公式：

```
Z = (fx × baseline) / disparity
```

其中：
- Z: 目标深度（距离）
- fx: 相机焦距
- baseline: 双目基线距离
- disparity: 视差

### 5. 3D坐标计算

使用针孔相机模型：

```
X = (u - cx) × Z / fx
Y = (v - cy) × Z / fy
```

其中 (u, v) 是左相机检测框的中心坐标，(cx, cy) 是主点坐标。

## 消息格式

### 输入：Target2DArray

```
std_msgs/Header header
Target2D[] targets
```

### 输出：Target3DArray

```
std_msgs/Header header
Target3D[] targets
```

Target3D包含：
- position (geometry_msgs/Point): 3D坐标
- distance (float32): 估算距离
- confidence (float32): 置信度
- class_name (string): 类别名称

## 标定参数获取

### 方法1：自动获取（推荐）

设置 `use_manual_calibration: false`，节点将自动从camera_info话题提取参数：
- fx 从左相机的 projection_matrix[0,0] 获取
- baseline 从右相机的 projection_matrix[0,3] 计算

### 方法2：手动配置

1. 从标定文件获取fx：
   ```yaml
   # 在 camera_info.yaml 中查找 projection_matrix
   projection_matrix:
     data: [1550.59436, 0, 629.28898, 0, ...]
            ^^^^^^^^^
            这是fx的值
   ```

2. 测量基线距离：
   - 使用卷尺或游标卡尺测量两个相机镜头中心的距离
   - 或从立体标定的右相机projection_matrix计算：
     ```
     baseline = -projection_matrix[0,3] / fx
     ```

## 调试

### 查看节点日志

```bash
ros2 run stereo_yolo_distance stereo_yolo_distance_node --ros-args --log-level debug
```

### 监控话题

```bash
# 查看输入检测结果
ros2 topic echo /detector/left/target2d_array
ros2 topic echo /detector/right/target2d_array

# 查看输出3D目标
ros2 topic echo /stereo/target3d_array

# 检查发布频率
ros2 topic hz /stereo/target3d_array
```

### 可视化

使用RViz2可视化3D目标位置：

```bash
ros2 run rviz2 rviz2
```

添加MarkerArray或PointCloud2显示类型。

## 常见问题

### 1. 没有输出3D目标

- 检查左右相机是否都有检测结果
- 检查高度IOU是否过于严格，尝试降低 `min_height_iou`
- 检查视差范围是否合理
- 当前节点支持在无有效目标时周期发布空的 `/stereo/target3d_array`，可通过 `empty_publish_hz` 调整频率
- 排障时可将 `debug_invalid_reason` 设为 true，并使用 `--ros-args --log-level stereo_yolo_distance:=debug` 查看原因日志

### 2. 距离不准确

- 验证fx和baseline参数是否正确
- 确保相机已正确标定
- 检查视差计算是否合理（左x应大于右x）

### 3. 匹配失败

- 降低 `min_height_iou` 阈值
- 检查左右检测框的类别是否一致
- 查看DEBUG日志了解详细匹配信息

## 许可证

Apache License 2.0
