# Stereo Distance Estimator

ROS2 节点，用于将 2D 目标检测结果转换为 3D 空间坐标。支持使用点云或视差图进行深度估计。

## 功能特性

- **2D 到 3D 转换**：将图像平面的 2D 目标转换为相机坐标系的 3D 坐标
- **双模式支持**：
  - 点云模式：直接从点云数据中查询 3D 坐标（推荐）
  - 视差图模式：通过视差图和相机内参计算 3D 坐标
- **消息同步**：使用 `message_filters` 同步多个输入话题
- **距离过滤**：过滤超出有效范围的目标
- **Component 支持**：支持作为 ROS2 component 加载

## 订阅话题

| 话题名 | 类型 | 描述 |
|--------|------|------|
| `/filter/target2d_array` | `rm_interfaces/msg/Target2DArray` | 经过滤波的 2D 目标数组 |
| `/stereo/disparity` | `sensor_msgs/msg/Image` | 视差图（32FC1 格式） |
| `/stereo/points2` | `sensor_msgs/msg/PointCloud2` | 立体视觉生成的点云 |

## 发布话题

| 话题名 | 类型 | 描述 |
|--------|------|------|
| `/stereo/target3d_array_raw` | `rm_interfaces/msg/Target3DArray` | 未经滤波的 3D 目标数组 |

## 参数

### 话题配置

- `target2d_topic` (string, 默认: `/filter/target2d_array`)：输入 2D 目标话题
- `disparity_topic` (string, 默认: `/stereo/disparity`)：输入视差图话题
- `pointcloud_topic` (string, 默认: `/stereo/points2`)：输入点云话题
- `target3d_topic` (string, 默认: `/stereo/target3d_array_raw`)：输出 3D 目标话题

### 算法配置

- `use_pointcloud` (bool, 默认: `true`)：使用点云（true）或视差图（false）进行深度估计
- `max_distance` (double, 默认: `10.0`)：最大有效距离（米）
- `min_distance` (double, 默认: `0.1`)：最小有效距离（米）
- `queue_size` (int, 默认: `10`)：消息队列大小

### 相机内参（仅用于视差图模式）

- `fx` (double, 默认: `600.0`)：焦距 x（像素）
- `fy` (double, 默认: `600.0`)：焦距 y（像素）
- `cx` (double, 默认: `320.0`)：主点 x 坐标（像素）
- `cy` (double, 默认: `240.0`)：主点 y 坐标（像素）
- `baseline` (double, 默认: `0.12`)：立体相机基线距离（米）

### 图像缩放配置（重要！）

- `image_scale` (double, 默认: `0.5`)：点云/视差图相对于原始图像的缩放比例
  - ⚠️ **必须与 `stereo_image_proc_wrapper` 配置中的 `image_scale` 保持一致**
  - 如果立体视觉处理将图像缩小到 0.5 倍，此参数也应设为 0.5
  - 如果不缩放，设为 1.0
  - 此参数用于将 2D 目标坐标从原始图像尺寸映射到点云/视差图尺寸

## 编译

```bash
cd ~/Dart_2026_ws
colcon build --packages-select stereo_distance_estimator
source install/setup.bash
```

## 使用方法

### 1. 直接运行节点

```bash
ros2 run stereo_distance_estimator stereo_distance_estimator_node
```

### 2. 使用 launch 文件

```bash
ros2 launch stereo_distance_estimator stereo_distance_estimator.launch.py
```

### 3. 使用配置文件启动

```bash
ros2 launch stereo_distance_estimator stereo_distance_estimator_config.launch.py
```

### 4. 自定义参数启动

```bash
ros2 launch stereo_distance_estimator stereo_distance_estimator.launch.py \
  use_pointcloud:=false \
  max_distance:=15.0 \
  fx:=700.0 \
  fy:=700.0 \
  baseline:=0.15
```

### 5. 使用自定义配置文件

```bash
ros2 launch stereo_distance_estimator stereo_distance_estimator_config.launch.py \
  config_file:=/path/to/your/config.yaml
```

### 6. 作为 Component 加载

```bash
ros2 component standalone stereo_distance_estimator stereo_distance_estimator::StereoDistanceEstimatorNode
```

## 工作原理

### 点云模式（推荐）

1. 接收同步的 2D 目标、视差图和点云数据
2. 对每个 2D 目标，获取其像素坐标 (x, y)
3. 在点云中查找对应像素的 3D 点（row-major 存储）
4. 计算目标到相机原点的欧氏距离
5. 过滤无效或超出范围的目标
6. 发布 3D 目标数组

### 视差图模式

1. 接收同步的 2D 目标、视差图和点云数据
2. 对每个 2D 目标，从视差图中获取视差值 d
3. 使用公式计算深度：`Z = (fx × baseline) / d`
4. 使用针孔相机模型计算 3D 坐标：
   - `X = (u - cx) × Z / fx`
   - `Y = (v - cy) × Z / fy`
5. 过滤无效或超出范围的目标
6. 发布 3D 目标数组

## 坐标系

- **输入**：2D 目标使用图像像素坐标系（左上角为原点）
- **输出**：3D 目标使用相机坐标系（一般为 `camera_link` 或左相机坐标系）
  - X: 右
  - Y: 下
  - Z: 前（光轴方向）

## 注意事项

1. **点云 vs 视差图**：
   - 点云模式更简单可靠，推荐使用
   - 视差图模式需要准确的相机内参和基线参数

2. **消息同步**：
   - 使用 `ApproximateTime` 同步策略
   - 确保输入话题的时间戳相近
   - 调整 `queue_size` 以适应不同的发布频率

3. **有效性检查**：
   - 点云中的无效点（NaN、Inf）会被自动过滤
   - 超出距离范围的目标不会被发布

4. **性能优化**：
   - 点云访问使用了高效的 `PointCloud2Iterator`
   - 避免不必要的数据拷贝

## 数据流示例

```
[2D Targets] ──┐
               ├──> [Synchronizer] ──> [3D Conversion] ──> [Distance Filter] ──> [3D Targets]
[Disparity]  ──┤
               │
[PointCloud] ──┘
```

## 故障排除

### 没有输出

- 检查输入话题是否有数据：`ros2 topic echo <topic_name>`
- 检查消息时间戳是否同步
- 增加 `queue_size` 参数

### 3D 坐标不准确

- 验证相机内参是否正确（视差图模式）
- 检查点云数据质量
- 调整距离范围参数

### 性能问题

- 减少输入目标数量
- 降低话题发布频率
- 使用点云模式而非视差图模式

## 依赖项

- ROS2 (Humble/Iron)
- rclcpp
- rclcpp_components
- sensor_msgs
- geometry_msgs
- message_filters
- cv_bridge
- OpenCV
- rm_interfaces (自定义消息包)

## 许可证

Apache-2.0

## 作者

amatrix02
