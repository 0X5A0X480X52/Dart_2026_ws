# 图像缩放坐标系修复说明

## 问题描述

在原始实现中，存在一个**关键的坐标系不匹配问题**：

### 数据流分析

1. **目标检测节点** (`object_detection_openvino`):
   - 订阅：`/camera_left/image_raw` (原始图像，例如 1280x720)
   - 发布：`/detector/target2d_array` (坐标基于原始图像尺寸)
   - ✅ 代码中已有 `fitRec()` 函数将检测结果转换回原始尺寸

2. **立体视觉处理** (`stereo_image_proc_wrapper`):
   - 订阅：`/camera_left/image_raw` 和 `/camera_right/image_raw` (原始图像)
   - 配置参数：`image_scale: 0.5` - **将图像缩小到 50%**
   - 发布：
     - `/stereo/disparity` (基于缩小后的图像，例如 640x360)
     - `/stereo/points2` (基于缩小后的图像)

3. **距离估计节点** (`stereo_distance_estimator`):
   - 订阅：
     - `/detector/target2d_array` (原始图像坐标，1280x720)
     - `/stereo/disparity` 和 `/stereo/points2` (缩小后的数据，640x360)
   - ❌ **问题**：直接用原始坐标访问缩小后的点云会导致：
     - 坐标越界
     - 访问错误的 3D 点
     - 距离估计完全不准确

### 具体示例

假设：

- 原始图像尺寸：1280x720
- 立体处理缩放：0.5 倍
- 点云/视差图尺寸：640x360

**问题场景**：

```
目标检测结果：目标在原始图像的 (640, 360) - 图像中心
点云尺寸：640x360
使用坐标 (640, 360) 访问点云 → 越界！(x >= width)
```

## 解决方案

### 修改内容

1. **添加 `image_scale` 参数**：
   - 在 `stereo_distance_estimator_node.hpp` 中添加 `image_scale_` 成员变量
   - 在配置文件中添加 `image_scale` 参数，默认 0.5

2. **坐标缩放**：

   ```cpp
   // 原始代码（错误）
   int pixel_x = static_cast<int>(target2d.x);  // 例如: 640
   int pixel_y = static_cast<int>(target2d.y);  // 例如: 360
   get3DPointFromCloud(cloud_msg, pixel_x, pixel_y, point_3d);  // 越界！
   
   // 修复后的代码
   int pixel_x = static_cast<int>(target2d.x);  // 640
   int pixel_y = static_cast<int>(target2d.y);  // 360
   int scaled_x = static_cast<int>(pixel_x * image_scale_);  // 640 * 0.5 = 320
   int scaled_y = static_cast<int>(pixel_y * image_scale_);  // 360 * 0.5 = 180
   get3DPointFromCloud(cloud_msg, scaled_x, scaled_y, point_3d);  // 正确！
   ```

3. **配置文件更新**：

   ```yaml
   stereo_distance_estimator:
     ros__parameters:
       # ... 其他参数 ...
       
       # 图像缩放参数
       # ⚠️ 必须与 stereo_image_proc_wrapper 的 image_scale 保持一致！
       image_scale: 0.5
   ```

### 使用说明

#### 重要配置原则

**`image_scale` 参数必须与立体视觉处理节点的缩放比例保持一致！**

检查立体视觉配置：

```bash
cat src/stereo_vision/stereo_image_proc_wrapper/config/stereo_params.yaml
```

查找：

```yaml
image_scale: 0.5  # 或其他值
```

然后在 `stereo_distance_estimator.yaml` 中设置相同的值：

```yaml
stereo_distance_estimator:
  ros__parameters:
    image_scale: 0.5  # 与立体视觉处理保持一致
```

#### 不同场景的配置

| 场景 | stereo_image_proc_wrapper | stereo_distance_estimator | 说明 |
|------|---------------------------|---------------------------|------|
| 缩小 50% | `image_scale: 0.5` | `image_scale: 0.5` | 推荐：平衡性能和精度 |
| 缩小 25% | `image_scale: 0.25` | `image_scale: 0.25` | 高性能模式 |
| 不缩放 | `image_scale: 1.0` | `image_scale: 1.0` | 高精度模式 |

## 验证修复

### 1. 编译

```bash
cd ~/Dart_2026_ws
colcon build --packages-select stereo_distance_estimator
source install/setup.bash
```

### 2. 运行测试

```bash
# 启动完整系统
ros2 launch stereo_distance_estimator full_system_test.launch.py

# 查看日志，确认没有越界错误
ros2 topic echo /stereo/target3d_array_raw
```

### 3. 检查坐标转换

在日志中应该能看到：

```
[stereo_distance_estimator]: Target at orig(640, 360) scaled(320, 180) has valid 3D point
```

而不是：

```
[stereo_distance_estimator]: Pixel coordinates (640, 360) out of cloud bounds (640 x 360)
```

## 性能影响

- ✅ **无性能损失**：只是简单的坐标乘法运算
- ✅ **提高准确性**：正确访问对应的 3D 点
- ✅ **避免崩溃**：防止数组越界访问

## 后续建议

1. **自动检测缩放比例**（可选优化）：
   - 从点云消息的 width/height 和相机参数自动计算 `image_scale`
   - 避免手动配置不一致的问题

2. **添加坐标验证**：
   - 在访问点云前检查缩放后的坐标是否在范围内
   - 提供更详细的错误信息

3. **配置验证工具**：
   - 创建 launch 文件参数验证
   - 确保两个节点的 `image_scale` 参数一致

## 相关文件

修改的文件：

- `include/stereo_distance_estimator/stereo_distance_estimator_node.hpp`
- `src/stereo_distance_estimator_node.cpp`
- `config/stereo_distance_estimator.yaml`
- `README.md`

参考配置：

- `src/stereo_vision/stereo_image_proc_wrapper/config/stereo_params.yaml`
- `src/object_detection_openvino/config/params.yaml`

## 总结

这个修复解决了一个关键的坐标系统不匹配问题，确保：

1. 2D 目标坐标正确映射到点云/视差图坐标系
2. 避免数组越界访问
3. 获得准确的 3D 位置估计

**关键点**：务必保持 `stereo_image_proc_wrapper` 和 `stereo_distance_estimator` 的 `image_scale` 参数一致！
