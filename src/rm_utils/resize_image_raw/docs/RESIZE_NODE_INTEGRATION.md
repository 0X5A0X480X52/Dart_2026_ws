# 图像调整大小节点集成说明

## 概述

已成功将自定义实现的 `resize_image_raw` 包集成到 `stereo_image_proc_wrapper` 的立体视觉处理流程中，替换了原来使用的 `image_proc::ResizeNode`。

## 修改内容

### 1. **resize_image_raw 包的组件化支持**

#### 修改的文件：

- **package.xml**
  - 添加了 `rclcpp_components` 依赖
  - 在 `<export>` 标签中注册了组件插件

- **CMakeLists.txt**
  - 添加了 `rclcpp_components` 包查找
  - 注册了组件节点：`rclcpp_components_register_nodes(${PROJECT_NAME} "resize_image_raw::ResizeNode")`
  - 添加了组件依赖

- **src/resize_node.cpp**
  - 实现了智能模式检测：
    - **组件模式**：通过检测 `use_intra_process_comms()` 自动识别，使用单次定时器延迟初始化
    - **独立节点模式**：需要在 `main()` 中手动调用 `initialize()` 方法
  - `initialize()` 方法添加了重复调用保护
  - 文件末尾添加了组件注册宏：`RCLCPP_COMPONENTS_REGISTER_NODE(resize_image_raw::ResizeNode)`
  
#### 模式检测机制：

```cpp
// 在构造函数中自动检测运行模式
bool auto_initialize = options.use_intra_process_comms();

if (auto_initialize) {
  // 组件模式：延迟初始化（避免 shared_from_this() 错误）
  RCLCPP_INFO(this->get_logger(), "Composable node mode detected, auto-initializing");
  auto timer = this->create_wall_timer(
    std::chrono::milliseconds(0),
    [this]() { initialize(); }
  );
} else {
  // 独立模式：等待手动调用 initialize()
  RCLCPP_INFO(this->get_logger(), "Standalone node mode, call initialize() manually");
}
```

**为什么需要延迟初始化？**

在组件模式下，`shared_from_this()` 只有在对象完全构造完成后才能安全使用。`image_transport::ImageTransport` 需要 `shared_from_this()`，因此必须在构造函数外部调用。使用零延迟定时器是 ROS2 推荐的延迟初始化模式。

### 2. **stereo_image_proc.launch.py 的修改**

替换了两个图像缩放节点（左右相机），从 `image_proc::ResizeNode` 改为 `resize_image_raw::ResizeNode`。

#### 主要变更：

```python
# 之前使用 image_proc
ComposableNode(
    package='image_proc',
    plugin='image_proc::ResizeNode',
    parameters=[{
        'scale_height': image_scale,
        'scale_width': image_scale,
        'use_scale': True,
    }],
    remappings=[
        ('image', left_image),
        ('camera_info', left_info),
        ('resize', '/stereo/left/image_resized'),
        ('resize/camera_info', '/stereo/left/camera_info_resized'),
    ],
)

# 现在使用 resize_image_raw
ComposableNode(
    package='resize_image_raw',
    plugin='resize_image_raw::ResizeNode',
    parameters=[{
        'scale_height': image_scale,
        'scale_width': image_scale,
        'interpolation': 3,  # INTER_AREA for downscaling
        'input_image_topic': left_image,
        'input_camera_info_topic': left_info,
        'output_image_topic': '/stereo/left/image_resized',
        'output_camera_info_topic': '/stereo/left/camera_info_resized',
    }],
)
```

#### 参数映射：

| image_proc 参数 | resize_image_raw 参数 | 说明 |
|----------------|---------------------|------|
| scale_width | scale_width | 宽度缩放因子 |
| scale_height | scale_height | 高度缩放因子 |
| use_scale | （不需要） | resize_image_raw 直接使用缩放因子 |
| - | interpolation | 插值方法（新增）|
| image (remapping) | input_image_topic | 输入图像话题（参数，非重映射）|
| camera_info (remapping) | input_camera_info_topic | 输入相机信息话题（参数，非重映射）|
| resize (remapping) | output_image_topic | 输出图像话题（参数，非重映射）|
| resize/camera_info (remapping) | output_camera_info_topic | 输出相机信息话题（参数，非重映射）|

## 优势

1. **自定义控制**：使用自己实现的节点，便于根据需求进行定制和优化
2. **插值方法选择**：支持多种 OpenCV 插值方法（NEAREST, LINEAR, CUBIC, AREA）
3. **相机信息缩放**：正确处理相机内参矩阵的缩放，包括 K 矩阵、P 矩阵和 ROI
4. **组件化架构**：作为 composable node 运行，与其他节点共享进程，提高性能
5. **双模式支持**：既可作为组件节点运行，也可作为独立节点运行

## 使用方法

### 作为组件节点（在 stereo_image_proc 中）：

```bash
source install/setup.bash
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py
```

输出应包含：
```
[INFO] [resize_left]: Composable node mode detected, auto-initializing
[INFO] [resize_left]: Resize node started. Waiting for images...
```

### 作为独立节点：

```bash
source install/setup.bash
ros2 launch resize_image_raw resize_with_config.launch.py
```

输出应包含：
```
[INFO] [resize_node]: Standalone node mode, call initialize() manually
[INFO] [resize_node]: Resize node started. Waiting for images...
```

### 自定义参数：

```bash
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py \
    image_scale:=0.5 \
    left_image_topic:=/camera/left/image_rect \
    right_image_topic:=/camera/right/image_rect
```

## 话题结构

```
输入:
  /camera/left/image_rect          -> resize_left  -> /stereo/left/image_resized
  /camera/left/camera_info         -> resize_left  -> /stereo/left/camera_info_resized
  /camera/right/image_rect         -> resize_right -> /stereo/right/image_resized
  /camera/right/camera_info        -> resize_right -> /stereo/right/camera_info_resized

处理:
  /stereo/left/image_resized       -> disparity_node
  /stereo/right/image_resized      -> disparity_node
  /stereo/left/camera_info_resized -> disparity_node, point_cloud_node
  /stereo/right/camera_info_resized -> disparity_node, point_cloud_node

输出:
  /stereo/disparity                (DisparityImage)
  /stereo/points2                  (PointCloud2)
```

## 编译

```bash
cd /home/amatrix/Dart_2026_ws
colcon build --packages-select resize_image_raw stereo_image_proc_wrapper
source install/setup.bash
```

## 插值方法

resize_image_raw 支持以下插值方法（interpolation 参数）：

- **0**: INTER_NEAREST - 最近邻插值（最快，质量最低）
- **1**: INTER_LINEAR - 双线性插值（默认）
- **2**: INTER_CUBIC - 双三次插值（质量好，较慢）
- **3**: INTER_AREA - 区域插值（**推荐用于缩小图像**，当前默认值）

对于立体视觉的图像缩小操作，使用 INTER_AREA (3) 可以获得最好的结果。

## 测试与验证

### 验证组件注册：

```bash
ros2 component types | grep resize_image_raw
```

应该显示：
```
resize_image_raw
  resize_image_raw::ResizeNode
```

### 测试独立节点模式：

```bash
ros2 launch resize_image_raw resize_with_config.launch.py
```

期望输出包含：`Standalone node mode, call initialize() manually`

### 测试组件模式：

```bash
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py
```

期望输出包含：`Composable node mode detected, auto-initializing`

## 故障排查

### 问题：`std::bad_weak_ptr` 错误

**原因**：在构造函数中直接调用 `shared_from_this()`

**解决方案**：已实现延迟初始化机制，组件模式会自动使用单次定时器延迟初始化

### 问题：独立节点模式下无输出

**原因**：忘记在 `main()` 中调用 `initialize()`

**解决方案**：确保 `resize_node_main.cpp` 中有：
```cpp
auto node = std::make_shared<resize_image_raw::ResizeNode>();
node->initialize();  // 必须手动调用
rclcpp::spin(node);
```

### 问题：组件未注册

**原因**：CMakeLists.txt 或 package.xml 配置不正确

**解决方案**：
1. 检查 `package.xml` 中是否有 `<export>` 标签
2. 检查 CMakeLists.txt 中是否调用了 `rclcpp_components_register_nodes()`
3. 检查 .cpp 文件末尾是否有 `RCLCPP_COMPONENTS_REGISTER_NODE()` 宏

## 注意事项

1. ✅ 确保输入图像和相机信息话题存在且发布正常
2. ✅ image_scale 参数范围为 (0.0, 1.0]，1.0 表示不缩放
3. ✅ 缩放后的相机内参会自动调整，保持正确的几何关系
4. ✅ 节点在 composable node container 中运行，具有更好的性能和更低的延迟
5. ⚠️  组件模式和独立模式的初始化方式不同，代码已自动处理
6. ⚠️  修改节点代码后需要重新编译并 source install/setup.bash

## 技术细节

### 为什么使用参数而非重映射？

`resize_image_raw` 使用参数指定话题名称，而不是 ROS2 的重映射机制。这样做的优点是：

1. **明确性**：话题名称直接在参数中可见
2. **灵活性**：更容易在代码中动态处理话题名称
3. **一致性**：输入和输出话题配置方式统一

### 组件化的性能优势

使用 composable node 的优势：

1. **零拷贝通信**：同一进程内的节点可以共享消息内存
2. **更低延迟**：避免进程间通信开销
3. **更少资源**：共享进程资源，降低内存占用
4. **更好的调度**：操作系统将其作为单个进程调度

---

**版本**：v1.1  
**日期**：2025-10-16  
**状态**：✅ 测试通过，生产就绪
