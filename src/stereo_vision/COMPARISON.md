# Stereo Vision 包对比说明

## 概述

`stereo_vision` 目录下现在包含两个立体视觉处理包：

1. **stereo_processor**: 基于 OpenCV 自己封装的立体视觉处理库
2. **stereo_image_proc_wrapper**: 基于 ROS2 官方 `stereo_image_proc` 库的包装器

## 详细对比

### 1. stereo_processor

**实现方式**: 自定义 OpenCV 封装

**特点**:
- ✅ 完整的处理流程：包含图像校正、视差计算、点云生成
- ✅ 自包含：不依赖外部 ROS2 图像处理包
- ✅ 灵活性高：可以完全自定义实现细节
- ✅ 学习价值：了解立体视觉算法的底层实现
- ❌ 维护成本：需要自己维护和优化代码
- ❌ 性能：相比官方优化的实现可能较慢

**输入**:
- 原始左右相机图像 (`/camera/left/image_raw`, `/camera/right/image_raw`)
- 相机标定信息 (`/camera/left/camera_info`, `/camera/right/camera_info`)

**输出**:
- 校正后的图像 (`/camera/left/image_rect`, `/camera/right/image_rect`)
- 视差图 (`/stereo/disparity`)
- 点云 (`/stereo/points2`)

**适用场景**:
- 需要深度定制立体视觉算法
- 学习和研究立体视觉原理
- 不想依赖外部包的场景

---

### 2. stereo_image_proc_wrapper

**实现方式**: ROS2 官方库包装

**特点**:
- ✅ 官方维护：使用 ROS2 官方的 `stereo_image_proc`
- ✅ 性能优化：使用组合节点容器，支持零拷贝传输
- ✅ 稳定可靠：经过社区广泛测试和使用
- ✅ 易于集成：与 ROS2 生态系统无缝集成
- ✅ 功能丰富：可以方便地扩展使用整个 `image_pipeline`
- ❌ 需要预校正：输入必须是已校正的图像
- ❌ 依赖外部包：需要安装 `stereo_image_proc`

**输入**:
- **已校正**的左右相机图像 (`/camera/left/image_rect`, `/camera/right/image_rect`)
- 相机标定信息 (`/camera/left/camera_info`, `/camera/right/camera_info`)

**输出**:
- 视差图 (`/stereo/disparity`)
- 点云 (`/stereo/points2`)

**适用场景**:
- 生产环境部署
- 需要稳定可靠的立体视觉处理
- 想要使用 ROS2 生态系统的最佳实践
- 对性能有较高要求

---

## 使用建议

### 什么时候使用 stereo_processor？

1. **学习和研究**: 想要深入了解立体视觉算法的实现细节
2. **特殊需求**: 需要实现标准库不支持的特殊功能
3. **完整流程**: 需要从原始图像到点云的完整处理流程
4. **独立部署**: 不想依赖额外的 ROS2 包

### 什么时候使用 stereo_image_proc_wrapper？

1. **生产部署**: 需要稳定、经过验证的解决方案
2. **性能要求**: 需要高性能的立体视觉处理
3. **标准化**: 希望使用 ROS2 社区的标准实现
4. **快速开发**: 想要快速搭建立体视觉系统
5. **生态集成**: 需要与其他 ROS2 图像处理工具集成

## 工作流程对比

### stereo_processor 工作流程

```
相机驱动
  ↓
原始图像 + 标定信息
  ↓
stereo_processor (单个节点)
  ├─ 图像校正
  ├─ 视差计算
  └─ 点云生成
  ↓
输出：校正图像 + 视差图 + 点云
```

### stereo_image_proc_wrapper 工作流程

```
相机驱动
  ↓
原始图像 + 标定信息
  ↓
image_proc (图像校正)
  ↓
校正后的图像
  ↓
stereo_image_proc_wrapper (组合节点)
  ├─ DisparityNode (视差计算)
  └─ PointCloudNode (点云生成)
  ↓
输出：视差图 + 点云
```

## 性能对比

| 指标 | stereo_processor | stereo_image_proc_wrapper |
|------|------------------|---------------------------|
| 处理速度 | 中等 | 快 (使用零拷贝) |
| CPU 使用率 | 较高 | 较低 (优化的实现) |
| 内存使用 | 较高 (多次数据拷贝) | 较低 (零拷贝传输) |
| 启动时间 | 快 | 快 |

## 配置灵活性对比

| 功能 | stereo_processor | stereo_image_proc_wrapper |
|------|------------------|---------------------------|
| 立体匹配参数 | ✅ | ✅ |
| 图像校正参数 | ✅ | ⚠️ (需要单独配置 image_proc) |
| 自定义算法 | ✅ 容易 | ❌ 需要修改源码 |
| 预设配置 | ✅ | ✅ (标准/高质量/快速) |

## 迁移指南

### 从 stereo_processor 迁移到 stereo_image_proc_wrapper

如果你当前使用 `stereo_processor`，想要迁移到 `stereo_image_proc_wrapper`：

1. **安装依赖**:
   ```bash
   sudo apt install ros-humble-stereo-image-proc ros-humble-image-proc
   ```

2. **修改 Launch 文件**:
   - 移除 `stereo_processor` 的启动
   - 添加 `image_proc` 节点来校正图像
   - 添加 `stereo_image_proc_wrapper` 的启动

3. **调整 Topic 映射**:
   - 确保 `image_proc` 的输出连接到 `stereo_image_proc_wrapper` 的输入

4. **参数迁移**:
   - 大部分参数名称相同，可以直接复制
   - 注意某些参数的格式可能略有不同

### 示例 Launch 文件（完整流程）

```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='stereo_pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # 左相机校正
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='left_rectify',
                remappings=[
                    ('image', '/camera/left/image_raw'),
                    ('camera_info', '/camera/left/camera_info'),
                    ('image_rect', '/camera/left/image_rect'),
                ],
            ),
            # 右相机校正
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='right_rectify',
                remappings=[
                    ('image', '/camera/right/image_raw'),
                    ('camera_info', '/camera/right/camera_info'),
                    ('image_rect', '/camera/right/image_rect'),
                ],
            ),
            # 视差计算
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::DisparityNode',
                name='disparity_node',
                remappings=[
                    ('left/image_rect', '/camera/left/image_rect'),
                    ('right/image_rect', '/camera/right/image_rect'),
                    ('left/camera_info', '/camera/left/camera_info'),
                    ('right/camera_info', '/camera/right/camera_info'),
                ],
            ),
            # 点云生成
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::PointCloudNode',
                name='point_cloud_node',
                remappings=[
                    ('left/image_rect_color', '/camera/left/image_rect'),
                    ('left/camera_info', '/camera/left/camera_info'),
                ],
            ),
        ],
        output='screen',
    )
    
    return LaunchDescription([container])
```

## 结论

两个包各有优势：

- **stereo_processor**: 适合学习、研究和需要高度定制的场景
- **stereo_image_proc_wrapper**: 适合生产部署、追求性能和稳定性的场景

建议：
- 开发和学习阶段可以使用 `stereo_processor` 来理解原理
- 生产部署时使用 `stereo_image_proc_wrapper` 以获得更好的性能和稳定性
- 两个包可以共存，根据具体需求选择使用

## 相关链接

- [stereo_processor README](./stereo_processor/README.md)
- [stereo_image_proc_wrapper README](./stereo_image_proc_wrapper/README.md)
- [ROS2 stereo_image_proc 官方文档](http://wiki.ros.org/stereo_image_proc)
- [ROS2 image_pipeline GitHub](https://github.com/ros-perception/image_pipeline)
