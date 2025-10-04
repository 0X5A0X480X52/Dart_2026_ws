# Stereo Vision 目录结构

```
stereo_vision/
├── README.md                        # 本文件：总体说明
├── COMPARISON.md                    # stereo_processor vs stereo_image_proc_wrapper 对比
│
├── stereo_processor/                # 基于 OpenCV 自定义实现的立体视觉包
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── README.md
│   ├── config/
│   │   └── stereo_processor.yaml   # 参数配置
│   ├── include/
│   │   └── stereo_processor/
│   ├── src/
│   ├── launch/
│   └── scripts/
│
├── stereo_rectifier/                # 图像校正包（集成相机驱动）✨新增
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── README.md                    # 详细使用说明
│   ├── QUICKSTART.md                # 快速入门指南
│   ├── config/
│   │   ├── left_camera_info.yaml   # 左相机标定参数模板
│   │   ├── right_camera_info.yaml  # 右相机标定参数模板
│   │   └── stereo_calibration.yaml # 立体标定参数示例
│   └── launch/
│       ├── stereo_rectify.launch.py           # 仅图像校正
│       ├── hik_stereo_rectify.launch.py       # 海康威视相机 + 图像校正
│       └── mindvision_stereo_rectify.launch.py # 迈德威视相机 + 图像校正
│
└── stereo_image_proc_wrapper/       # 基于 ROS2 官方库的立体视觉包
    ├── CMakeLists.txt
    ├── package.xml
    ├── README.md                    # 详细使用说明
    ├── QUICKSTART.md                # 快速入门指南
    ├── config/
    │   ├── stereo_params.yaml       # 默认配置（平衡模式）
    │   ├── high_quality.yaml        # 高质量配置
    │   └── fast_mode.yaml           # 快速处理配置
    └── launch/
        ├── stereo_image_proc.launch.py      # 核心立体视觉处理
        └── stereo_pipeline.launch.py        # 完整管道（可扩展）
```

## 包概述

### 1. stereo_rectifier ✨ 新增

**功能**: 集成相机驱动和图像校正

**特点**:
- 支持海康威视（HIK）和迈德威视（MindVision）相机
- 使用 ROS2 官方 `image_proc` 进行图像校正
- 消除镜头畸变和立体校正
- 提供校正后的图像供后续处理使用

**输入**: 原始图像 + 相机标定信息
**输出**: 校正后的图像

**使用场景**: 
- 作为立体视觉流程的第一步
- 与 `stereo_image_proc_wrapper` 配合使用
- 为任何需要无畸变图像的应用提供输入

### 2. stereo_processor

**功能**: 自定义 OpenCV 实现的完整立体视觉处理

**特点**:
- 包含图像校正、视差计算、点云生成全流程
- 基于 OpenCV 自己封装实现
- 自包含，不依赖外部 ROS2 图像处理包
- 高度可定制

**输入**: 原始图像 + 相机标定信息
**输出**: 校正图像 + 视差图 + 点云

**使用场景**:
- 学习立体视觉算法原理
- 需要自定义算法实现
- 研究和开发

### 3. stereo_image_proc_wrapper

**功能**: 基于 ROS2 官方库的立体视觉处理

**特点**:
- 使用 ROS2 官方维护的 `stereo_image_proc`
- 高性能，使用组合节点容器
- 稳定可靠，经过社区广泛测试
- 与 ROS2 生态系统无缝集成

**输入**: **已校正**的图像 + 相机标定信息
**输出**: 视差图 + 点云

**使用场景**:
- 生产环境部署
- 需要高性能和稳定性
- 标准 ROS2 应用

## 完整立体视觉流程

### 推荐流程（使用官方库）

```
相机驱动 (HIK/MindVision)
    ↓
stereo_rectifier (图像校正)
    ↓
stereo_image_proc_wrapper (视差计算 + 点云生成)
    ↓
点云输出
```

**启动命令**:

```bash
# 终端 1: 相机驱动 + 图像校正
ros2 launch stereo_rectifier hik_stereo_rectify.launch.py

# 终端 2: 立体视觉处理
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py
```

### 备选流程（使用自定义实现）

```
相机驱动
    ↓
stereo_processor (图像校正 + 视差计算 + 点云生成)
    ↓
点云输出
```

**启动命令**:

```bash
ros2 launch stereo_processor stereo_processor.launch.py
```

## 快速开始

### 1. 编译所有包

```bash
cd ~/Dart_2026_ws
colcon build --packages-select stereo_rectifier stereo_processor stereo_image_proc_wrapper
source install/setup.bash
```

### 2. 准备相机标定

**重要**: 必须先进行相机标定！

```bash
# 立体标定
ros2 run camera_calibration cameracalibrator \
    --size 8x6 --square 0.025 --approximate 0.1 \
    --ros-args -r left:=/camera/left/image_raw \
                -r right:=/camera/right/image_raw
```

将标定文件保存到：
- `stereo_rectifier/config/left_camera_info.yaml`
- `stereo_rectifier/config/right_camera_info.yaml`

### 3. 选择方案并启动

#### 方案 A: 官方库流程（推荐）

```bash
# 启动相机和图像校正
ros2 launch stereo_rectifier hik_stereo_rectify.launch.py

# 在另一个终端启动立体处理
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py
```

#### 方案 B: 自定义实现流程

```bash
ros2 launch stereo_processor stereo_processor.launch.py
```

### 4. 可视化

```bash
# 查看点云
rviz2

# 查看视差图
ros2 run rqt_image_view rqt_image_view /stereo/disparity
```

## 包选择指南

| 需求 | 推荐包 | 原因 |
|------|--------|------|
| 相机驱动集成 | stereo_rectifier | 直接支持 HIK 和 MindVision |
| 图像校正 | stereo_rectifier | 使用官方 image_proc |
| 学习算法原理 | stereo_processor | 自定义实现，代码清晰 |
| 生产部署 | stereo_image_proc_wrapper | 高性能，稳定可靠 |
| 快速原型 | stereo_image_proc_wrapper | 易用，标准接口 |
| 自定义算法 | stereo_processor | 完全可控 |
| 完整流程 | stereo_rectifier + stereo_image_proc_wrapper | 模块化，各司其职 |

## 性能对比

| 指标 | stereo_processor | stereo_rectifier + stereo_image_proc_wrapper |
|------|------------------|---------------------------------------------|
| 处理速度 | 中等 | 快（零拷贝） |
| CPU 使用 | 较高 | 较低 |
| 内存使用 | 较高 | 较低 |
| 可定制性 | 高 | 中 |
| 维护成本 | 高（自行维护） | 低（官方维护） |
| 学习曲线 | 陡（需要了解算法） | 平缓（标准接口） |

## 文件说明

### stereo_rectifier 包文件
   - 完整管道的启动文件模板
   - 可以扩展添加相机驱动等
   - 目前包含对 stereo_image_proc.launch.py 的引用

#### 文档文件

1. **README.md**: 完整的使用文档
   - 功能特性介绍
   - 详细的参数说明
   - 性能优化建议
   - 故障排除指南

2. **QUICKSTART.md**: 快速入门指南
   - 安装步骤
   - 基本使用示例
   - 完整流程示例（包含图像校正）
   - 可视化方法
   - 与相机集成示例

3. **COMPARISON.md** (在上级目录): 对比文档
   - stereo_processor vs stereo_image_proc_wrapper
   - 详细的功能对比表
   - 使用场景建议
   - 迁移指南

### 技术特点

1. **组合节点 (Composable Nodes)**:
   - 使用 `ComposableNodeContainer`
   - 启用进程内通信（零拷贝）
   - 降低延迟和 CPU 使用率

2. **话题映射**:
   - 输入: `/camera/left/image_rect`, `/camera/right/image_rect`
   - 输出: `/stereo/disparity`, `/stereo/points2`
   - 可通过 launch 文件自定义

3. **参数管理**:
   - 使用 YAML 配置文件
   - 支持运行时参数调整
   - 提供多种预设配置

### 依赖包

- `stereo_image_proc`: ROS2 官方立体视觉处理包
- `image_proc`: ROS2 官方图像处理包
- `rclcpp_components`: 组合节点支持
- `sensor_msgs`, `stereo_msgs`: 消息类型

## 使用建议

### 选择合适的包

- **学习/研究**: 使用 `stereo_processor` 了解算法实现细节
- **生产部署**: 使用 `stereo_image_proc_wrapper` 获得更好的性能和稳定性
- **快速原型**: 使用 `stereo_image_proc_wrapper` 快速搭建系统
- **特殊需求**: 使用 `stereo_processor` 进行自定义修改

### 性能考虑

`stereo_image_proc_wrapper` 的优势：
- ✅ 使用零拷贝传输减少内存开销
- ✅ 官方优化的算法实现
- ✅ 组合节点减少通信开销
- ✅ 社区持续维护和改进

### 集成建议

如果你的系统需要：
1. **原始图像 → 点云**: 需要添加 `image_proc` 节点进行图像校正
2. **已校正图像 → 点云**: 直接使用 `stereo_image_proc_wrapper`
3. **自定义处理**: 可以继承和扩展这些节点

## 编译与安装

```bash
# 进入工作空间
cd /home/amatrix/Dart_2026_ws

# 安装依赖
sudo apt install ros-humble-stereo-image-proc ros-humble-image-proc

# 编译新包
colcon build --packages-select stereo_image_proc_wrapper

# 加载环境
source install/setup.bash
```

## 测试

### 基本功能测试

```bash
# 1. 启动处理节点
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py

# 2. 在另一个终端检查节点状态
ros2 node list
ros2 topic list

# 3. 如果有相机或测试数据，检查输出
ros2 topic hz /stereo/disparity
ros2 topic echo /stereo/points2 --no-arr
```

### 可视化测试

```bash
# 启动 RViz2
rviz2

# 添加 PointCloud2 显示
# Topic: /stereo/points2
```

## 下一步计划

可以考虑的扩展：
1. 添加相机驱动集成示例
2. 添加标定工具集成
3. 添加性能基准测试
4. 添加单元测试
5. 创建示例数据包

## 相关链接

- [stereo_processor 包](./stereo_processor/)
- [stereo_image_proc_wrapper 包](./stereo_image_proc_wrapper/)
- [ROS2 Image Pipeline](https://github.com/ros-perception/image_pipeline)
- [OpenCV Stereo Vision Tutorial](https://docs.opencv.org/master/dd/d53/tutorial_py_depthmap.html)
