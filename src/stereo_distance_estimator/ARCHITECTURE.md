# Stereo Distance Estimator - 项目结构

```
stereo_distance_estimator/
├── CMakeLists.txt                          # CMake 构建配置
├── package.xml                              # ROS2 包清单
├── README.md                                # 完整文档
├── QUICKSTART.md                            # 快速开始指南
│
├── include/stereo_distance_estimator/      # C++ 头文件
│   └── stereo_distance_estimator_node.hpp  # 主节点类定义
│
├── src/                                     # C++ 源文件
│   ├── stereo_distance_estimator_node.cpp  # 主节点实现
│   └── main.cpp                             # 程序入口点
│
├── launch/                                  # Launch 文件
│   ├── stereo_distance_estimator.launch.py        # 基本 launch 文件（参数化）
│   └── stereo_distance_estimator_config.launch.py # 使用配置文件的 launch
│
├── config/                                  # 配置文件
│   └── stereo_distance_estimator.yaml      # 默认参数配置
│
└── scripts/                                 # Python 脚本
    └── test_publisher.py                    # 测试数据发布器
```

## 文件说明

### 核心代码

#### `include/stereo_distance_estimator/stereo_distance_estimator_node.hpp`
节点类的头文件定义，包含：
- 类声明和成员变量
- 订阅器、发布器和同步器
- 方法声明（回调函数、坐标转换等）
- 参数定义

#### `src/stereo_distance_estimator_node.cpp`
节点类的实现文件，包含：
- 构造函数：初始化参数、订阅器、发布器
- `syncCallback()`: 同步多个输入话题并处理数据
- `get3DPointFromCloud()`: 从点云中提取 3D 坐标
- `get3DPointFromDisparity()`: 从视差图计算 3D 坐标
- `calculateDistance()`: 计算欧氏距离
- Component 注册宏

#### `src/main.cpp`
可执行文件的入口点：
- 初始化 ROS2
- 创建并运行节点
- 清理资源

### 构建配置

#### `CMakeLists.txt`
定义编译过程：
- 查找依赖项（rclcpp, sensor_msgs, cv_bridge 等）
- 编译库和可执行文件
- 注册 component
- 安装目标（头文件、库、可执行文件、launch 文件等）

#### `package.xml`
ROS2 包清单：
- 包元数据（名称、版本、描述、维护者等）
- 构建和运行时依赖项
- 导出信息

### 启动文件

#### `launch/stereo_distance_estimator.launch.py`
参数化的 launch 文件：
- 声明所有可配置参数
- 创建节点并传递参数
- 适合命令行参数覆盖

#### `launch/stereo_distance_estimator_config.launch.py`
基于配置文件的 launch：
- 从 YAML 文件读取参数
- 更适合固定配置场景
- 支持自定义配置文件路径

### 配置文件

#### `config/stereo_distance_estimator.yaml`
默认参数配置：
- 话题名称
- 算法设置（使用点云/视差图、距离范围等）
- 相机内参（用于视差图模式）
- 便于版本控制和参数管理

### 测试脚本

#### `scripts/test_publisher.py`
Python 测试工具：
- 发布模拟的 2D 目标、视差图和点云数据
- 用于独立测试节点功能
- 无需真实相机即可验证算法

### 文档

#### `README.md`
完整文档，包含：
- 功能特性和系统架构
- 接口说明（话题、参数）
- 使用方法和示例
- 工作原理详解
- 故障排除

#### `QUICKSTART.md`
快速开始指南：
- 编译和安装步骤
- 快速测试流程
- 实际使用示例
- 常见问题解答

## 数据流

```
输入:
  /filter/target2d_array (Target2DArray)
  /stereo/disparity (Image)
  /stereo/points2 (PointCloud2)
         ↓
  [Message Synchronizer]
         ↓
  [2D → 3D Conversion]
    (使用点云或视差图)
         ↓
  [Distance Calculation]
         ↓
  [Range Filtering]
         ↓
输出:
  /stereo/target3d_array_raw (Target3DArray)
```

## 关键技术

1. **消息同步**: 使用 `message_filters::Synchronizer` 确保多个话题数据时间一致
2. **点云访问**: 使用 `sensor_msgs::PointCloud2Iterator` 高效访问有序点云
3. **视差反投影**: 使用针孔相机模型和视差-深度公式计算 3D 坐标
4. **Component 架构**: 支持作为共享库动态加载，便于组合和复用

## 扩展点

如果需要扩展功能，可以考虑以下位置：

1. **添加新的深度估计方法**: 在 `stereo_distance_estimator_node.cpp` 中添加新函数
2. **增强过滤逻辑**: 在 `syncCallback()` 中添加更复杂的过滤条件
3. **添加可视化**: 创建新的发布器发布 `visualization_msgs::MarkerArray`
4. **性能优化**: 使用并行处理或 GPU 加速
5. **添加追踪**: 集成卡尔曼滤波器或其他追踪算法

## 依赖关系

```
stereo_distance_estimator
  ├── rclcpp (ROS2 C++ 客户端库)
  ├── rclcpp_components (Component 支持)
  ├── sensor_msgs (图像和点云消息)
  ├── geometry_msgs (几何消息)
  ├── message_filters (消息同步)
  ├── cv_bridge (OpenCV-ROS 桥接)
  ├── OpenCV (图像处理)
  └── rm_interfaces (自定义消息类型)
```

## 编译流程

1. CMake 查找依赖项
2. 编译 `stereo_distance_estimator_node.cpp` 为共享库
3. 编译 `main.cpp` 并链接共享库
4. 注册 component
5. 安装所有文件到 install 目录

## 运行时流程

1. 加载节点（standalone 或 component）
2. 读取参数（从 launch 文件或配置文件）
3. 创建订阅器和发布器
4. 设置消息同步器
5. 进入 spin 循环，等待消息
6. 收到同步消息时触发回调
7. 处理数据并发布结果
