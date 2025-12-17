# 双目YOLO测距系统实现总结

## 实现概述

成功实现了基于双目YOLO检测的立体测距系统，替代传统的SGBM方法。该系统通过在左右相机图像上分别运行YOLO检测，使用高度IOU匹配算法匹配检测结果，基于视差计算3D距离。

## 创建的文件清单

### 1. stereo_yolo_distance 包（新建）

**核心文件**：
- `package.xml` - ROS2包描述文件
- `CMakeLists.txt` - 编译配置文件
- `include/stereo_yolo_distance/stereo_yolo_distance_node.hpp` - 节点头文件
- `src/stereo_yolo_distance_node.cpp` - 节点实现（~330行）
- `src/main.cpp` - 主程序入口

**配置和启动**：
- `launch/stereo_yolo_distance.launch.py` - 单独启动文件
- `config/stereo_yolo_distance.yaml` - 参数配置文件
- `README.md` - 包使用文档

### 2. rm_bringup 启动集成

**新增文件**：
- `launch/stereo_yolo_system.launch.py` - 完整系统启动文件
- `docs/STEREO_YOLO_SYSTEM.md` - 系统配置文档

### 3. 工作区文档

- `STEREO_YOLO_QUICKSTART.md` - 快速启动指南

**总计**：11个新文件

## 核心功能实现

### 1. 双目检测匹配

**算法**：高度IOU（Intersection Over Union）匹配
```cpp
double calculateHeightIOU(
    float left_y, float left_height,
    float right_y, float right_height)
{
    // 计算垂直方向的重叠度
    float intersection_top = max(left_top, right_top);
    float intersection_bottom = min(left_bottom, right_bottom);
    float intersection_height = max(0, intersection_bottom - intersection_top);
    float union_height = left_height + right_height - intersection_height;
    return intersection_height / union_height;
}
```

**匹配条件**：
- 高度IOU ≥ min_height_iou（默认0.5）
- 类别名称相同
- 视差在有效范围内（min_disparity ~ max_disparity）

### 2. 视差计算

```cpp
double disparity = left_target.x - right_target.x;
```

基于左右检测框中心点的水平坐标差。

### 3. 深度估算

使用标准立体视觉公式：
```cpp
double depth = (fx * baseline) / disparity;
```

参数来源：
- `fx`: 从camera_info的projection_matrix[0,0]获取，或手动配置
- `baseline`: 双目基线距离，需要实际测量

### 4. 3D坐标转换

使用针孔相机模型：
```cpp
double x = (left_target.x - cx) * depth / fx;
double y = (left_target.y - cy) * depth / fx;
double z = depth;
```

输出为相机坐标系下的3D位置。

## 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                      双目YOLO测距系统                            │
└─────────────────────────────────────────────────────────────────┘

┌──────────────┐     ┌──────────────┐
│  左相机驱动  │     │  右相机驱动  │
│ (mindvision) │     │ (mindvision) │
└──────┬───────┘     └──────┬───────┘
       │                    │
       │ image_raw          │ image_raw
       │ camera_info        │ camera_info
       │                    │
       ▼                    ▼
┌──────────────┐     ┌──────────────┐
│  左YOLO检测  │     │  右YOLO检测  │
│  (openvino)  │     │  (openvino)  │
└──────┬───────┘     └──────┬───────┘
       │                    │
       │ target2d_array     │ target2d_array
       │                    │
       └────────┬───────────┘
                │
                ▼
      ┌─────────────────────┐
      │  立体YOLO测距节点   │
      │ stereo_yolo_distance│
      │                     │
      │ • ROI高度IOU匹配    │
      │ • 视差计算          │
      │ • 深度估算          │
      │ • 3D坐标转换        │
      └──────────┬──────────┘
                 │
                 │ target3d_array
                 │
                 ▼
        ┌────────────────┐
        │   串口驱动     │
        │ (serial_driver)│
        └────────┬───────┘
                 │
                 ▼
            【下位机】
```

## 话题连接图

```
/camera/left/image_raw  ──────────┐
                                  ▼
                          左YOLO检测节点
                                  │
                                  ▼
/detector/left/target2d_array ────┐
                                  │
                                  ├──► 立体YOLO测距
                                  │        │
/detector/right/target2d_array ───┤        │
                                  │        ▼
                          右YOLO检测节点   /stereo/target3d_array
                                  ▲              │
/camera/right/image_raw  ─────────┘              │
                                                 ▼
                                            串口驱动
                                                 │
                                                 ▼
/camera/left/camera_info  ────────┐          下位机
/camera/right/camera_info ────────┘
```

## 关键参数配置

### 1. 相机标定参数

从 `ros2_mindvision_camera/config/camera_info.yaml` 获取：
```yaml
camera_matrix:
  data: [1560.46759, 0, 627.94252,  # fx, 0, cx
         0, 1563.0218, 519.00352,   # 0, fy, cy
         0, 0, 1]

projection_matrix:
  data: [1550.59436, 0, 629.28898, 0,  # fx (用于深度计算)
         0, 1554.92236, 518.36577, 0,
         0, 0, 1, 0]
```

**关键值**：
- **fx = 1550.59436** (像素)
- **baseline = ?** (需要实际测量，单位：米)

### 2. 匹配参数

```yaml
min_height_iou: 0.5      # 高度IOU阈值（可调整到0.3-0.7）
max_distance: 10.0       # 最大有效距离（米）
min_distance: 0.5        # 最小有效距离（米）
max_disparity: 300.0     # 最大视差（像素）
min_disparity: 10.0      # 最小视差（像素）
```

### 3. YOLO检测参数

从 `object_detection_openvino/config/params.yaml` 配置：
```yaml
score_threshold: 0.5     # 检测置信度阈值
nms_threshold: 0.4       # NMS阈值
input_width: 640
input_height: 384
```

## 特性说明

### 优势

1. **对长焦友好**：不依赖图像纹理，适合长焦镜头
2. **计算高效**：仅在检测目标处计算深度，无需密集视差图
3. **抗干扰强**：基于语义检测，对光照变化不敏感
4. **易于调试**：匹配逻辑清晰，IOU阈值可调

### 限制

1. **单目标假设**：当前版本选择置信度最高的目标
2. **需要双侧检测**：左右相机都必须检测到目标
3. **基线测量精度**：深度精度直接依赖基线测量准确度
4. **检测依赖**：受YOLO检测性能影响

## 使用流程

### 1. 编译安装

```bash
cd ~/Dart_2026_ws
colcon build --packages-select stereo_yolo_distance
source install/setup.bash
```

### 2. 配置参数

- 测量并配置基线距离（**最重要！**）
- 验证相机标定参数
- 设置YOLO模型路径
- 调整匹配阈值

### 3. 启动系统

```bash
ros2 launch rm_bringup stereo_yolo_system.launch.py
```

### 4. 监控验证

```bash
# 查看3D输出
ros2 topic echo /stereo/target3d_array

# 查看匹配日志
ros2 node info /stereo_yolo_distance
```

## 调试建议

### 问题排查顺序

1. **检查相机输出**
   ```bash
   ros2 topic hz /camera/left/image_raw
   ros2 topic hz /camera/right/image_raw
   ```

2. **检查YOLO检测**
   ```bash
   ros2 topic echo /detector/left/target2d_array --once
   ros2 topic echo /detector/right/target2d_array --once
   ```

3. **检查匹配结果**
   - 查看节点日志中的IOU值
   - 检查视差是否合理
   - 验证距离范围

4. **检查3D输出**
   ```bash
   ros2 topic echo /stereo/target3d_array
   ```

### 参数调优策略

1. **匹配不成功**：降低 `min_height_iou` (0.5 → 0.3)
2. **距离不准**：重新测量 `baseline`
3. **范围不对**：调整 `max_distance/min_distance`
4. **视差异常**：调整 `max_disparity/min_disparity`

## 性能指标

### 计算复杂度

- **YOLO检测**：O(1) per frame (GPU加速)
- **匹配算法**：O(1) (单目标假设)
- **深度计算**：O(1)

**总延迟**：主要由YOLO推理时间决定（~20-50ms）

### 精度分析

深度误差：
```
ΔZ = (Z² / (f × B)) × Δd

其中：
Z: 目标距离
f: 焦距
B: 基线
Δd: 视差误差（通常1-2像素）
```

**示例**（f=1550px, B=0.12m, Z=5m）：
- Δd = 1px → ΔZ ≈ 0.13m (2.6%误差)
- Δd = 2px → ΔZ ≈ 0.27m (5.4%误差)

## 扩展方向

### 短期优化

1. 添加时间戳同步检查
2. 实现多帧平滑滤波
3. 支持多目标匹配（匈牙利算法）
4. 添加卡尔曼滤波

### 长期扩展

1. 集成IMU数据校正
2. 支持动态目标跟踪
3. 融合点云数据验证
4. 深度学习端到端优化

## 与SGBM方案对比

| 特性 | SGBM方案 | 双目YOLO方案 |
|------|----------|--------------|
| 计算量 | 高（密集匹配） | 低（稀疏计算） |
| 长焦适配 | 差（纹理依赖） | 好（语义匹配） |
| 抗干扰 | 中等 | 强 |
| 精度 | 高（亚像素） | 中等（像素级） |
| 适用场景 | 纹理丰富 | 目标明确 |
| 部署难度 | 低 | 中等（需训练YOLO） |

## 参考文档

- [stereo_yolo_distance/README.md](../src/stereo_yolo_distance/README.md) - 测距包文档
- [rm_bringup/docs/STEREO_YOLO_SYSTEM.md](../src/rm_bringup/docs/STEREO_YOLO_SYSTEM.md) - 系统配置
- [STEREO_YOLO_QUICKSTART.md](../STEREO_YOLO_QUICKSTART.md) - 快速启动

## 贡献者

- 实现：amatrix02
- 日期：2025-12-17
- 版本：v1.0

## 许可证

Apache License 2.0
