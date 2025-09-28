# rm_interfaces

`rm_interfaces` 包含与双目视觉相关的 ROS 消息定义，主要用于双目相机系统中的数据传输和处理。

## 目录结构

```text
rm_interfaces/
├── CMakeLists.txt
├── package.xml
├── msg/                           # 消息定义
│   ├── Target2D.msg               # 图像平面目标检测信息
│   ├── Target2DArray.msg          # 多目标检测数组
│   ├── MultiViewTarget.msg          # 多视角目标检测信息
│   ├── MultiViewTargetArray.msg     # 多视角目标检测数组
│   ├── Target3D.msg               # 三维目标定位信息
│   └── Target3DArray.msg          # 多目标三维数组
├── include/                       # 消息头文件目录
│   └── rm_interfaces/
└── src/                           # 自动生成的消息源文件
```

### 消息文件说明

#### Target2D.msg

- `std_msgs/Header header`：ROS 消息头，包含时间戳和坐标系
- `float32 x`：目标中心在图像中的 x 像素坐标
- `float32 y`：目标中心在图像中的 y 像素坐标
- `float32 width`：目标框的宽度 (像素)
- `float32 height`：目标框的高度 (像素)
- `float32 confidence`：检测置信度
- `string class_name`：目标类别
- `int32 id`：目标唯一 ID（可选，跟踪使用）
- `bool is_filtered`：是否经过滤波（0=raw,1=filtered）

#### Target2DArray.msg

- `std_msgs/Header header`：ROS 消息头
- `Target2D[] targets`：多目标检测数组

#### MultiViewTarget.msg

- `std_msgs/Header header`：ROS 消息头
- `int32 id`：统一的目标 ID（同一目标在多视角下的标识）
- `Target2D[] views`：不同相机视角的检测结果
- `string class_name`：目标类别

#### MultiViewTargetArray.msg

- `std_msgs/Header header`：ROS 消息头
- `MultiViewTarget[] targets`：多目标的多视角信息

#### Target3D.msg

- `std_msgs/Header header`：ROS 消息头，包含时间戳和坐标系
- `geometry_msgs/Point position`：目标在相机坐标系下的三维位置 (m)
- `float32 distance`：估计距离 (m)
- `float32 confidence`：检测置信度
- `string class_name`：目标类别
- `int32 id`：目标唯一 ID（可选，用于跟踪）
- `bool is_filtered`：是否经过滤波（0=raw, 1=filtered）

#### Target3DArray.msg

- `std_msgs/Header header`：ROS 消息头
- `Target3D[] targets`：多目标三维数组
