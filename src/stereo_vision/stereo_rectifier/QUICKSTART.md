# Stereo Rectifier - 快速入门

## 快速开始

### 1. 编译包

```bash
cd ~/Dart_2026_ws
colcon build --packages-select stereo_rectifier
source install/setup.bash
```

### 2. 准备标定文件

**重要**: 必须先进行相机标定才能使用图像校正功能！

#### 方法一：使用现有标定文件

如果你已经有标定文件，将它们复制到配置目录：

```bash
cp /path/to/your/left_calibration.yaml ~/Dart_2026_ws/src/stereo_vision/stereo_rectifier/config/left_camera_info.yaml
cp /path/to/your/right_calibration.yaml ~/Dart_2026_ws/src/stereo_vision/stereo_rectifier/config/right_camera_info.yaml
```

#### 方法二：进行新的标定

**步骤 1**: 启动相机（以海康威视为例）

```bash
# 终端 1: 启动左相机
ros2 launch ros2_hik_camera hik_camera_launch.py

# 终端 2: 启动右相机（需要修改配置以使用不同序列号）
ros2 launch ros2_hik_camera hik_camera_launch.py
```

**步骤 2**: 运行标定工具

```bash
# 立体标定（推荐）
ros2 run camera_calibration cameracalibrator \
    --size 8x6 \
    --square 0.025 \
    --approximate 0.1 \
    --ros-args -r left:=/camera/left/image_raw \
                -r right:=/camera/right/image_raw \
                -r left_camera:=/camera/left \
                -r right_camera:=/camera/right
```

**步骤 3**: 标定操作

1. 将标定板移动到不同位置、角度和距离
2. 等待 X、Y、Size、Skew 进度条都变绿
3. 点击 "Calibrate" 按钮
4. 标定完成后点击 "Save"
5. 将保存的文件重命名并复制到 config 目录

### 3. 启动系统

#### 选项 A: 海康威视相机

```bash
ros2 launch stereo_rectifier hik_stereo_rectify.launch.py
```

#### 选项 B: 迈德威视相机

```bash
ros2 launch stereo_rectifier mindvision_stereo_rectify.launch.py
```

#### 选项 C: 仅图像校正（相机已运行）

```bash
ros2 launch stereo_rectifier stereo_rectify.launch.py
```

### 4. 查看结果

**查看原始图像**:
```bash
ros2 run rqt_image_view rqt_image_view /camera/left/image_raw
```

**查看校正图像**:
```bash
ros2 run rqt_image_view rqt_image_view /camera/left/image_rect
```

**检查话题**:
```bash
ros2 topic list | grep camera
```

应该看到：
```
/camera/left/image_raw
/camera/left/image_rect
/camera/left/image_rect_color
/camera/left/camera_info
/camera/right/image_raw
/camera/right/image_rect
/camera/right/image_rect_color
/camera/right/camera_info
```

## 常用命令

### 查看图像频率

```bash
ros2 topic hz /camera/left/image_rect
```

### 查看相机信息

```bash
ros2 topic echo /camera/left/camera_info
```

### 列出节点

```bash
ros2 node list
```

### 查看组合节点

```bash
ros2 component list
```

## 完整立体视觉流程

如果要使用完整的立体视觉系统（相机 → 校正 → 视差 → 点云）：

**终端 1**: 启动相机和图像校正
```bash
ros2 launch stereo_rectifier hik_stereo_rectify.launch.py
```

**终端 2**: 启动立体视觉处理
```bash
ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py
```

**终端 3**: 可视化点云
```bash
rviz2
```

在 RViz2 中：
1. 设置 Fixed Frame 为 `camera_left_optical_frame`
2. 添加 PointCloud2 显示
3. Topic 设置为 `/stereo/points2`

## 参数调整示例

### 指定相机序列号

```bash
ros2 launch stereo_rectifier hik_stereo_rectify.launch.py \
    left_camera_sn:=00J12345678 \
    right_camera_sn:=00J12345679
```

### 使用自定义标定文件

```bash
ros2 launch stereo_rectifier hik_stereo_rectify.launch.py \
    left_camera_info_url:=file:///home/user/my_left_calib.yaml \
    right_camera_info_url:=file:///home/user/my_right_calib.yaml
```

### 组合参数

```bash
ros2 launch stereo_rectifier hik_stereo_rectify.launch.py \
    left_camera_sn:=00J12345678 \
    right_camera_sn:=00J12345679 \
    left_camera_info_url:=file:///home/user/left_calib.yaml \
    right_camera_info_url:=file:///home/user/right_calib.yaml
```

## 故障排除速查

### 没有图像输出？

1. 检查相机是否连接：`lsusb` 或检查网络连接
2. 检查话题：`ros2 topic list | grep camera`
3. 查看原始图像：`ros2 run rqt_image_view rqt_image_view /camera/left/image_raw`

### 没有校正图像？

1. 检查标定文件：`ls ~/Dart_2026_ws/src/stereo_vision/stereo_rectifier/config/`
2. 查看节点日志输出
3. 确认 camera_info 发布：`ros2 topic echo /camera/left/camera_info`

### 相机无法打开？

1. 检查相机电源和连接
2. 确认没有其他程序使用相机
3. 检查相机驱动权限

### 图像有黑边？

这是正常的！畸变校正后边缘会有无效区域，可以在后续处理中裁剪。

## 下一步

- 阅读完整的 [README.md](README.md) 了解详细功能
- 查看 [stereo_image_proc_wrapper](../stereo_image_proc_wrapper/) 进行立体视觉处理
- 学习 [相机标定教程](https://navigation.ros.org/tutorials/docs/camera_calibration.html)

## 提示

- 标定质量直接影响校正效果，建议多次标定取最好的结果
- 标定时确保标定板覆盖整个视野的不同位置
- 立体标定比单独标定两个相机效果更好
- 定期重新标定以确保精度（特别是相机位置改变后）
