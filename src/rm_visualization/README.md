# rm_visualization

## 简介

负责可视化的软件包，包含一个节点visualization_node，该节点由DartUI类维护，因此软件包的入口函数为`rm_visualization.ui_dart_main:main`，可参考`./setup.py`进行确认

## 参数说明

- 节点参数详见`./config/visualization_params.yaml`
- `./config/DartV1.0.ui`存储的是图形化界面的布局

## 使用方法

**编译软件包：**

```bash
cd ~/Dart_2026_ws
colcon build --packages-select rm_visualization
source install/setup.bash
```

**运行（启用虚拟串口节点时）：**

```bash
ros2 launch rm_visualization test_visualization.launch.py
```

**运行（不启用虚拟串口节点时）：**

```bash
ros2 launch rm_visualization visualization.launch.py
```

## 其它说明

请忽略`./test`文件夹，其中的内容由ros2自动生成，**与实际测试无关！**
