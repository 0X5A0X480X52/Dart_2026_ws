# rm_serial_driver

## 软件包简介

该软件包包含serial_driver_node节点和virtual_driver_node节点，serial_driver_node节点用于串口数据发送和接收，virtual_driver_node节点用于测试串口数据发送和接收功能。

## 编译运行方法

**编译：**

```bash
cd ~/Dart_2026_ws
colcon build --packages-select rm_serial_driver
source install/setup.bash
```

**运行串口节点：**

```bash
sudo chmod 666 /dev/ttyACM0
ros2 launch rm_serial_driver serial_driver.launch.py
```

**或者运行虚拟串口：**

```bash
sudo chmod 666 /dev/ttyACM0
ros2 launch rm_serial_driver virtual_serial.launch.py
```

## 话题和服务

话题相关说明见`./config/serial_driver_params.yaml`文件

**服务：**

| 服务名称 | 服务类型 | 备注说明 |
|---|---|---|
| ~/set_parameters | rcl_interfaces::srv::SetParameters | 用于设置desired_pos_x_的服务 |
| ~/get_desired_pos_x | rm_interfaces::srv::GetDesiredPosX | 用于获取desired_pos_x_的初始值，该服务仅会在visualization_node开始工作时被调用一次，用于可视化界面初始数据的显示 |

## 串口通信相关细节

见`./docs/飞镖串口通信方法_V1.3.md`

## 需要注意的问题

串口线很不稳定，有时候出问题了，程序在初始化串口设备的阶段并不会有异常输出，请时刻检查接线
