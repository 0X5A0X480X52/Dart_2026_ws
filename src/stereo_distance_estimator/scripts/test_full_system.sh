#!/bin/bash
# 完整系统测试脚本
# 按顺序启动相机、立体视觉、目标检测和距离估计

echo "=========================================="
echo "    完整系统测试启动脚本"
echo "=========================================="
echo ""

# 切换到工作空间
cd /home/amatrix/Dart_2026_ws
source install/setup.bash

# 定义颜色
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}[Stage 1/4]${NC} 启动双目相机..."
gnome-terminal --tab --title="Cameras" -- bash -c "
    cd /home/amatrix/Dart_2026_ws
    source install/setup.bash
    ros2 launch mindvision_camera dual_camera_launch.py
    exec bash
"
sleep 3

echo -e "${GREEN}[Stage 2/4]${NC} 启动立体视觉处理..."
gnome-terminal --tab --title="Stereo Processing" -- bash -c "
    cd /home/amatrix/Dart_2026_ws
    source install/setup.bash
    ros2 launch stereo_image_proc_wrapper stereo_image_proc.launch.py
    exec bash
"
sleep 3

echo -e "${GREEN}[Stage 3/4]${NC} 启动目标检测..."
gnome-terminal --tab --title="Object Detection" -- bash -c "
    cd /home/amatrix/Dart_2026_ws
    source install/setup.bash
    ros2 launch object_detection_openvino object_detection_openvino.launch.py
    exec bash
"
sleep 3

echo -e "${GREEN}[Stage 4/4]${NC} 启动立体距离估计..."
gnome-terminal --tab --title="Distance Estimator" -- bash -c "
    cd /home/amatrix/Dart_2026_ws
    source install/setup.bash
    ros2 launch stereo_distance_estimator stereo_distance_estimator_config.launch.py
    exec bash
"

echo ""
echo -e "${BLUE}=========================================="
echo -e "    所有节点已启动!"
echo -e "==========================================${NC}"
echo ""
echo "使用以下命令监控话题："
echo "  ros2 topic list"
echo "  ros2 topic hz /stereo/target3d_array_raw"
echo "  ros2 topic echo /stereo/target3d_array_raw"
echo ""
echo "使用 Ctrl+C 停止各个终端中的节点"
