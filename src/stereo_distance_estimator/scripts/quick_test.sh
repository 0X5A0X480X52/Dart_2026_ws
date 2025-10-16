#!/bin/bash
# 快速测试脚本 - 使用 Launch 文件启动完整系统

cd /home/amatrix/Dart_2026_ws
source install/setup.bash

echo "=========================================="
echo "  启动完整立体视觉检测系统"
echo "=========================================="
echo ""
echo "包含以下组件："
echo "  1. 双目相机 (MindVision)"
echo "  2. 立体视觉处理"
echo "  3. 目标检测 (OpenVINO)"
echo "  4. 立体距离估计"
echo ""
echo "按 Ctrl+C 停止所有节点"
echo ""
echo "=========================================="

ros2 launch stereo_distance_estimator full_system_test.launch.py
