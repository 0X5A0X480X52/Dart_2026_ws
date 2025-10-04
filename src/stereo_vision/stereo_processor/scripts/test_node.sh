#!/bin/bash

# Stereo Processor 测试脚本
# 用于验证节点是否正常工作

echo "=========================================="
echo "Stereo Processor 节点测试"
echo "=========================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Source workspace
echo -e "${YELLOW}[1/6]${NC} Sourcing workspace..."
source /home/amatrix/Dart_2026_ws/install/setup.bash
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓${NC} Workspace sourced successfully"
else
    echo -e "${RED}✗${NC} Failed to source workspace"
    exit 1
fi
echo ""

# 检查包是否存在
echo -e "${YELLOW}[2/6]${NC} Checking if stereo_processor package exists..."
ros2 pkg list | grep -q "stereo_processor"
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓${NC} Package stereo_processor found"
else
    echo -e "${RED}✗${NC} Package stereo_processor not found"
    exit 1
fi
echo ""

# 检查可执行文件
echo -e "${YELLOW}[3/6]${NC} Checking executable..."
if [ -f "/home/amatrix/Dart_2026_ws/install/stereo_processor/lib/stereo_processor/stereo_processor_node" ] || \
   [ -L "/home/amatrix/Dart_2026_ws/install/stereo_processor/lib/stereo_processor/stereo_processor_node" ]; then
    echo -e "${GREEN}✓${NC} Executable stereo_processor_node found"
else
    echo -e "${RED}✗${NC} Executable stereo_processor_node not found"
    exit 1
fi
echo ""

# 检查配置文件
echo -e "${YELLOW}[4/6]${NC} Checking configuration files..."
if [ -f "/home/amatrix/Dart_2026_ws/install/stereo_processor/share/stereo_processor/config/stereo_processor.yaml" ] || \
   [ -L "/home/amatrix/Dart_2026_ws/install/stereo_processor/share/stereo_processor/config/stereo_processor.yaml" ]; then
    echo -e "${GREEN}✓${NC} Configuration file found"
else
    echo -e "${RED}✗${NC} Configuration file not found"
    exit 1
fi
echo ""

# 检查 launch 文件
echo -e "${YELLOW}[5/6]${NC} Checking launch files..."
if [ -f "/home/amatrix/Dart_2026_ws/install/stereo_processor/share/stereo_processor/launch/stereo_processor.launch.py" ] || \
   [ -L "/home/amatrix/Dart_2026_ws/install/stereo_processor/share/stereo_processor/launch/stereo_processor.launch.py" ]; then
    echo -e "${GREEN}✓${NC} Launch file found"
else
    echo -e "${RED}✗${NC} Launch file not found"
    exit 1
fi
echo ""

# 显示节点信息
echo -e "${YELLOW}[6/6]${NC} Displaying package information..."
echo ""
echo "Package: stereo_processor"
echo "Executables:"
ros2 pkg executables stereo_processor
echo ""
echo "Launch files:"
ros2 pkg prefix stereo_processor 2>/dev/null && \
    ls -1 "$(ros2 pkg prefix stereo_processor)/share/stereo_processor/launch/"
echo ""

echo "=========================================="
echo -e "${GREEN}All tests passed!${NC}"
echo "=========================================="
echo ""
echo "To run the node:"
echo "  ros2 launch stereo_processor stereo_processor.launch.py"
echo ""
echo "To check node parameters:"
echo "  ros2 run stereo_processor stereo_processor_node --ros-args --help"
echo ""
echo "To view topics (after starting):"
echo "  ros2 topic list | grep stereo"
echo ""
