#!/bin/bash

# Stereo Distance Estimator - 演示脚本
# 此脚本将启动节点和测试发布器，演示完整的功能

echo "=================================="
echo "Stereo Distance Estimator Demo"
echo "=================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查是否在工作空间中
if [ ! -f "install/setup.bash" ]; then
    echo -e "${RED}错误: 请在 Dart_2026_ws 目录下运行此脚本${NC}"
    exit 1
fi

# Source 环境
echo -e "${YELLOW}[1/4] 加载 ROS2 环境...${NC}"
source install/setup.bash
echo -e "${GREEN}✓ 环境已加载${NC}"
echo ""

# 检查包是否编译
if [ ! -f "install/stereo_distance_estimator/lib/stereo_distance_estimator/stereo_distance_estimator_node" ]; then
    echo -e "${YELLOW}包未编译，正在编译...${NC}"
    colcon build --packages-select stereo_distance_estimator
    source install/setup.bash
fi

echo -e "${GREEN}✓ 包已编译${NC}"
echo ""

# 创建临时目录存储 PID
TMP_DIR="/tmp/stereo_distance_estimator_demo"
mkdir -p $TMP_DIR

# 清理函数
cleanup() {
    echo ""
    echo -e "${YELLOW}正在清理...${NC}"
    if [ -f "$TMP_DIR/node.pid" ]; then
        NODE_PID=$(cat $TMP_DIR/node.pid)
        kill $NODE_PID 2>/dev/null
        echo -e "${GREEN}✓ 已停止 stereo_distance_estimator 节点${NC}"
    fi
    if [ -f "$TMP_DIR/test.pid" ]; then
        TEST_PID=$(cat $TMP_DIR/test.pid)
        kill $TEST_PID 2>/dev/null
        echo -e "${GREEN}✓ 已停止测试发布器${NC}"
    fi
    rm -rf $TMP_DIR
    echo -e "${GREEN}演示结束${NC}"
    exit 0
}

# 注册信号处理
trap cleanup SIGINT SIGTERM

echo -e "${YELLOW}[2/4] 启动 stereo_distance_estimator 节点...${NC}"
ros2 launch stereo_distance_estimator stereo_distance_estimator.launch.py > /tmp/stereo_node.log 2>&1 &
NODE_PID=$!
echo $NODE_PID > $TMP_DIR/node.pid
sleep 2

if ps -p $NODE_PID > /dev/null; then
    echo -e "${GREEN}✓ 节点已启动 (PID: $NODE_PID)${NC}"
else
    echo -e "${RED}✗ 节点启动失败${NC}"
    echo "查看日志: tail /tmp/stereo_node.log"
    exit 1
fi
echo ""

echo -e "${YELLOW}[3/4] 启动测试数据发布器...${NC}"
ros2 run stereo_distance_estimator test_publisher.py > /tmp/stereo_test.log 2>&1 &
TEST_PID=$!
echo $TEST_PID > $TMP_DIR/test.pid
sleep 2

if ps -p $TEST_PID > /dev/null; then
    echo -e "${GREEN}✓ 测试发布器已启动 (PID: $TEST_PID)${NC}"
else
    echo -e "${RED}✗ 测试发布器启动失败${NC}"
    cleanup
    exit 1
fi
echo ""

echo -e "${YELLOW}[4/4] 监听输出话题...${NC}"
echo -e "${GREEN}正在接收 3D 目标数据...${NC}"
echo ""
echo "=================================="
echo "输出示例 (按 Ctrl+C 停止):"
echo "=================================="
echo ""

# 显示输出
ros2 topic echo /stereo/target3d_array_raw --once

echo ""
echo "=================================="
echo "系统信息:"
echo "=================================="
echo ""

# 显示节点信息
echo -e "${YELLOW}节点列表:${NC}"
ros2 node list | grep stereo

echo ""
echo -e "${YELLOW}话题列表:${NC}"
ros2 topic list | grep -E "(target|stereo)" | head -10

echo ""
echo -e "${YELLOW}话题频率:${NC}"
timeout 5 ros2 topic hz /stereo/target3d_array_raw 2>/dev/null || echo "等待更多消息..."

echo ""
echo "=================================="
echo -e "${GREEN}演示正在运行...${NC}"
echo "按 Ctrl+C 停止演示"
echo "=================================="
echo ""

# 保持运行
wait $NODE_PID $TEST_PID
