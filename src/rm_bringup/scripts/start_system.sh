#!/bin/bash
# 系统快速启动脚本

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== RM系统启动脚本 ===${NC}"
echo ""

# 检查是否已经source了环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}检测到未source ROS环境，正在source...${NC}"
    source /opt/ros/humble/setup.bash
    source /home/amatrix/Dart_2026_ws/install/setup.bash
fi

# 默认参数
CONFIG_FILE=""
STAGE2_DELAY="5.0"
STAGE3_DELAY="10.0"

# 显示帮助信息
show_help() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -c, --config FILE       指定配置文件路径"
    echo "  -2, --stage2-delay SEC  设置Stage 2启动延迟（秒）"
    echo "  -3, --stage3-delay SEC  设置Stage 3启动延迟（秒）"
    echo "  -h, --help             显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0                                    # 使用默认配置"
    echo "  $0 -c my_config.yaml                 # 使用自定义配置"
    echo "  $0 -2 8.0 -3 15.0                    # 自定义延迟时间"
}

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -c|--config)
            CONFIG_FILE="$2"
            shift 2
            ;;
        -2|--stage2-delay)
            STAGE2_DELAY="$2"
            shift 2
            ;;
        -3|--stage3-delay)
            STAGE3_DELAY="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "未知选项: $1"
            show_help
            exit 1
            ;;
    esac
done

# 构建launch命令
LAUNCH_CMD="ros2 launch rm_bringup system_bringup.launch.py stage2_delay:=$STAGE2_DELAY stage3_delay:=$STAGE3_DELAY"

if [ ! -z "$CONFIG_FILE" ]; then
    LAUNCH_CMD="$LAUNCH_CMD system_config_file:=$CONFIG_FILE"
fi

echo -e "${GREEN}启动命令:${NC} $LAUNCH_CMD"
echo ""
echo -e "${YELLOW}启动参数:${NC}"
echo "  - Stage 2延迟: ${STAGE2_DELAY}秒"
echo "  - Stage 3延迟: ${STAGE3_DELAY}秒"
if [ ! -z "$CONFIG_FILE" ]; then
    echo "  - 配置文件: $CONFIG_FILE"
else
    echo "  - 配置文件: 默认"
fi
echo ""

# 执行启动
eval $LAUNCH_CMD
