#!/usr/bin/env bash
#===============================================================================
# 接实车跑中间件: 在 Humble 容器里启动 bridge, 订阅小车的 /scan
#
# 前提:
#   1. 已连上小车 WiFi
#   2. 小车上的雷达已在跑, 例如在 ROS 目录执行 ./connect_wheeltec.sh start
#   3. Godot 里的雷达视图已经运行(它在 127.0.0.1:9100 收包)
#
# 用法: ./run_live.sh [额外参数...]
#===============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DIR="${WHEELTEC_ROS_DIR:-$HOME/桌面/ROS}"
HUMBLE="$ROS_DIR/wheeltec_humble.sh"

if [ ! -x "$HUMBLE" ]; then
    echo "!! 找不到 Humble 容器启动器: $HUMBLE" >&2
    echo "   用 WHEELTEC_ROS_DIR 指定 ROS 工作目录" >&2
    exit 1
fi

if ! docker image inspect wheeltec-humble:latest >/dev/null 2>&1; then
    echo "!! Docker 里没有 wheeltec-humble:latest, 先跑 $ROS_DIR/setup_humble_docker.sh" >&2
    exit 1
fi

echo ">>> 在容器里启动中间件 (ROS 域 ${ROS_DOMAIN_ID:-42}, 话题 /scan) ..."
# --network host 让容器和 Godot 共享 127.0.0.1;
# 家目录按原路径挂载, 所以脚本路径不用换算。
# 默认用域 42; 想换域/加参数直接往后传
if [ "$#" -eq 0 ]; then
    set -- --domain 42
fi
exec "$HUMBLE" python3 "$HERE/lidar_bridge.py" --source ros2 --topic /scan "$@"
