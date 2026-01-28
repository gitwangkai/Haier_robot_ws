#!/bin/bash

# 启动 robot_mover 节点的脚本

WORKSPACE_DIR="/home/aidlux/Haier_robot_ws"
NODE_SCRIPT="${WORKSPACE_DIR}/src/robot_mover/robot_mover/mover_node.py"

cd "${WORKSPACE_DIR}"

echo "=========================================="
echo "  启动 robot_mover 节点"
echo "=========================================="
echo ""

# 检查是否已经编译
if [ ! -d "${WORKSPACE_DIR}/install/robot_mover" ]; then
    echo "robot_mover 包未编译，正在编译..."
    colcon build --packages-select robot_mover
    if [ $? -ne 0 ]; then
        echo "编译失败，请检查错误信息"
        exit 1
    fi
fi

# 源代码环境
echo "正在设置环境..."
source "${WORKSPACE_DIR}/install/setup.bash"
echo "环境设置完成"
echo ""

# 检查节点文件是否存在
if [ ! -f "${NODE_SCRIPT}" ]; then
    echo "错误: 节点文件 ${NODE_SCRIPT} 不存在"
    exit 1
fi

# 启动节点
echo "启动 robot_mover 节点..."
echo "服务地址: /robot_mover/rotate"
echo "服务地址: /robot_mover/pause_rotation"
echo ""
echo "按 Ctrl+C 停止节点"
echo ""

python3 "${NODE_SCRIPT}"
