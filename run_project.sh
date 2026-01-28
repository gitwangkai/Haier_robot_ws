#!/bin/bash

WORKSPACE_DIR="/home/aidlux/Haier_robot_ws"

# 启动 robot_mover 节点
echo "=========================================="
echo "  启动 robot_mover 节点"
echo "=========================================="
echo ""

# 检查是否已经编译
if [ ! -d "${WORKSPACE_DIR}/install/robot_mover" ]; then
    echo "robot_mover 包未编译，正在编译..."
    cd "${WORKSPACE_DIR}"
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
NODE_SCRIPT="${WORKSPACE_DIR}/src/robot_mover/robot_mover/mover_node.py"
if [ ! -f "${NODE_SCRIPT}" ]; then
    echo "错误: 节点文件 ${NODE_SCRIPT} 不存在"
    exit 1
fi

# 启动 robot_mover 节点
echo "启动 robot_mover 节点..."
echo "服务地址: /robot_mover/rotate"
echo "服务地址: /robot_mover/pause_rotation"
echo ""
python3 "${NODE_SCRIPT}" > robot_mover.log 2>&1 &
ROBOT_MOVER_PID=$!
sleep 2  # 给 robot_mover 节点一些启动时间
echo "robot_mover 节点已启动，PID: $ROBOT_MOVER_PID"
echo ""

# 启动后端服务
BACKEND_DIR="${WORKSPACE_DIR}/src/pico_control_V0.5"
cd "$BACKEND_DIR"
echo "启动后端服务..."
python3 web_control.py > backend.log 2>&1 &
BACKEND_PID=$!
sleep 3  # 给后端服务一些启动时间

echo "后端服务已启动，PID: $BACKEND_PID"
echo ""

# 启动前端服务
PROJECT_DIR="${WORKSPACE_DIR}/src/robot_control_html"
cd "$PROJECT_DIR"

if ! command -v node &> /dev/null; then
    echo "Node.js 未安装，正在安装..."
    curl -fsSL https://deb.nodesource.com/setup_20.x | sudo bash -
    sudo apt-get install -y nodejs
fi

if ! command -v pnpm &> /dev/null; then
    echo "pnpm 未安装，正在安装..."
    npm install -g pnpm
fi

echo "安装前端依赖..."
pnpm install

echo "启动前端服务..."
echo ""

# 当脚本被中断时，终止所有服务
function cleanup {
    echo "停止所有服务..."
    
    echo "停止后端服务..."
    kill $BACKEND_PID 2>/dev/null
    wait $BACKEND_PID 2>/dev/null
    echo "后端服务已停止"
    
    echo "停止 robot_mover 节点..."
    kill $ROBOT_MOVER_PID 2>/dev/null
    wait $ROBOT_MOVER_PID 2>/dev/null
    echo "robot_mover 节点已停止"
    
    echo "所有服务已停止"
}

trap cleanup SIGINT SIGTERM EXIT

# 启动前端服务（前台运行，这样脚本会等待前端服务结束）
pnpm run dev
