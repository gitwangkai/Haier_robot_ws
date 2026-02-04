#!/bin/bash

# 设置脚本在后台稳定运行
# 使用 nohup 启动，即使终端关闭也能继续运行

WORKSPACE_DIR="/home/aidlux/Haier_robot_ws"
LOG_DIR="${WORKSPACE_DIR}/logs"

# 创建日志目录
mkdir -p "${LOG_DIR}"

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
# 使用 nohup 启动，确保后台稳定运行
nohup python3 "${NODE_SCRIPT}" > "${LOG_DIR}/robot_mover.log" 2>&1 &
ROBOT_MOVER_PID=$!
sleep 2  # 给 robot_mover 节点一些启动时间
echo "robot_mover 节点已启动，PID: $ROBOT_MOVER_PID"
echo ""

# 保存 PID 到文件，方便后续停止服务
echo $ROBOT_MOVER_PID > "${LOG_DIR}/robot_mover.pid"

# 启动后端服务
BACKEND_DIR="${WORKSPACE_DIR}/src/pico_control_V0.5"
cd "$BACKEND_DIR"
echo "启动后端服务..."
# 使用 nohup 启动，确保后台稳定运行
nohup python3 web_control.py > "${LOG_DIR}/backend.log" 2>&1 &
BACKEND_PID=$!
sleep 3  # 给后端服务一些启动时间

echo "后端服务已启动，PID: $BACKEND_PID"
echo ""

# 保存 PID 到文件，方便后续停止服务
echo $BACKEND_PID > "${LOG_DIR}/backend.pid"

# 启动前端生产服务
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

# 检查是否已构建
if [ ! -d ".next" ]; then
    echo "生产构建不存在，正在构建..."
    pnpm install
    npx next build
    if [ $? -ne 0 ]; then
        echo "构建失败，请检查错误信息"
        exit 1
    fi
fi

echo "启动前端生产服务..."
echo "服务地址: http://localhost:5001"
echo ""

# 注意：由于使用 nohup 启动服务，脚本退出时不会停止服务
# 服务会在后台继续运行，即使终端关闭
# 要停止服务，请运行：/home/aidlux/Haier_robot_ws/stop_service.sh

# 启动前端生产服务
echo "启动前端生产服务..."
echo "服务地址: http://localhost:5001"
echo ""

# 使用 nohup 启动，确保后台稳定运行
nohup npx next start --port 5001 > "${LOG_DIR}/frontend.log" 2>&1 &
FRONTEND_PID=$!
sleep 5  # 给前端服务一些启动时间

echo "前端生产服务已启动，PID: $FRONTEND_PID"
echo ""

# 保存 PID 到文件，方便后续停止服务
echo $FRONTEND_PID > "${LOG_DIR}/frontend.pid"

echo "=========================================="
echo "  所有服务已成功启动"
echo "=========================================="
echo "服务地址: http://localhost:5001"
echo ""
echo "日志文件位置: ${LOG_DIR}"
echo "停止服务: 运行 /home/aidlux/Haier_robot_ws/stop_service.sh"
echo ""

# 退出脚本，让服务在后台继续运行
exit 0
