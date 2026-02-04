#!/bin/bash

# 停止所有机器人控制系统服务
# 包括：
# 1. 前端生产服务 (next-server)
# 2. 后端服务 (web_control.py)
# 3. robot_mover 节点 (mover_node.py)

WORKSPACE_DIR="/home/aidlux/Haier_robot_ws"
LOG_DIR="${WORKSPACE_DIR}/logs"

echo "=========================================="
echo "  停止所有服务"
echo "=========================================="
echo ""

# 函数：停止服务
stop_service() {
    local service_name=$1
    local pid_file=$2
    local process_pattern=$3
    
    echo "停止 ${service_name}..."
    
    # 1. 尝试通过 PID 文件停止
    if [ -f "${pid_file}" ]; then
        local pid=$(cat "${pid_file}")
        if ps -p $pid > /dev/null; then
            echo "通过 PID 文件停止 ${service_name} (PID: $pid)"
            kill -9 $pid 2>/dev/null
            wait $pid 2>/dev/null
            rm -f "${pid_file}"
            echo "${service_name} 已停止"
            return 0
        else
            echo "PID 文件存在但进程不存在，清理 PID 文件"
            rm -f "${pid_file}"
        fi
    fi
    
    # 2. 尝试通过进程名停止
    local pids=$(ps aux | grep "${process_pattern}" | grep -v grep | awk '{print $2}')
    if [ ! -z "$pids" ]; then
        for pid in $pids; do
            echo "通过进程名停止 ${service_name} (PID: $pid)"
            kill -9 $pid 2>/dev/null
            wait $pid 2>/dev/null
        done
        echo "${service_name} 已停止"
        return 0
    fi
    
    echo "${service_name} 未运行"
    return 1
}

# 停止前端服务
stop_service "前端生产服务" "${LOG_DIR}/frontend.pid" "next-server"

# 停止后端服务
stop_service "后端服务" "${LOG_DIR}/backend.pid" "web_control.py"

# 停止 robot_mover 节点
stop_service "robot_mover 节点" "${LOG_DIR}/robot_mover.pid" "mover_node.py"

# 清理所有 PID 文件
rm -f "${LOG_DIR}/"*.pid 2>/dev/null

echo ""
echo "=========================================="
echo "  所有服务停止操作已完成"
echo "=========================================="
echo ""
echo "日志文件位置: ${LOG_DIR}"
echo "启动服务: 运行 /home/aidlux/Haier_robot_ws/start_production.sh"
echo ""
