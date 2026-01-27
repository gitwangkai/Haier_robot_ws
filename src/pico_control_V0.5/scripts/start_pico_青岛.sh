#!/bin/bash

# ==============================================================================
# 启动PICO机械臂控制程序
# 此版本添加了输出重定向，实现静默运行。
# ==============================================================================

PROJECT_DIR="/home/aidlux/pico/pico_control_V0.5"
#PYTHON_SCRIPT="vr_arm_control_V0.5.py"
PYTHON_SCRIPT="vr_arm_control_api.py"


echo "--- 正在切换到项目目录: ${PROJECT_DIR} ---"
cd "${PROJECT_DIR}"

if [ $? -ne 0 ]; then
    echo "❌ 错误: 无法切换到目录 ${PROJECT_DIR}。请检查路径是否正确。"
    exit 1
fi

echo "--- 正在后台静默启动Python控制程序 ---"

# 运行Python脚本，并将标准输出(1)和标准错误(2)重定向到/dev/null
# 最后的 & 符号表示在后台运行
python3 "${PYTHON_SCRIPT}"
#python3 "${PYTHON_SCRIPT}" > /dev/null 2>&1 &

echo "--- 程序已在后台启动 (PID: $!)，将不会显示任何输出。 ---"
echo "--- 您可以使用 'jobs' 或 'ps aux | grep python' 查看进程状态。 ---"
echo "--- 如需停止，请使用 'kill $!' 或 'killall python3'。 ---"

exit 0
