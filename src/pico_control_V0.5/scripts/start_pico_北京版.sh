#!/bin/bash

# 启动机械臂控制程序并显示日志
echo "启动机械臂控制程序..."
cd /home/aidlux/pico/pico_control_V0.5/
python3 vr_arm_control_api.py
echo "程序已运行"
