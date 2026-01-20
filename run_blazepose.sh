#!/bin/bash

# 出错即退出
set -e

# 进入项目目录
cd /home/aidlux/Haier_robot_ws/src/BlazeposeRobot || {
  echo "目录不存在：/home/aidlux/Haier_robot_ws/src/BlazeposeRobot"
  exit 1
}

# 激活虚拟环境
source venv/bin/activate

# 运行主程序 - 使用xvfb-run提供虚拟X11显示
xvfb-run -a python3 main.py

