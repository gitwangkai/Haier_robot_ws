#!/bin/bash

PROJECT_DIR="/home/aidlux/Haier_robot_ws/src/robot_control_html"
cd "$PROJECT_DIR"

if ! command -v node &> /dev/null; then
    echo "Node.js 未安装，正在安装..."
    curl -fsSL https://deb.nodesource.com/setup_20.x | sudo bash -
    sudo apt-get install -y nodejs
fi

if ! command -v npm &> /dev/null; then
    echo "npm 未安装，正在安装..."
    sudo apt-get install -y npm
fi

npm install
npm run dev
