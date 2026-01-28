#!/bin/bash

# 启动音频播放器节点

# 设置工作目录
cd "$(dirname "$0")/.." || exit 1

# 确保音频目录存在
mkdir -p audio

# 启动音频播放器节点
rosrun robot_music music_player_node.py
