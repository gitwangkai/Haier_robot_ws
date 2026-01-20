# Haier_robot_ws 项目说明

## 项目概述

本项目是一个基于Blazepose的机器人跟随系统，通过视觉检测和姿态估计实现机器人对人体的跟随功能。

## 核心文件功能说明

### 主程序文件

- **run.sh**：项目启动脚本
- **send.sh**：数据发送脚本
- **robot.rviz**：RViz配置文件，用于机器人可视化
- **run_blazepose.sh**：Blazepose姿态估计启动脚本

### src目录下的核心文件

#### BlazeposeRobot目录
- **main.py**：项目主程序，整合视觉处理、机器人控制和Web界面

#### BlazeposeRobot/vision目录
- **__init__.py**：视觉模块导出文件，定义公共接口
- **async_vision.py**：异步视觉处理类，在独立线程中执行目标检测和姿态估计
- **config.py**：视觉模块配置文件，包含模型参数设置
- **pose_estimator.py**：基于MediaPipe的人体姿态估计实现
- **vision_manager.py**：同步视觉处理类，提供目标检测和姿态估计功能
- **vision_processor.py**：视觉处理基类，定义共享功能和接口
- **yolo_detector.py**：基于YOLOv8的目标检测实现

#### BlazeposeRobot/follower目录
- **__init__.py**：跟随模块导出文件，定义公共接口
- **robot_follower_node.py**：机器人跟随节点，实现完整的人体跟随控制逻辑
  - 基于视觉检测结果实现目标选择和跟随
  - 支持PID控制算法，实现平滑的线速度和角速度控制
  - 集成速度斜坡功能，限制加速度，保护机器人底盘
  - 实现完整的状态机管理（IDLE/FOLLOW/LOST/STOP/EMERGENCY）
  - 支持摔倒检测和紧急停止功能
  - 通过ROS2 /cmd_vel接口控制机器人运动

## 项目结构

```
Haier_robot_ws/
├── run.sh                     # 项目启动脚本
├── send.sh                    # 数据发送脚本
├── robot.rviz                 # RViz配置文件
├── run_blazepose.sh           # Blazepose启动脚本
└── src/
    └── BlazeposeRobot/
        ├── main.py            # 项目主程序
        ├── vision/            # 视觉处理模块
        └── follower/          # 机器人跟随控制模块
```

## 启动方式

1. 使用run.sh脚本启动项目：
   ```bash
   ~/Haier_robot_ws/run_blazepose.sh 
   ```

2. 或手动执行启动命令：
   ```bash
   cd src/BlazeposeRobot
   python main.py
   ```

## Web界面

项目启动后，可通过以下地址访问Web监控界面：
```
http://<设备IP>:55555
```

## 注意事项

- 确保摄像头已正确连接并配置
- 首次运行可能需要下载YOLOv8模型文件
- 调整config.py中的参数可以优化检测和姿态估计效果
