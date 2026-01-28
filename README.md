# Haier_robot_ws 项目说明

## 项目概述

本项目是一个集成了视觉检测、姿态估计和机器人控制的综合系统，通过Web界面实现对机器人的实时控制和监控。系统包含机器人跟随功能、机械臂控制、表演动作选择等核心功能。

## 核心文件功能说明

### 主程序文件

- **run_project.sh**：项目启动脚本，整合了所有服务的启动
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

#### pico_control_V0.5目录
- **web_control.py**：后端服务主程序，基于Flask和Socket.IO实现
  - 提供机械臂控制接口
  - 支持表演动作控制
  - 实现Socket.IO实时通信
  - 提供状态监控和反馈

#### robot_control_html目录
- **src/app/page.tsx**：前端页面主文件，基于React/Next.js实现
  - 机械臂控制界面
  - 表演动作选择（仅保留旋转动作）
  - Socket.IO连接状态显示
  - 实时状态监控

#### robot_mover目录
- **robot_mover/mover_node.py**：机器人移动节点，提供旋转服务
  - 服务地址：/robot_mover/rotate
  - 服务地址：/robot_mover/pause_rotation

## 项目结构

```
Haier_robot_ws/
├── run_project.sh             # 整合启动脚本
├── run.sh                     # 项目启动脚本
├── send.sh                    # 数据发送脚本
├── robot.rviz                 # RViz配置文件
├── run_blazepose.sh           # Blazepose启动脚本
└── src/
    ├── BlazeposeRobot/        # 视觉和跟随模块
    │   ├── main.py            # 项目主程序
    │   ├── vision/            # 视觉处理模块
    │   └── follower/          # 机器人跟随控制模块
    ├── pico_control_V0.5/      # 后端服务模块
    │   └── web_control.py     # 后端服务主程序
    ├── robot_control_html/     # 前端Web界面
    │   └── src/app/page.tsx    # 前端页面主文件
    └── robot_mover/            # 机器人移动模块
        └── robot_mover/mover_node.py  # 移动节点实现
```

## 启动方式

### 使用整合启动脚本（推荐）

```bash
bash /home/aidlux/Haier_robot_ws/run_project.sh
```

此脚本会自动：
1. 启动robot_mover节点（带自动编译检查）
2. 启动后端web_control.py服务
3. 安装前端依赖（使用pnpm）
4. 启动前端服务

### 手动启动各服务

1. 启动robot_mover节点：
   ```bash
   cd /home/aidlux/Haier_robot_ws
   source install/setup.bash
   python3 src/robot_mover/robot_mover/mover_node.py
   ```

2. 启动后端服务：
   ```bash
   cd /home/aidlux/Haier_robot_ws/src/pico_control_V0.5
   python3 web_control.py
   ```

3. 启动前端服务：
   ```bash
   cd /home/aidlux/Haier_robot_ws/src/robot_control_html
   pnpm install
   pnpm run dev
   ```

## Web界面

### 前端界面（控制界面）

项目启动后，可通过以下地址访问前端控制界面：
```
http://<设备IP>:3000
```

功能特点：
- 机械臂控制
- 表演动作选择（仅旋转动作）
- Socket.IO连接状态显示
- 实时状态监控

### 后端界面

后端服务启动后，可通过以下地址访问：
```
http://<设备IP>:8082
```

## 核心接口

### Socket.IO接口

- **start_playback**：开始播放动作文件
  ```javascript
  socket.emit('start_playback', {
      filename: 'record_20260127_123456.json',
      speed: 1.0
  });
  ```

- **stop_playback**：停止播放
  ```javascript
  socket.emit('stop_playback');
  ```

- **reset_pose**：重置位姿
  ```javascript
  socket.emit('reset_pose');
  ```

- **get_status**：刷新状态
  ```javascript
  socket.emit('get_status');
  ```

### ROS2服务接口

- **/robot_mover/rotate**：机器人旋转服务
- **/robot_mover/pause_rotation**：暂停旋转服务

## 注意事项

- 确保摄像头已正确连接并配置
- 首次运行可能需要下载YOLOv8模型文件
- 调整config.py中的参数可以优化检测和姿态估计效果
- 确保网络连接稳定，避免Socket.IO连接中断
- 首次启动前端服务时，会自动安装依赖包，可能需要较长时间
- 如果遇到pnpm命令未找到的问题，请先安装pnpm：`npm install -g pnpm`

## 故障排除

1. **Socket.IO连接失败**：
   - 检查后端服务是否正在运行
   - 检查网络连接是否正常
   - 检查防火墙设置是否允许端口访问

2. **机械臂无响应**：
   - 检查机械臂是否正确连接
   - 检查robot_mover节点是否正在运行
   - 检查后端服务是否正常启动

3. **前端服务启动失败**：
   - 检查Node.js是否已安装
   - 检查pnpm是否已安装
   - 检查网络连接是否正常，确保依赖包能够下载

4. **Git推送超时**：
   - 执行以下命令优化Git网络配置：
     ```bash
     git config --global http.postBuffer 524288000
     git config --global http.lowSpeedLimit 0
     git config --global http.lowSpeedTime 999999
     git config --global http.version HTTP/1.1
     ```
