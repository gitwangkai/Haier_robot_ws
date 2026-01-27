# Robot Mover 服务使用说明

## 功能介绍

Robot Mover 包提供了一个旋转服务接口，允许通过ROS 2服务调用控制机器人旋转指定的时间。

## 服务接口

### 服务名称
- `/robot_mover/rotate`

### 服务类型
- `robot_mover/srv/Rotate`

### 服务参数
- `duration` (float64): 旋转持续时间（秒）

### 服务响应
- `success` (bool): 操作是否成功
- `message` (string): 操作结果消息

## 启动服务

### 方法一：直接运行节点

```bash
# 首先加载环境
source install/setup.bash

# 启动跟随节点（包含旋转服务）
ros2 run robot_mover follower
```

### 方法二：使用启动脚本

```bash
# 赋予脚本执行权限
chmod +x start_rotate_service.sh

# 运行脚本
./start_rotate_service.sh
```

## 调用服务

### 示例：旋转 5 秒

```bash
# 加载环境（如果还未加载）
source install/setup.bash

# 调用服务，指定旋转时间为5秒
ros2 service call /robot_mover/rotate robot_mover/srv/Rotate "{duration: 5.0}"
```

### 示例：旋转 10 秒

```bash
ros2 service call /robot_mover/rotate robot_mover/srv/Rotate "{duration: 10.0}"
```

## 服务执行流程

1. 发送服务请求，指定旋转时间
2. 服务端接收请求，开始执行旋转
3. 旋转持续指定的时间后自动停止
4. 服务端返回执行结果

## 注意事项

- 旋转时间从服务请求发送时开始计算
- 服务调用会立即返回，旋转操作在后台执行
- 同一时间只能执行一个旋转操作