# Web控制界面功能模块说明

本文档详细介绍网页控制界面中的各个功能模块及其对应的代码位置。

## 目录
- [1. 系统初始化模块](#1-系统初始化模块)
- [2. 舵机控制核心类](#2-舵机控制核心类)
- [3. 姿态控制模块](#3-姿态控制模块)
- [4. 状态监控模块](#4-状态监控模块)
- [5. 电源管理模块](#5-电源管理模块)
- [6. 动作录制模块](#6-动作录制模块)
- [7. 动作回放模块](#7-动作回放模块)
- [8. 文件管理模块](#8-文件管理模块)
- [9. 同步控制模块](#9-同步控制模块)

---

## 1. 系统初始化模块

### 功能描述
负责Flask服务器启动、Socket.IO连接管理、硬件串口初始化等系统级功能。

### 主要功能
- **服务器启动**: 启动Flask和Socket.IO服务器，监听8082端口
- **客户端连接处理**: 处理客户端连接事件，自动触发硬件初始化
- **硬件初始化**: 后台线程中初始化左右臂串口连接
- **初始姿态设置**: 将双臂移动到预设的安全初始位置

### 代码位置
- **全局配置**: [web_control.py#L26-L29](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L26-L29)
  ```python
  app = Flask(__name__)
  app.config['SECRET_KEY'] = 'secret_key_for_robot_arm_!@#'
  socketio = SocketIO(app, async_mode='eventlet')
  ```

- **硬件常量定义**: [web_control.py#L31-L35](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L31-L35)
  ```python
  RIGHT_ARM_PORT = "/dev/arm_right"
  LEFT_ARM_PORT = "/dev/ttyUSB10"
  BAUDRATE = 115200
  SERVO_IDS = [1, 2, 3, 4, 5, 6, 7]
  ARM_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_data")
  ```

- **后台初始化函数**: [web_control.py#L232-L262](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L232-L262)
  - `do_initialization()`: 在后台线程中执行硬件初始化
  - 创建左右臂控制器实例
  - 测试串口连接
  - 调用初始姿态设置

- **客户端连接处理**: [web_control.py#L265-L277](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L265-L277)
  ```python
  @socketio.on('connect')
  def handle_connect():
  ```

- **服务器启动**: [web_control.py#L788](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L788)
  ```python
  socketio.run(app, host='0.0.0.0', port=8082, debug=False)
  ```

---

## 2. 舵机控制核心类

### 功能描述
`MultiServoController` 类是整个系统的核心，负责与舵机硬件的所有底层通信。

### 主要功能
- **串口通信**: 通过串口发送指令和接收响应
- **位置读取**: 读取单个或所有舵机的当前位置
- **状态读取**: 读取舵机电压、温度、负载等状态信息
- **运动控制**: 控制舵机移动到指定位置
- **力矩控制**: 上力（保持位置）或卸力（放松）

### 代码位置
- **类定义**: [web_control.py#L60-L62](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L60-L62)
  ```python
  class MultiServoController:
  ```

- **校验和计算**: [web_control.py#L65-L66](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L65-L66)
  ```python
  def checksum(self, data):
      return (~sum(data[2:])) & 0xFF
  ```

- **发送指令**: [web_control.py#L68-L77](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L68-L77)
  ```python
  def send_command(self, servo_id, cmd, data=None):
  ```

- **读取位置**: [web_control.py#L95-L99](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L95-L99)
  ```python
  def read_position(self, servo_id):
      response = self.send_read_command(servo_id, 28)
  ```

- **读取电压**: [web_control.py#L101-L105](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L101-L105)
  ```python
  def read_voltage(self, servo_id):
      response = self.send_read_command(servo_id, 27)
  ```

- **读取温度**: [web_control.py#L107-L111](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L107-L111)
  ```python
  def read_temperature(self, servo_id):
      response = self.send_read_command(servo_id, 26)
  ```

- **读取负载**: [web_control.py#L113-L118](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L113-L118)
  ```python
  def read_load(self, servo_id):
      response = self.send_read_command(servo_id, 29)
  ```

- **读取所有状态**: [web_control.py#L120-L129](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L120-L129)
  ```python
  def read_all_statuses(self):
  ```

- **移动舵机**: [web_control.py#L135-L138](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L135-L138)
  ```python
  def move_servo(self, servo_id, position, duration=100):
  ```

- **设置力矩状态**: [web_control.py#L140-L142](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L140-L142)
  ```python
  def set_unload(self, servo_id, unload):
  ```

- **卸载所有舵机**: [web_control.py#L144-L149](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L144-L149)
  ```python
  def unload_all_servos(self):
  ```

- **装载所有舵机**: [web_control.py#L151-L156](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L151-L156)
  ```python
  def load_all_servos(self):
  ```

---

## 3. 姿态控制模块

### 功能描述
控制机械臂移动到预设的安全姿态，用于初始化和重置。

### 主要功能
- **初始姿态设置**: 系统启动时自动设置双臂到安全位置
- **姿态重置**: 用户手动触发重置双臂到初始位置
- **姿态参数**: 初始角度数组 [120, 120, 120, 120, 120, 120, 80]

### 代码位置
- **姿态初始化函数**: [web_control.py#L189-L229](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L189-L229)
  ```python
  def initialize_arms_to_default_pose(force=False):
  ```

- **重置姿态事件处理**: [web_control.py#L280-L285](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L280-L285)
  ```python
  @socketio.on('reset_pose')
  def handle_reset_pose():
  ```

- **初始角度定义**: [web_control.py#L195](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L195)
  ```python
  init_angles = [120, 120, 120, 120, 120, 120, 80]
  ```

---

## 4. 状态监控模块

### 功能描述
实时监控和显示机械臂的运行状态，包括位置、电压、温度、负载等信息。

### 主要功能
- **状态请求**: 前端请求刷新状态
- **状态读取**: 读取所有舵机的完整状态信息
- **状态推送**: 将状态数据实时推送到前端显示

### 代码位置
- **获取状态事件处理**: [web_control.py#L293-L301](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L293-L301)
  ```python
  @socketio.on('get_status')
  def handle_get_status():
      log_action('请求刷新状态...')
      status = {'right': {}, 'left': {}}
      if right_controller:
          status['right'] = right_controller.read_all_statuses()
      if left_controller:
          status['left'] = left_controller.read_all_statuses()
      socketio.emit('arm_status', status)
  ```

- **状态数据结构**: [web_control.py#L120-L129](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L120-L129)
  ```python
  def read_all_statuses(self):
      statuses = {}
      for sid in self.servo_ids:
          pos = self.read_position(sid)
          if pos is not None:
              statuses[sid] = {
                  'pos': pos,
                  'volt': self.read_voltage(sid),
                  'temp': self.read_temperature(sid),
                  'load': self.read_load(sid)
              }
  ```

---

## 5. 电源管理模块

### 功能描述
控制舵机的力矩状态，包括上力（保持位置）和卸力（放松）。

### 主要功能
- **单舵机控制**: 对单个舵机进行上力/卸力操作
- **整臂控制**: 对整条手臂的所有舵机进行统一操作
- **双臂控制**: 同时对双臂进行操作
- **操作反馈**: 实时显示操作结果

### 代码位置
- **电源管理事件处理**: [web_control.py#L303-L336](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L303-L336)
  ```python
  @socketio.on('manage_power')
  def handle_manage_power(data):
      target = data.get('target')
      action = data.get('action')
  ```

- **卸载所有舵机**: [web_control.py#L144-L149](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L144-L149)
  ```python
  def unload_all_servos(self):
  ```

- **装载所有舵机**: [web_control.py#L151-L156](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L151-L156)
  ```python
  def load_all_servos(self):
  ```

- **设置力矩状态**: [web_control.py#L140-L142](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L140-L142)
  ```python
  def set_unload(self, servo_id, unload):
  ```

---

## 6. 动作录制模块

### 功能描述
录制机械臂的动作轨迹并保存为JSON文件，用于后续回放。

### 主要功能
- **开始录制**: 启动录制线程，记录舵机位置变化
- **停止录制**: 停止录制并保存数据到文件
- **录制参数**: 支持选择录制左臂、右臂或双臂
- **文件命名**: 支持自定义文件名或自动生成时间戳文件名
- **录制进度**: 实时显示录制时间和帧数
- **自动卸力**: 录制开始时自动卸力，停止时恢复

### 代码位置
- **开始录制事件处理**: [web_control.py#L368-L392](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L368-L392)
  ```python
  @socketio.on('start_recording')
  def handle_start_recording(data):
  ```

- **停止录制事件处理**: [web_control.py#L394-L407](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L394-L407)
  ```python
  @socketio.on('stop_recording')
  def handle_stop_recording():
  ```

- **录制工作线程**: [web_control.py#L338-L366](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L338-L366)
  ```python
  def record_worker(filename, arms_to_record):
  ```

- **录制频率**: [web_control.py#L345](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L345)
  ```python
  interval = 0.05  # 20Hz
  ```

- **数据格式**: [web_control.py#L347-L359](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L347-L359)
  ```python
  entry = {"time": round(current_time, 4)}
  if 'right' in arms_to_record:
      pos = right_controller.read_all_positions()
      if any(p is not None for p in pos.values()):
          entry["right_arm_angles"] = {f"{s:03d}": p for s, p in pos.items() if p is not None}
  ```

---

## 7. 动作回放模块

### 功能描述
从JSON文件读取录制的动作数据，并控制机械臂执行回放。

### 主要功能
- **文件读取**: 从JSON文件加载动作数据
- **平滑回放**: 使用插值算法实现平滑运动
- **速度控制**: 支持调节回放速度
- **自动上力**: 回放开始时自动上力
- **错误处理**: 处理文件不存在、文件为空等异常情况

### 代码位置
- **回放工作线程**: [web_control.py#L438-L540](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L438-L540)
  ```python
  def playback_worker(filename, speed):
  ```

- **文件读取**: [web_control.py#L445-L457](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L445-L457)
  ```python
  try:
      with open(filepath, 'r') as f:
          action_data = json.load(f)
  except Exception as e:
      log_action(f'❌ 读取文件失败: {e}', 'error')
  ```

- **控制参数**: [web_control.py#L467](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L467)
  ```python
  CONTROL_FREQUENCY, MAX_SPEED_UNITS_PER_SEC = 50, 800
  ```

- **初始位置设置**: [web_control.py#L472-L477](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L472-L477)
  ```python
  first_frame = action_data[0]
  if "right_arm_angles" in first_frame and right_controller:
      initial_pos = {int(k): v for k, v in first_frame["right_arm_angles"].items()}
  ```

---

## 8. 文件管理模块

### 功能描述
管理动作文件的增删改查操作。

### 主要功能
- **获取文件列表**: 列出所有已保存的动作文件
- **删除文件**: 删除指定的动作文件
- **重命名文件**: 重命名指定的动作文件
- **自动刷新**: 文件操作后自动刷新列表

### 代码位置
- **获取动作文件列表**: [web_control.py#L338-L342](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L338-L342)
  ```python
  @socketio.on('get_action_files')
  def handle_get_action_files():
      log_action('请求刷新动作文件列表...')
      files = []
      if os.path.exists(ARM_DATA_DIR):
          files = [f for f in os.listdir(ARM_DATA_DIR) if f.endswith('.json')]
      emit('action_files_list', {'files': sorted(files, reverse=True)})
  ```

- **删除动作文件**: [web_control.py#L409-L425](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L409-L425)
  ```python
  @socketio.on('delete_action_file')
  def handle_delete_action_file(data):
      filename = data.get('filename')
      filepath = os.path.join(ARM_DATA_DIR, filename)
      if os.path.exists(filepath):
          os.remove(filepath)
  ```

- **重命名动作文件**: [web_control.py#L427-L436](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L427-L436)
  ```python
  @socketio.on('rename_action_file')
  def handle_rename_action_file(data):
      old_filename = data.get('old_filename')
      new_filename = data.get('new_filename')
  ```

---

## 9. 同步控制模块

### 功能描述
实现双臂的同步控制功能。

### 主要功能
- **双臂同步**: 同时控制左右臂执行相同的动作
- **同步录制**: 同时录制双臂的动作
- **同步回放**: 同时回放双臂的动作

### 代码位置
- **同步线程变量**: [web_control.py#L45](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L45)
  ```python
  sync_thread = None
  sync_active = False
  ```

- **同步状态检查**: [web_control.py#L371-L374](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L371-L374)
  ```python
  if recording_active or playback_active or sync_active:
      log_action('无法开始录制：另一个操作正在进行中。', 'error')
  ```

- **双臂录制支持**: [web_control.py#L349-L359](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L349-L359)
  ```python
  if 'right' in arms_to_record:
      pos = right_controller.read_all_positions()
      if any(p is not None for p in pos.values()):
          entry["right_arm_angles"] = {f"{s:03d}": p for s, p in pos.items() if p is not None}
  
  if 'left' in arms_to_record:
      pos = left_controller.read_all_positions()
      if any(p is not None for p in pos.values()):
          entry["left_arm_angles"] = {f"{s:03d}": p for s, p in pos.items() if p is not None}
  ```

---

## 附录

### Socket.IO事件列表

| 事件名称 | 功能描述 | 代码位置 |
|---------|---------|---------|
| `connect` | 客户端连接 | [web_control.py#L265](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L265) |
| `disconnect` | 客户端断开 | [web_control.py#L287](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L287) |
| `reset_pose` | 重置姿态 | [web_control.py#L280](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L280) |
| `get_status` | 获取状态 | [web_control.py#L293](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L293) |
| `manage_power` | 电源管理 | [web_control.py#L303](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L303) |
| `get_action_files` | 获取文件列表 | [web_control.py#L338](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L338) |
| `start_recording` | 开始录制 | [web_control.py#L368](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L368) |
| `stop_recording` | 停止录制 | [web_control.py#L394](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L394) |
| `delete_action_file` | 删除文件 | [web_control.py#L409](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L409) |
| `rename_action_file` | 重命名文件 | [web_control.py#L427](file:///home/aidlux/Haier_robot_ws/src/pico_control_V0.5/web_control.py#L427) |

### 关键常量

| 常量名称 | 值 | 说明 |
|---------|---|------|
| `RIGHT_ARM_PORT` | `/dev/arm_right` | 右臂串口设备 |
| `LEFT_ARM_PORT` | `/dev/ttyUSB10` | 左臂串口设备 |
| `BAUDRATE` | `115200` | 串口波特率 |
| `SERVO_IDS` | `[1, 2, 3, 4, 5, 6, 7]` | 舵机ID列表 |
| `ARM_DATA_DIR` | `./arm_data` | 动作数据存储目录 |
| `CONTROL_FREQUENCY` | `50` | 控制频率 (Hz) |
| `MAX_SPEED_UNITS_PER_SEC` | `800` | 最大速度 (单位/秒) |

### 数据文件格式

动作文件采用JSON格式，结构如下：

```json
[
  {
    "time": 0.0,
    "right_arm_angles": {
      "001": 500,
      "002": 500,
      ...
    },
    "left_arm_angles": {
      "001": 500,
      "002": 500,
      ...
    }
  },
  ...
]
```

- `time`: 相对开始时间（秒）
- `right_arm_angles`: 右臂各舵机位置（键为舵机ID，值为位置值0-1000）
- `left_arm_angles`: 左臂各舵机位置（键为舵机ID，值为位置值0-1000）

---

**文档版本**: 1.0  
**最后更新**: 2026-01-27  
**维护者**: Pico Control System Team
