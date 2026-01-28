# robot_music 功能包

## 功能描述

robot_music 是一个ROS 2功能包，用于控制机器人底板的音频设备，播放指定文件夹中的音频文件。该功能包提供了以下功能：

- 播放指定路径的音频文件
- 从默认目录播放音频文件
- 停止当前播放的音频
- 检查音频播放状态
- 提供ROS服务接口，供其他功能包调用

## 目录结构

```
robot_music/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── music_player.launch
├── run_music_player.sh
├── srv/
│   └── PlayAudio.srv
└── src/
    ├── audio_player.py
    ├── music_player_node.py
    ├── test_audio_player.py
    └── example_usage.py
```

## 核心文件说明

### audio_player.py
音频播放器核心类，使用pygame库实现音频播放功能。

### music_player_node.py
ROS 2节点，提供音频播放相关的服务和话题接口。

### test_audio_player.py
测试脚本，用于测试音频播放器的功能。

### example_usage.py
示例脚本，展示如何从其他功能包调用robot_music的音频播放功能。

### PlayAudio.srv
自定义服务类型，用于播放音频的服务请求和响应。

## 使用方法

### 启动音频播放器节点

```bash
# 方法1：使用ros2 run命令
ros2 run robot_music music_player_node.py

# 方法2：使用launch文件
ros2 launch robot_music music_player.launch

# 方法3：使用脚本文件
./run_music_player.sh
```

### 调用音频播放服务

#### 播放默认目录中的音频文件

```python
from robot_music.srv import PlayAudio
from rclpy.node import Node

class MusicClient(Node):
    def __init__(self):
        super().__init__('music_client')
        self.client = self.create_client(PlayAudio, 'play_audio')
    
    def play_audio(self, filename):
        req = PlayAudio.Request()
        req.audio_file = filename
        req.use_default_dir = True
        future = self.client.call_async(req)
        # 等待服务响应
        rclpy.spin_until_future_complete(self, future)
        return future.result()
```

#### 播放指定路径的音频文件

```python
req = PlayAudio.Request()
req.audio_file = '/path/to/audio/file.mp3'
req.use_default_dir = False
future = self.client.call_async(req)
```

#### 停止音频播放

```python
from std_srvs.srv import Trigger

# 创建服务客户端
stop_client = node.create_client(Trigger, 'stop_audio')

# 发送停止请求
req = Trigger.Request()
future = stop_client.call_async(req)
```

#### 检查音频播放状态

```python
from std_srvs.srv import Trigger

# 创建服务客户端
check_client = node.create_client(Trigger, 'check_audio_playing')

# 发送检查请求
req = Trigger.Request()
future = check_client.call_async(req)
```

## 配置参数

### 音频文件默认目录

音频播放器节点默认从 `/home/aidlux/Haier_robot_ws/audio` 目录读取音频文件。您可以通过以下方式修改默认目录：

1. 在launch文件中修改：

```xml
<node pkg="robot_music" type="music_player_node.py" name="music_player_node" output="screen">
    <param name="audio_dir" value="/path/to/your/audio/directory" />
</node>
```

2. 在运行节点时通过命令行参数修改：

```bash
ros2 run robot_music music_player_node.py --ros-args -p audio_dir:=/path/to/your/audio/directory
```

## 测试

### 运行测试脚本

```bash
ros2 run robot_music test_audio_player.py
```

### 示例用法

```bash
ros2 run robot_music example_usage.py
```

## 依赖项

- ROS 2 Humble
- Python 3.10+
- pygame

## 安装依赖

```bash
pip install pygame
```

## 注意事项

1. 确保机器人底板上有可用的音频设备
2. 确保音频文件格式被pygame库支持（如MP3、WAV等）
3. 确保音频文件路径正确，并且有读取权限
4. 如果在没有音频设备的环境中运行，可能会看到ALSA相关的错误，但这不会影响服务的可用性
