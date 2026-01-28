# 摄像头视频流服务器

## 概述

这是一个基于 Flask 和 OpenCV 的摄像头视频流服务器，提供 HTTP 视频流服务，支持实时视频捕获和多设备切换。

## 功能特性

- 🚀 **实时视频流**：提供 MJPEG 格式的实时视频流
- 📹 **多摄像头支持**：支持切换不同的摄像头设备 (/dev/video2, /dev/video4, /dev/video6)
- 🌐 **Web 界面**：友好的 Web 界面，支持设备切换和视频流查看
- 🔄 **动态切换**：运行时可通过 Web 界面切换摄像头设备
- 📱 **多客户端**：支持多个客户端同时观看视频流
- ⚡ **高性能**：使用多线程架构，30 FPS 实时捕获

## 安装依赖

```bash
pip install flask opencv-python
```

## 使用方法

### 基本启动

```bash
python video_stream.py
```

### 指定摄像头设备

```bash
python video_stream.py --device 2
```

### 指定服务器端口和主机

```bash
python video_stream.py --port 8084 --host 0.0.0.0
```

### 完整参数

```bash
python video_stream.py --device 2 --port 8084 --host 0.0.0.0
```

## 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--device`, `-d` | int | 0 | 摄像头设备ID |
| `--port`, `-p` | int | 8084 | 服务器端口 |
| `--host` | str | '0.0.0.0' | 服务器主机地址 |

## Web 界面功能

访问服务器地址后，可以：

1. **查看视频流**：实时显示摄像头画面
2. **切换摄像头**：选择不同的摄像头设备并切换
3. **刷新视频流**：手动刷新视频流
4. **在新窗口打开**：单独窗口查看视频流

## API 接口

### GET /

主页，显示 Web 界面

### GET /video_feed

返回 MJPEG 视频流

**响应格式**: `multipart/x-mixed-replace; boundary=frame`

### POST /switch_camera

切换摄像头设备

**请求体**:
```json
{
    "device_id": 2
}
```

**响应**:
```json
{
    "success": true,
    "message": "已切换到摄像头 /dev/video2"
}
```

## 技术规格

- **分辨率**: 640x480
- **帧率**: 30 FPS
- **视频格式**: MJPEG
- **并发支持**: 多客户端同时观看

## 系统要求

- Python 3.6+
- Linux 系统（支持 /dev/video* 设备）
- OpenCV 支持的摄像头设备

## 摄像头设备

程序默认支持以下摄像头设备：

- `/dev/video2`
- `/dev/video4`
- `/dev/video6`

可以通过修改代码中的设备列表来添加更多设备。

## 停止服务器

在终端中按 `Ctrl+C` 停止服务器。

## 故障排除

### 摄像头无法打开

- 检查摄像头设备是否存在：`ls /dev/video*`
- 检查摄像头权限：`sudo chmod 666 /dev/video*`
- 确认摄像头未被其他程序占用

### 视频流无画面

- 检查摄像头是否正常工作
- 尝试切换到其他摄像头设备
- 查看终端输出信息

### 端口被占用

- 使用不同的端口：`--port 8085`
- 检查端口占用：`netstat -tlnp | grep 8084`

## 代码结构

```
video_stream.py
├── 全局变量和配置
├── capture_frames()          # 后台视频捕获线程
├── generate_frames()         # 视频流生成器
├── Flask 路由
│   ├── /                    # 主页
│   ├── /video_feed          # 视频流
│   └── /switch_camera       # 摄像头切换
├── init_camera()            # 摄像头初始化
└── main()                   # 主函数
```