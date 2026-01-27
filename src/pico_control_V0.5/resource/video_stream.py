#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
摄像头视频流服务器
提供HTTP视频流服务
"""

import cv2
import sys
import time
import threading
from flask import Flask, Response, render_template_string, request
import argparse

app = Flask(__name__)

# 全局变量
camera = None
camera_id = 0
frame = None
frame_lock = threading.Lock()

def capture_frames():
    """后台线程持续捕获视频帧"""
    global camera, frame

    while True:
        if camera is not None and camera.isOpened():
            ret, current_frame = camera.read()
            if ret:
                with frame_lock:
                    frame = current_frame.copy()
        time.sleep(0.033)  # 约30FPS

def generate_frames():
    """生成视频流帧"""
    global frame

    while True:
        with frame_lock:
            if frame is not None:
                # 将帧编码为JPEG
                ret, buffer = cv2.imencode('.jpg', frame)
                if ret:
                    frame_bytes = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(0.033)

@app.route('/')
def index():
    """主页"""
    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>摄像头视频流</title>
        <style>
            body { font-family: Arial, sans-serif; text-align: center; margin: 20px; }
            .container { max-width: 800px; margin: 0 auto; }
            .stream { border: 2px solid #333; margin: 20px 0; }
            .controls { margin: 20px 0; }
            button { padding: 10px 20px; margin: 0 10px; font-size: 16px; }
            .info { background: #f0f0f0; padding: 10px; border-radius: 5px; margin: 10px 0; }
            .device-selector { background: #e8f4f8; padding: 15px; border-radius: 5px; margin: 10px 0; }
            .device-selector select, .device-selector input { padding: 5px; margin: 0 10px; font-size: 16px; }
            .status { color: green; font-weight: bold; }
            .error { color: red; font-weight: bold; }
        </style>
        <script>
            function switchCamera() {
                const deviceId = document.getElementById('deviceSelect').value;
                const statusDiv = document.getElementById('status');

                statusDiv.innerHTML = '🔄 正在切换摄像头...';
                statusDiv.className = 'status';

                fetch('/switch_camera', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({ device_id: parseInt(deviceId) })
                })
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        statusDiv.innerHTML = '✅ 摄像头切换成功！';
                        // 刷新页面以更新信息
                        setTimeout(() => {
                            location.reload();
                        }, 1000);
                    } else {
                        statusDiv.innerHTML = '❌ ' + data.message;
                        statusDiv.className = 'error';
                    }
                })
                .catch(error => {
                    statusDiv.innerHTML = '❌ 切换失败: ' + error.message;
                    statusDiv.className = 'error';
                });
            }

            function refreshStream() {
                const videoElement = document.querySelector('img');
                videoElement.src = videoElement.src;
            }
        </script>
    </head>
    <body>
        <div class="container">
            <h1>摄像头视频流服务器</h1>

            <div class="device-selector">
                <h3>摄像头设备选择</h3>
                <label for="deviceSelect">选择摄像头设备:</label>
                <select id="deviceSelect">
                    <option value="2" {% if camera_id == 2 %}selected{% endif %}>/dev/video2</option>
                    <option value="4" {% if camera_id == 4 %}selected{% endif %}>/dev/video4</option>
                    <option value="6" {% if camera_id == 6 %}selected{% endif %}>/dev/video6</option>
                </select>
                <button onclick="switchCamera()">切换摄像头</button>
                <button onclick="refreshStream()">刷新视频流</button>
                <div id="status"></div>
            </div>

            <div class="info">
                <p><strong>当前摄像头设备:</strong> /dev/video{{ camera_id }}</p>
                <p><strong>分辨率:</strong> 640x480</p>
                <p><strong>帧率:</strong> 30 FPS</p>
            </div>

            <div class="controls">
                <button onclick="location.reload()">刷新页面</button>
                <button onclick="window.open('/video_feed', '_blank')">在新窗口打开视频流</button>
            </div>

            <img src="/video_feed" class="stream" alt="视频流" />

            <div class="info">
                <h3>使用说明:</h3>
                <ul style="text-align: left; display: inline-block;">
                    <li>选择摄像头设备并点击"切换摄像头"按钮</li>
                    <li>直接在浏览器中查看实时视频流</li>
                    <li>支持多个客户端同时观看</li>
                    <li>按 Ctrl+C 停止服务器</li>
                </ul>
            </div>
        </div>
    </body>
    </html>
    """
    return render_template_string(html, camera_id=camera_id)

@app.route('/video_feed')
def video_feed():
    """视频流路由"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/switch_camera', methods=['POST'])
def switch_camera():
    """切换摄像头设备"""
    global camera, camera_id

    try:
        data = request.get_json()
        new_device_id = data.get('device_id')

        if new_device_id is None:
            return {'success': False, 'message': '未提供设备ID'}

        print(f"正在切换到摄像头 /dev/video{new_device_id}...")

        # 释放当前摄像头
        if camera is not None:
            camera.release()

        # 初始化新摄像头
        camera = cv2.VideoCapture(new_device_id)

        if not camera.isOpened():
            return {'success': False, 'message': f'无法打开摄像头 /dev/video{new_device_id}'}

        # 设置摄像头参数
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        camera.set(cv2.CAP_PROP_FPS, 30)

        # 获取实际参数
        width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = camera.get(cv2.CAP_PROP_FPS)

        camera_id = new_device_id
        print("✅ 摄像头切换成功:")
        print(f"   设备: /dev/video{new_device_id}")
        print(f"   分辨率: {width}x{height}")
        print(f"   帧率: {fps} FPS")

        return {'success': True, 'message': f'已切换到摄像头 /dev/video{new_device_id}'}

    except Exception as e:
        print(f"❌ 切换摄像头时出错: {e}")
        return {'success': False, 'message': f'切换失败: {str(e)}'}

def init_camera(device_id):
    """初始化摄像头"""
    global camera, camera_id

    print(f"正在初始化摄像头 /dev/video{device_id}...")

    try:
        camera = cv2.VideoCapture(device_id)

        if not camera.isOpened():
            print(f"❌ 无法打开摄像头 /dev/video{device_id}")
            return False

        # 设置摄像头参数
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        camera.set(cv2.CAP_PROP_FPS, 30)

        # 获取实际参数
        width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = camera.get(cv2.CAP_PROP_FPS)

        camera_id = device_id
        print("✅ 摄像头初始化成功:")
        print(f"   设备: /dev/video{device_id}")
        print(f"   分辨率: {width}x{height}")
        print(f"   帧率: {fps} FPS")

        return True

    except Exception as e:
        print(f"❌ 初始化摄像头时出错: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='摄像头视频流服务器')
    parser.add_argument('--device', '-d', type=int, default=0,
                       help='摄像头设备ID (默认: 0)')
    parser.add_argument('--port', '-p', type=int, default=8084,
                       help='服务器端口 (默认: 8084)')
    parser.add_argument('--host', type=str, default='0.0.0.0',
                       help='服务器主机地址 (默认: 0.0.0.0)')

    args = parser.parse_args()

    print("🎥 摄像头视频流服务器")
    print("=" * 50)

    # 初始化摄像头
    if not init_camera(args.device):
        print("❌ 摄像头初始化失败，退出程序")
        return

    # 启动后台捕获线程
    capture_thread = threading.Thread(target=capture_frames, daemon=True)
    capture_thread.start()

    print(f"\n🚀 启动视频流服务器...")
    print(f"📡 服务器地址: http://{args.host}:{args.port}")
    print(f"🎬 视频流地址: http://{args.host}:{args.port}/video_feed")
    print("💡 在浏览器中打开上述地址即可观看视频流")
    print("🛑 按 Ctrl+C 停止服务器")
    print("=" * 50)

    try:
        app.run(host=args.host, port=args.port, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n👋 正在停止服务器...")
    finally:
        if camera:
            camera.release()
        print("✅ 服务器已停止")

if __name__ == "__main__":
    main()
