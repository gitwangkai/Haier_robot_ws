import cv2
import time
import datetime as dt
import rclpy
from rclpy.node import Node
from matplotlib import pyplot as plt
from matplotlib import animation
from flask import Flask, Response
import threading
import queue
import numpy as np

from follower.robot_follower_node import RobotFollowerNode
from vision.async_vision import AsyncVision
from utils.geometry import calculate_angle
from vision.pose_estimator import PoseEstimator

# Flask Web服务器配置
app = Flask(__name__)
frame_queue = queue.Queue(maxsize=1)  # 视频帧队列，限制大小避免内存占用过多

# 生成视频流的函数
def generate_frames():
    while True:
        try:
            frame = frame_queue.get(timeout=1.0)
            # 将帧转换为JPEG格式
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            # 转换为字节流
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        except queue.Empty:
            continue
        except Exception as e:
            print(f"视频流生成错误: {e}")

# Flask路由：视频流
@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Flask路由：主页面
@app.route('/')
def index():
    return '''
    <html>
        <head>
            <title>BlazePose 机器人跟随 - Web监控</title>
            <style>
                body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background-color: #f0f0f0; }
                h1 { color: #333; text-align: center; }
                .container { max-width: 800px; margin: 0 auto; }
                .video-container { border: 2px solid #ddd; border-radius: 8px; overflow: hidden; }
                img { width: 100%; height: auto; }
            </style>
        </head>
        <body>
            <div class="container">
                <h1>BlazePose 机器人跟随系统</h1>
                <div class="video-container">
                    <img src="/video_feed" alt="BlazePose 实时视频">
                </div>
            </div>
        </body>
    </html>
    '''


# ===============================
# Plot（保持你原来的）
# ===============================
fig, ax = plt.subplots()
times, angles = [], []

def update_plot(i):
    ax.clear()
    ax.plot(times, angles, label="Knee Angle")
    ax.legend()

ani = animation.FuncAnimation(
    fig, update_plot, interval=1000, cache_frame_data=False
)


# ===============================
# 主程序
# ===============================
def main():
    rclpy.init()

    # 🔹 跟随节点（直接发布 /cmd_vel）
    follower_node = RobotFollowerNode()

    cap = cv2.VideoCapture(2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 启动Flask Web服务器线程
    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=55555, debug=False, use_reloader=False))
    flask_thread.daemon = True  # 当主线程结束时，子线程也会结束
    flask_thread.start()
    print("Flask Web服务器已启动，访问地址: http://<设备IP>:55555")

    vision = AsyncVision()
    vision.start()

    prev = time.time()

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.flip(frame, -1)
            vision.update_frame(frame)

            # 获取视觉结果
            bboxes, classes, pose_results = vision.get_results()

            # ⭐ 核心：调用 follower_node.update
            failure = follower_node.update(
                bboxes,
                classes,
                pose_results,
                frame.shape[1],
                frame.shape[0]
            )

            # ------------------ 可视化 ------------------
            if bboxes is not None:
                for (x1, y1, x2, y2) in bboxes:
                    cv2.rectangle(
                        frame,
                        (x1, y1),
                        (x2, y2),
                        (0, 0, 255),
                        2
                    )

            if pose_results and pose_results.pose_landmarks:
                PoseEstimator.draw(frame, pose_results)

                lm = pose_results.pose_landmarks.landmark
                lh, lk, la = lm[23], lm[25], lm[27]
                angle_knee = calculate_angle(
                    [lh.x, lh.y],
                    [lk.x, lk.y],
                    [la.x, la.y]
                )

                times.append(dt.datetime.now())
                angles.append(angle_knee)

            status = "Failure !!!" if failure else "All Good"
            color = (0, 0, 255) if failure else (0, 255, 0)
            cv2.putText(
                frame,
                status,
                (30, 60),
                cv2.FONT_HERSHEY_PLAIN,
                2,
                color,
                2
            )

            fps = 1 / max(1e-6, time.time() - prev)
            prev = time.time()
            cv2.putText(
                frame,
                f"FPS:{fps:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2
            )

            # 将处理后的帧放入队列，供Web服务器使用
            if not frame_queue.full():
                try:
                    frame_queue.put(frame.copy(), block=False)
                except queue.Full:
                    # 队列已满，跳过当前帧
                    pass

            # 注释掉cv2.imshow，使用Web界面替代
            # cv2.imshow("BlazePose Debug View", frame)
            # if cv2.waitKey(10) & 0xFF == ord('q'):
            #     break

            # ⭐ ROS2 回调处理（非常重要）
            rclpy.spin_once(follower_node, timeout_sec=0.0)

    finally:
        vision.stop()
        cap.release()
        # 不再需要cv2.destroyAllWindows()，因为我们使用Web界面替代了
        rclpy.shutdown()
        plt.show()

//测试新分支并提交内容

if __name__ == "__main__":
    main()
