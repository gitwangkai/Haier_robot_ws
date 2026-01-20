# BlazePoseRobot

简体中文说明（README）

## 项目简介
- BlazePoseRobot 是一个基于 YOLO (yolov8n.pt) 与 BlazePose 姿态估计的机器人视觉与跟随演示代码。项目集成了视觉检测、姿态估计、状态机跟随逻辑与机器人动作控制（通过 `motion` 模块），并包含用于控制电机的示例固件文件（`.ino`）。

## 主要功能
- 实时摄像头采集并进行目标检测（YOLO）和姿态估计（BlazePose）。
- 根据检测与姿态结果由 `follower` 模块决定跟随/纠偏/失败检测并驱动 `motion` 模块。
- 可视化调试窗口显示相机画面、检测框、姿态关键点、运行状态与 FPS；同时将膝关节角度绘制为时间序列曲线（Matplotlib）。

## 代码结构 & 每部分作用
- `main.py`：主入口，启动 ROS2 节点（`BlazePoseNode`）、摄像头循环、异步视觉线程 `AsyncVision`，以及显示与调试逻辑。
- `requirements.txt`：Python 运行所需第三方包清单（请通过 `pip install -r requirements.txt` 安装）。
- `yolov8n.pt`：YOLOv8 的轻量检测模型权重，用于目标检测。
- `vision/`：视觉相关代码
  - `async_vision.py`：异步摄像头/推理线程管理，接收帧并输出检测与姿态结果。
  - `yolo_detector.py`：封装 YOLO 检测逻辑（加载 `yolov8n.pt`）。
  - `pose_estimator.py`：BlazePose 或其他姿态估计器的封装与绘制辅助（关键点绘制）。
  - `vision_manager.py`：可选的高层管理器，用于协调多个视觉子模块（若存在）。
- `follower/`：跟随与状态机逻辑
  - `robot_follower.py`：从视觉结果计算是否“失败”或需移动，并调用 `motion` 接口。
  - `target_state_machine.py`：实现跟随/搜索/失稳等状态机逻辑。
- `motion/`：机器人动作抽象与控制
  - `robot_motion.py`：机器人运动控制接口（发布命令到电机或仿真）。
  - `mover_node.py`：若启用，可作为独立 ROS 节点来接收运动命令。
- `utils/geometry.py`：几何计算辅助函数（如 `calculate_angle`，用于膝角计算）。
- `runs/`：检测/推理产生的预测与运行记录目录。
- `.ino` 文件：`new_working_controller.ino` 与 `Working_Motor_Control.ino`，示例 Arduino 固件用于电机控制板（作为参考/部署固件）。
- `function.py`, `classes.txt`, `CONTRIBUTIONS.md`, `LICENSE`：项目文档/贡献/许可等辅助文件。

## 依赖 & 准备
1. 安装系统依赖（确保有 ROS2 与 OpenCV，可选 GPU 支持）
2. 在项目目录下安装 Python 依赖：

```bash
python3 -m pip install -r requirements.txt
```

3. 若在 ROS2 环境中运行，确保已经 source ROS2 与工作区环境，例如：

```bash
source /opt/ros/<your-ros2-distro>/setup.bash
source ~/Haier_robot_ws/install/local_setup.bash
```

## 运行
1. 直接运行（非 launch）：

```bash
python3 main.py
```

2. 注意事项：
- `main.py` 中使用 `cv2.VideoCapture(2)`（摄像头索引 `2`），根据你的摄像头实际索引修改为 `0` 或其他。
- 如果使用 ROS2 集成或要把 `motion` 发布到 ROS topic，请先确保 ROS 节点环境已正确 source 并且相关 topic 已就绪。

## 调试与可视化
- 程序会弹出 `BlazePose Debug View` 窗口，显示检测框、关键点与状态文本；按 `q` 退出。
- 运行结束后会展示 Matplotlib 的膝关节角度随时间曲线（`plt.show()`）。若希望非阻塞或在远程机器上查看，建议改为保存数据或使用 web 界面展示。

## 常见问题
- 若没有检测到目标：确认 `yolov8n.pt` 文件存在且 `yolo_detector` 能正确加载模型。
- 如果帧率低：检查模型是否在 CPU 上推理，考虑使用 GPU 或降低输入分辨率。

## 下一步 / Todo（简要）
- 在仓库根目录添加 `launch` 文件以便 ROS2 启动（把 `main.py` 或 `mover_node.py` 集成进 launch）。
- 为 `vision` 模块补充 mock 测试数据与单元测试。
- 将 Matplotlib 实时绘图改为非阻塞或使用 WebSocket/Flask 展示以便远程查看。
- 编写 README 中未覆盖的硬件接线与电机固件部署步骤。
- 性能优化：模型量化/使用 ONNX + TensorRT 或改用更轻量推理后端。

## 贡献
- 欢迎提交 PR 或在 `CONTRIBUTIONS.md` 中描述贡献流程。

---
文件位置：`/home/aidlux/Haier_robot_ws/src/BlazeposeRobot/README.md`
