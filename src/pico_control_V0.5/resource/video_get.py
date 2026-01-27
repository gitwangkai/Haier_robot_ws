#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
摄像头视频获取脚本
用于测试和配置摄像头设备
"""

import cv2
import sys
import time

def test_camera(device_id):
    """测试指定摄像头设备"""
    print(f"正在测试摄像头设备: /dev/video{device_id}")

    try:
        # 尝试打开摄像头
        cap = cv2.VideoCapture(device_id)

        if not cap.isOpened():
            print(f"❌ 无法打开摄像头 /dev/video{device_id}")
            return False

        # 获取摄像头信息
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)

        print("✅ 摄像头信息:")
        print(f"   分辨率: {width}x{height}")
        print(f"   帧率: {fps} FPS")

        # 读取一帧测试
        ret, frame = cap.read()
        if ret:
            print("✅ 成功读取视频帧")
            print(f"   帧大小: {frame.shape}")
        else:
            print("❌ 无法读取视频帧")

        cap.release()
        return True

    except Exception as e:
        print(f"❌ 测试摄像头时出错: {e}")
        return False

def list_available_cameras():
    """列出所有可用的摄像头设备"""
    print("🔍 扫描可用摄像头设备...")
    available_cameras = []

    # 通常摄像头设备编号从0开始
    for i in range(20):  # 检查前20个设备
        try:
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = cap.get(cv2.CAP_PROP_FPS)

                camera_info = {
                    'device_id': i,
                    'width': width,
                    'height': height,
                    'fps': fps
                }
                available_cameras.append(camera_info)

                print("✅ 发现摄像头:")
                print(f"   设备ID: {i} (/dev/video{i})")
                print(f"   分辨率: {width}x{height}")
                print(f"   帧率: {fps} FPS")
                print()

            cap.release()
        except Exception as e:
            print(f"⚠️  检查设备 {i} 时出错: {e}")
            pass

    return available_cameras

def main():
    print("🎥 摄像头测试工具")
    print("=" * 50)

    if len(sys.argv) > 1:
        # 如果提供了设备ID参数
        try:
            device_id = int(sys.argv[1])
            test_camera(device_id)
        except ValueError:
            print("❌ 无效的设备ID，请输入数字")
    else:
        # 扫描所有可用摄像头
        cameras = list_available_cameras()

        if not cameras:
            print("❌ 未发现任何可用的摄像头设备")
            return

        print(f"\n📋 共发现 {len(cameras)} 个摄像头设备")
        print("可用设备ID:", [cam['device_id'] for cam in cameras])

        # 询问用户要测试哪个设备
        while True:
            try:
                choice = input("\n请输入要测试的设备ID (或按Enter退出): ").strip()
                if not choice:
                    break

                device_id = int(choice)
                camera_exists = any(cam['device_id'] == device_id for cam in cameras)

                if camera_exists:
                    print(f"\n🔍 正在测试设备ID: {device_id}")
                    test_camera(device_id)
                else:
                    print(f"❌ 设备ID {device_id} 不存在")
                    print("可用设备ID:", [cam['device_id'] for cam in cameras])

            except ValueError:
                print("❌ 请输入有效的数字")
                print("可用设备ID:", [cam['device_id'] for cam in cameras])
            except KeyboardInterrupt:
                print("\n👋 退出程序")
                break

if __name__ == "__main__":
    main()
