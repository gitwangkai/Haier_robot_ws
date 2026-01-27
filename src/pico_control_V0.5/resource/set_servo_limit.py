#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
舵机行程参数设置工具
用于设置和读取舵机的角度限制范围

使用方法：
python3 set_servo_limit.py <舵机ID> <最小值> <最大值>

例如：
python3 set_servo_limit.py 1 200 800
"""

import sys
import time
from Board import *

def set_servo_angle_limit(servo_id, low_limit, high_limit):
    """
    设置舵机行程参数（角度限制）
    
    Args:
        servo_id: 舵机ID (1-254)
        low_limit: 最小位置值 (0-1000)
        high_limit: 最大位置值 (0-1000)
    """
    print(f"\n开始设置舵机 {servo_id} 的行程参数...")
    print(f"  最小值: {low_limit}")
    print(f"  最大值: {high_limit}")
    
    # 设置角度限制
    setBusServoAngleLimit(servo_id, low_limit, high_limit)
    time.sleep(0.1)
    
    print("✓ 设置完成\n")


def get_servo_angle_limit(servo_id):
    """
    读取舵机当前的行程参数
    
    Args:
        servo_id: 舵机ID
        
    Returns:
        tuple: (最小值, 最大值)
    """
    print(f"正在读取舵机 {servo_id} 的行程参数...")
    
    limits = getBusServoAngleLimit(servo_id)
    
    if limits is not None:
        low, high = limits
        print(f"✓ 当前设置:")
        print(f"  最小值: {low}")
        print(f"  最大值: {high}")
        print(f"  行程范围: {high - low}")
        return limits
    else:
        print("✗ 读取失败")
        return None


def test_servo_movement(servo_id, low_limit, high_limit):
    """
    测试舵机在设置的行程范围内移动
    
    Args:
        servo_id: 舵机ID
        low_limit: 最小位置
        high_limit: 最大位置
    """
    print(f"\n开始测试舵机 {servo_id} 的行程...")
    
    # 移动到最小位置
    print(f"  移动到最小位置 {low_limit}...")
    setBusServoPulse(servo_id, low_limit, 1000)
    time.sleep(1.5)
    
    # 移动到中间位置
    mid = (low_limit + high_limit) // 2
    print(f"  移动到中间位置 {mid}...")
    setBusServoPulse(servo_id, mid, 1000)
    time.sleep(1.5)
    
    # 移动到最大位置
    print(f"  移动到最大位置 {high_limit}...")
    setBusServoPulse(servo_id, high_limit, 1000)
    time.sleep(1.5)
    
    # 回到中间位置
    print(f"  回到中间位置 {mid}...")
    setBusServoPulse(servo_id, mid, 1000)
    time.sleep(1.5)
    
    print("✓ 测试完成\n")


def main():
    """主函数"""
    print("=" * 50)
    print("舵机行程参数设置工具")
    print("=" * 50)
    
    if len(sys.argv) < 2:
        print("\n使用方法:")
        print("  1. 设置行程参数:")
        print("     python3 set_servo_limit.py <ID> <最小值> <最大值>")
        print("     例如: python3 set_servo_limit.py 1 200 800")
        print("\n  2. 读取当前参数:")
        print("     python3 set_servo_limit.py <ID>")
        print("     例如: python3 set_servo_limit.py 1")
        print("\n  3. 设置并测试:")
        print("     python3 set_servo_limit.py <ID> <最小值> <最大值> --test")
        print("     例如: python3 set_servo_limit.py 1 200 800 --test")
        print("\n参数说明:")
        print("  ID: 舵机编号 (1-254)")
        print("  最小值/最大值: 位置范围 (0-1000)")
        print("  默认出厂设置通常是: 0-1000 (全行程)")
        print("\n常用设置示例:")
        print("  - 限制到中间 50% 行程: 250-750")
        print("  - 限制到中间 60% 行程: 200-800")
        print("  - 恢复全行程: 0-1000")
        return
    
    try:
        servo_id = int(sys.argv[1])
        
        # 只读取参数
        if len(sys.argv) == 2:
            get_servo_angle_limit(servo_id)
        
        # 设置参数
        elif len(sys.argv) >= 4:
            low_limit = int(sys.argv[2])
            high_limit = int(sys.argv[3])
            
            # 参数验证
            if not (0 <= low_limit <= 1000 and 0 <= high_limit <= 1000):
                print("✗ 错误: 位置值必须在 0-1000 之间")
                return
            
            if low_limit >= high_limit:
                print("✗ 错误: 最小值必须小于最大值")
                return
            
            # 先读取当前值
            print("\n当前参数:")
            get_servo_angle_limit(servo_id)
            
            # 设置新值
            set_servo_angle_limit(servo_id, low_limit, high_limit)
            
            # 确认设置
            print("设置后的参数:")
            get_servo_angle_limit(servo_id)
            
            # 是否测试
            if '--test' in sys.argv or '-t' in sys.argv:
                test_servo_movement(servo_id, low_limit, high_limit)
        
        else:
            print("✗ 参数错误，请查看使用说明")
            
    except ValueError as e:
        print(f"✗ 参数格式错误: {e}")
    except KeyboardInterrupt:
        print("\n\n⚠ 用户中断")
    except Exception as e:
        print(f"✗ 发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n程序结束")


if __name__ == "__main__":
    main()
