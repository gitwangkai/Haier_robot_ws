#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
数据滤波工具 - 最终版：EMA + 死区抑制
"""
import json
import os
import sys
import copy

def combined_ema_deadzone_filter(input_filepath, smoothing_factor=0.3, deadzone_threshold=2):
    """
    对录制的JSON文件应用组合滤波器（EMA + 死区抑制）。
    :param input_filepath: 输入的JSON文件路径。
    :param smoothing_factor: 平滑因子 (alpha) for EMA. 0 < alpha <= 1. 值越小，平滑越强。
    :param deadzone_threshold: 死区阈值. 只有当变化大于此值时才记录。
    :return: 滤波后的动作数据列表。
    """
    try:
        with open(input_filepath, 'r', encoding='utf-8') as f:
            action_data = json.load(f)
    except Exception as e:
        print(f"Error reading file {input_filepath}: {e}")
        return None

    if not action_data:
        return []

    filtered_data = []
    # last_ema_value: for EMA calculation
    # last_accepted_value: for dead-zone logic
    last_ema_value = {'right_arm_angles': {}, 'left_arm_angles': {}}
    last_accepted_value = {'right_arm_angles': {}, 'left_arm_angles': {}}

    for frame in action_data:
        new_frame = copy.deepcopy(frame)
        
        for arm_key in ['right_arm_angles', 'left_arm_angles']:
            if arm_key in frame:
                # Make sure to process all servos that have appeared before, even if not in current frame
                all_servos = set(frame[arm_key].keys()) | set(last_accepted_value[arm_key].keys())
                
                for servo_id_str in all_servos:
                    # Get current raw position, or use last known good position if missing
                    pos = frame[arm_key].get(servo_id_str)
                    if pos is None:
                        if servo_id_str in last_accepted_value[arm_key]:
                            new_frame.setdefault(arm_key, {})[servo_id_str] = last_accepted_value[arm_key][servo_id_str]
                        continue

                    # --- Stage 1: EMA Smoothing ---
                    prev_ema = last_ema_value[arm_key].get(servo_id_str, pos)
                    current_ema = smoothing_factor * pos + (1 - smoothing_factor) * prev_ema
                    last_ema_value[arm_key][servo_id_str] = current_ema

                    # --- Stage 2: Dead-zone Suppression ---
                    prev_accepted = last_accepted_value[arm_key].get(servo_id_str, int(round(current_ema)))
                    
                    if abs(current_ema - prev_accepted) > deadzone_threshold:
                        # If change is significant, accept the new smoothed value
                        final_pos = int(round(current_ema))
                        last_accepted_value[arm_key][servo_id_str] = final_pos
                    else:
                        # If change is inside the dead-zone, ignore it and keep the last accepted value
                        final_pos = prev_accepted
                    
                    new_frame.setdefault(arm_key, {})[servo_id_str] = final_pos

        filtered_data.append(new_frame)

    return filtered_data

def main():
    if len(sys.argv) < 2:
        print("使用方法: python3 filter_tool.py <输入文件> [输出文件] [平滑因子] [死区阈值]")
        print("\n参数:")
        print("  平滑因子 (可选): 0-1之间的小数，越小越平滑。默认0.3。")
        print("  死区阈值 (可选): 整数，变化小于此值将被忽略。默认2。")
        print("\n示例:")
        print("  python3 filter_tool.py arm_data/my_action.json")
        print("  python3 filter_tool.py arm_data/my_action.json arm_data/my_action_filtered.json 0.2 3")
        return
    
    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None
    smoothing_factor = float(sys.argv[3]) if len(sys.argv) > 3 else 0.3
    deadzone_threshold = int(sys.argv[4]) if len(sys.argv) > 4 else 2

    if not os.path.exists(input_file):
        print(f"❌ 输入文件不存在: {input_file}")
        return

    if not output_file:
        base, ext = os.path.splitext(input_file)
        output_file = f"{base}_filtered{ext}"

    print(f"🔬 正在使用 平滑因子={smoothing_factor}, 死区阈值={deadzone_threshold} 对 {input_file} 进行组合滤波...")
    
    # 执行滤波
    filtered_data = combined_ema_deadzone_filter(input_file, smoothing_factor, deadzone_threshold)
    
    if filtered_data:
        try:
            with open(output_file, 'w', encoding='utf-8') as f:
                json.dump(filtered_data, f, ensure_ascii=False, indent=2)
            print(f"\n🎉 滤波完成！")
            print(f"📁 输出文件: {output_file}")
        except Exception as e:
            print(f"❌ 保存滤波文件失败: {e}")

if __name__ == "__main__":
    main()
