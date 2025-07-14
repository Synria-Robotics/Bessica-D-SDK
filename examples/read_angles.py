#!/usr/bin/env python3
# coding=utf-8

"""
关节角度读取示例
连续读取并显示机械臂的关节角度、夹爪角度和按钮状态。
"""

import os
import sys
import time
import math
import argparse

# 添加项目根目录到Python路径
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from bessica_d_sdk.controller import ArmController


def parse_args():
    parser = argparse.ArgumentParser(description="读取机械臂状态数据")
    parser.add_argument('--arm', type=str, choices=['left_arm', 'right_arm', 'both'], default='both',
                        help="选择要读取的机械臂：left_arm、right_arm 或 both（默认）")
    return parser.parse_args()

def main():
    """主函数"""
    args = parse_args()
    selected_arm = args.arm
    
    print(f"=== 机械臂数据读取示例（当前读取：{selected_arm}） ===")

    # 创建控制器实例 (可选参数: port="/dev/ttyUSB0", debug_mode=True) 
    controller = ArmController(debug_mode=False)
    
    try:
        # 连接到机械臂
        if not controller.connect():
            print("无法连接到机械臂，请检查连接")
            return
            
        print("连接成功，开始读取数据...")
        print("按 Ctrl+C 退出")
        print("-" * 50)

                
        # 持续读取数据
        while True:

            # 读取完整状态
            state = controller.read_joint_state()
            
            arms_to_read = ['left_arm', 'right_arm'] if selected_arm == 'both' else [selected_arm]

            # 转换并打印每个机械臂的数据
            for arm_name in arms_to_read:
                joint_state = state[arm_name]
                joint_state = state[arm_name]
                joint_angles_deg = [round(a * controller.RAD_TO_DEG, 2) for a in joint_state.angles]
                gripper_deg = round(joint_state.gripper * controller.RAD_TO_DEG, 2)

                print(f"【{arm_name}】关节角度(度): {joint_angles_deg}, 夹爪角度: {gripper_deg}°\n")

            print(f"状态更新线程运行中: {controller._thread_running}")
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\n\n程序已停止")
    finally:
        # 断开连接
        controller.disconnect()
        print("已断开连接")

if __name__ == "__main__":
    # python read_angles.py --arm left_arm
    # python read_angles.py --arm right_arm
    # python read_angles.py --arm both
    main()