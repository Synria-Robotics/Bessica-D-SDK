#!/usr/bin/env python3
# coding=utf-8

"""
机械臂运动控制示例
演示如何控制机械臂进行基本运动、力矩控制和运动示例。
"""

import os
import sys
import time
from typing import List, Union
import argparse


# 添加项目根目录到Python路径
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from bessica_d_sdk.controller import ArmController

def parse_args():
    parser = argparse.ArgumentParser(description="机械臂运动控制演示")
    parser.add_argument('--arm', type=str, choices=['left_arm', 'right_arm', 'both'], default='left_arm',
                        help='选择要控制的机械臂（left_arm, right_arm, both）')
    return parser.parse_args()

def control_move(controller, current_angles: Union[List[float],List[List[float]]], 
              target_angles: Union[List[float],List[List[float]]], arm: str='left_arm',
              steps: int = 120, delay: float = 0.03) -> None:
        
        """
        缓慢插值移动机械臂到目标角度（支持单臂或双臂）

        Args:
            current_angles: 
                - 单臂：长度为 7 的角度列表（单位：弧度）
                - 双臂：包含两个 7 维列表的二维结构 [[left], [right]]
            target_angles:
                - 同上，表示目标位置
            arm: 
                - 'left_arm' 或 'right_arm' 表示单臂控制
                - 'both' 表示双臂控制
            steps: 插值步数（越多越平滑）
            delay: 每步的延迟时间（单位：秒）
        """

        if arm == 'both':

            for joint_angles in [current_angles, target_angles]:
                if (not isinstance(joint_angles, list) or len(joint_angles) != 2 or
                        not all(isinstance(sublist, list) and len(sublist) == 7 for sublist in joint_angles)):
                    print("双臂模式下，joint_angles 必须是形如 [[left_arm], [right_arm]] 的二维列表，每个包含 7 个关节角度")
                    return False    
                
            for step in range(1, steps + 1):
                interp_angles = [
                    [  # 左臂插值
                        current + (target - current) * step / steps
                        for current, target in zip(current_angles[0], target_angles[0])
                    ],
                    [  # 右臂插值
                        current + (target - current) * step / steps
                        for current, target in zip(current_angles[1], target_angles[1])
                    ]
                ]
                result = controller.set_joint_angles(interp_angles, arm='both', wait_for_completion=False)
                if not result:
                    return False

                time.sleep(delay)

        elif arm in ['left_arm', 'right_arm']:
            if not isinstance(joint_angles, list) or len(joint_angles) != 7:
                print(f"{arm}：关节角度数量必须为7")
                return False
            
            for step in range(1, steps + 1):
                interp_angles = [ current + (target - current) * step / steps
                        for current, target in zip(current_angles, target_angles)]
                
                result = controller.set_joint_angles(interp_angles, arm=arm, wait_for_completion=False)
                if not result:
                    return False

                time.sleep(delay)

def single_arm_demo(arm: str='left_arm'):
    return 0

def main():
    """主函数"""
    args = parse_args()
    selected_arm = args.arm

    print("=== 机械臂运动控制示例 ===")
    
    # 创建控制器实例
    controller = ArmController(debug_mode=False)
    
    try:
        # 连接到机械臂
        if not controller.connect():
            print("无法连接到机械臂，请检查连接")
            return
            
        print("连接成功")
        
        # 初始化夹爪位置
        if selected_arm == 'both':
            print("初始化夹爪位置...")
            controller.set_gripper(angle_rad=[0 * controller.DEG_TO_RAD]*2, arm=selected_arm, wait_for_completion=True)
        elif selected_arm in ['left_arm', 'right_arm']:
            print("初始化夹爪位置...")
            controller.set_gripper(angle_rad=0 * controller.DEG_TO_RAD, arm=selected_arm, wait_for_completion=True)
        
        # 读取初始位置
        initial_angles = controller.read_joint_angles(arm='both')
        initial_angles_left = [round(angle * controller.RAD_TO_DEG, 2) for angle in initial_angles[0]]
        initial_angles_right = [round(angle * controller.RAD_TO_DEG, 2) for angle in initial_angles[1]]
        print(f"左臂关节角度(度): {initial_angles_left}")
        print(f"右臂关节角度(度): {initial_angles_right}")

        
        # 1. 演示关节控制 - 移动到零位置
        print("\n将双臂关节移动到零位置...")
        zero_angles = [[[0.0] * 7]*2] # 双臂七个关节设为0
        result = control_move(controller, 
                              arm='both',
                              current_angles=initial_angles, 
                              target_angles=zero_angles)
        
        print(f"移动到零位置结果: {result}")
        
        # 读取当前位置
        current_angles = controller.read_joint_angles(arm='both')
        current_angles_left = [round(angle * controller.RAD_TO_DEG, 2) for angle in current_angles[0]]
        current_angles_right = [round(angle * controller.RAD_TO_DEG, 2) for angle in current_angles[1]]
        print(f"左臂关节角度(度): {current_angles_left}")
        print(f"右臂关节角度(度): {current_angles_right}")
        
        # 2. 演示逐个关节移动
        print("\n演示逐个关节移动...")
        
        for i in range(7):
            # 移动当前关节到30度
            test_angles = [[[0.0] * 7]*2]
            test_angles[0][i] = 15 * controller.DEG_TO_RAD
            test_angles[1][i] = 15 * controller.DEG_TO_RAD
            
            print(f"移动关节{i+1}到30度...")
            control_move(controller,
                         current_angles=[current_angles_left,current_angles_right],
                         target_angles=test_angles)
            
            # 移动回零位置
            print(f"移动关节{i+1}回零位置...")
            control_move(controller, 
                         current_angles=test_angles, 
                         target_angles=zero_angles)
        
        # 演示夹爪控制
        print("\n演示夹爪控制...")
        print("打开夹爪...")
        controller.set_gripper(angle_rad=[100 * controller.DEG_TO_RAD]*2, arm='both', wait_for_completion=True)
        print("关闭夹爪...")
        controller.set_gripper(angle_rad=[0 * controller.DEG_TO_RAD]*2, arm='both', wait_for_completion=True)
        
        # 读取双臂当前位置
        current_angles = controller.read_joint_angles(arm='both')
        current_angles_left = [round(angle * controller.RAD_TO_DEG, 2) for angle in current_angles[0]]
        current_angles_right = [round(angle * controller.RAD_TO_DEG, 2) for angle in current_angles[1]]
        print(f"左臂关节角度(度): {current_angles_left}")
        print(f"右臂关节角度(度): {current_angles_right}")

        # 回到零位置
        print("\n回到零位置...")
        control_move(controller, current_angles=current_angles, target_angles=zero_angles)
        print("\n演示完成!")
        
    except KeyboardInterrupt:
        print("\n\n程序已停止")
    finally:
        # 断开连接
        controller.disconnect()
        print("已断开连接")

if __name__ == "__main__":
    main()