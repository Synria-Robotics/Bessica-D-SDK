"""
Demo: Trajectory planning and execution

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Features:
- Joint space trajectory planning (linear/cubic/quintic)
- Cartesian space linear trajectory planning
- Support single arm and dual arm modes
- Smooth motion with configurable duration and interpolation points
"""

import bessica_d_sdk
from bessica_d_sdk.utils.logger import logger
import time
import numpy as np

def main(args):
    """Demonstrate trajectory planning and execution.
    
    :param args: Command line arguments
    """
    # Initialize robot instance
    robot = bessica_d_sdk.create_robot(
        port=args.port,
        baudrate=args.baudrate,
        robot_version=args.robot_version,
        debug_mode=False,
        speed_deg_s=args.speed_deg_s,
    )
    
    try:
        
        print("=" * 60)
        print("轨迹控制演示")
        print("=" * 60)
        
        # 1. 先回到零位
        print("\n[1] 回到零位...")
        robot.set_home(arm="both")
        time.sleep(3)
        
        # 2. 单臂关节空间轨迹（cubic插值）
        print("\n[2] 单臂关节空间轨迹（cubic插值）...")
        print("   左臂移动到目标关节角度")
        target_left = [30, 35, 58, -45, -40, -35, 0]  # 度
        success = robot.move_joint_trajectory(
            q_end=target_left,
            arm="left",
            duration=3.0,
            method='cubic',
            num_points=100,
            joint_format='deg'
        )
        if success:
            print("   ✓ 左臂轨迹执行成功")
        else:
            print("   ✗ 左臂轨迹执行失败")
        time.sleep(2)
        
        # 3. 双臂关节空间轨迹（quintic插值）
        print("\n[3] 双臂关节空间轨迹（quintic插值）...")
        print("   双臂同步移动到目标关节角度")
        target_left = [15, 25, 35, 20, 50, 30, 0]  # 度
        target_right = [15, 25, 35, 20, 50, 30, 0]  # 度
        success = robot.move_joint_trajectory(
            q_end=[target_left, target_right],  # 二维数组
            arm="both",
            duration=4.0,
            method='quintic',
            num_points=150,
            joint_format='deg'
        )
        if success:
            print("   ✓ 双臂轨迹执行成功")
        else:
            print("   ✗ 双臂轨迹执行失败")
        time.sleep(2)
        
        # 4. 单臂关节空间轨迹（linear插值）
        print("\n[4] 单臂关节空间轨迹（linear插值）...")
        print("   右臂移动到目标关节角度")
        target_right = [-20, 30, 45, -30, -50, -25, 0]  # 度
        success = robot.move_joint_trajectory(
            q_end=target_right,
            arm="right",
            duration=2.5,
            method='linear',
            num_points=80,
            joint_format='deg'
        )
        if success:
            print("   ✓ 右臂轨迹执行成功")
        else:
            print("   ✗ 右臂轨迹执行失败")
        time.sleep(2)
        
        # 5. 单臂笛卡尔空间直线轨迹
        if robot.robot_model is not None:
            print("\n[5] 单臂笛卡尔空间直线轨迹...")
            print("   左臂沿直线移动到目标位姿")
            
            # 获取当前位姿
            current_pose = robot.get_pose(arm="left")
            if current_pose:
                # 目标位姿：在当前位置基础上，向前移动10cm，保持姿态
                target_pose = current_pose['output_to_ik'].copy()
                target_pose[0] += 0.1  # x方向增加10cm
                
                success = robot.move_cartesian_linear(
                    target_pose=target_pose,
                    arm="left",
                    duration=3.0,
                    num_points=100,
                    ik_method='dls'
                )
                if success:
                    print("   ✓ 左臂笛卡尔轨迹执行成功")
                else:
                    print("   ✗ 左臂笛卡尔轨迹执行失败")
                
                time.sleep(2)
                
                # 回到原位置
                print("   回到原位置...")
                success = robot.move_cartesian_linear(
                    target_pose=current_pose['output_to_ik'],
                    arm="left",
                    duration=3.0,
                    num_points=100,
                    ik_method='dls'
                )
                if success:
                    print("   ✓ 左臂回到原位置成功")
                else:
                    print("   ✗ 左臂回到原位置失败")
            else:
                print("   ✗ 无法获取当前位姿，跳过笛卡尔轨迹演示")
        else:
            print("\n[5] 跳过笛卡尔空间轨迹演示（robot_model 不可用）")
        
        time.sleep(2)
        
        # 6. 双臂笛卡尔空间直线轨迹
        if robot.robot_model is not None:
            print("\n[6] 双臂笛卡尔空间直线轨迹...")
            print("   双臂同步沿直线移动到目标位姿")
            
            # 获取当前位姿
            current_pose = robot.get_pose(arm="both")
            if current_pose:
                # 左臂目标位姿：向前移动5cm
                target_pose_left = list(current_pose['output_to_ik'][0])
                target_pose_left[0] += 0.05  # x方向增加5cm
                
                # 右臂目标位姿：向前移动5cm
                target_pose_right = list(current_pose['output_to_ik'][1])
                target_pose_right[0] += 0.05  # x方向增加5cm
                
                # 方式1：使用二维数组
                success = robot.move_cartesian_linear(
                    target_pose=[target_pose_left, target_pose_right],
                    arm="both",
                    duration=4.0,
                    num_points=120,
                    ik_method='dls'
                )
                if success:
                    print("   ✓ 双臂笛卡尔轨迹执行成功")
                else:
                    print("   ✗ 双臂笛卡尔轨迹执行失败")
                
                time.sleep(2)
                
                # 回到原位置（方式2：使用 target_pose_second_arm 参数）
                print("   回到原位置...")
                success = robot.move_cartesian_linear(
                    target_pose=current_pose['output_to_ik'][0],
                    target_pose_second_arm=current_pose['output_to_ik'][1],
                    arm="both",
                    duration=4.0,
                    num_points=120,
                    ik_method='dls'
                )
                if success:
                    print("   ✓ 双臂回到原位置成功")
                else:
                    print("   ✗ 双臂回到原位置失败")
            else:
                print("   ✗ 无法获取当前位姿，跳过双臂笛卡尔轨迹演示")
        else:
            print("\n[6] 跳过双臂笛卡尔空间轨迹演示（robot_model 不可用）")
        
        time.sleep(2)
        
        # 7. 回到零位
        print("\n[7] 回到零位...")
        robot.set_home(arm="both")
        time.sleep(2)
        
        print("\n" + "=" * 60)
        print("轨迹控制演示完成")
        print("=" * 60)
        
    except KeyboardInterrupt:
        print("\n✗ 操作被用户中断")
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        robot.disconnect()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Trajectory planning and execution demo")
    
    # Robot configuration
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--baudrate', type=int, default=1000000,  help="波特率 (默认: 1000000)")
    parser.add_argument('--robot_version', type=str, default="v1_0",  help="机械臂版本 (默认: v1_0)")
    parser.add_argument('--speed_deg_s', type=float, default=20.0,  help="运动速度 (度/秒, 默认: 20.0)")
    args = parser.parse_args()
    main(args)

