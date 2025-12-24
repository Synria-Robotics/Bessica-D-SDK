"""
Demo: Control robot to move to target joint positions using move_to_joint_state

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Features:
- Support degree and radian input
- Automatic joint angle interpolation
- Adjustable motion speed
"""

import bessica_d_sdk
from bessica_d_sdk.utils.logger import logger
import time

def main(args):
    """Control robot joint movements.
    
    :param args: Command line arguments
    """
    # Initialize robot instance
    robot = bessica_d_sdk.create_robot(
        port=args.port,
        robot_version=args.robot_version,
        debug_mode=False,
        speed_deg_s=args.speed_deg_s
    )   

    try:
        # Connect to robot
        if not robot.connect():
            print("✗ Connection failed, please check serial port settings")
            return
        robot.set_speed(speed_deg_s=args.speed_deg_s)
        # right_angles = [20, 0, 0, 0, 0, 0, 9]
        left_angles = [20, 20, 20 ,20, 20, 20, 20]
        right_angles = [20, 20, 20 ,20, 20, 20, 20]
        # both_angles = [15, 25, 25, 15, 50, 25, 0]
        # home_angles = [0.0] * 7
        robot.set_home()
        time.sleep(5)
        # robot.set_joint_target(target_joints=right_angles, arm="right_arm")
        # robot.set_joint_target(target_joints=left_angles, arm="left_arm")
        # time.sleep(5)
        # #robot.set_home(arm="both")
        # robot.set_gripper_target(arm="both", command="close")
        # 您可自行输入目标关节角度，然后控制机械臂移动到目标位置
        # self_target=[]
        # robot.set_joint_target(target_joints=self_target, arm=args.arm)
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        robot.disconnect()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Robot joint movement control")
    
    # Robot configuration
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--robot_version', type=str, default="v1_0",  help="机械臂版本 (默认: v1_0)")
    parser.add_argument('--arm', type=str, default="both", choices=["left_arm", "right_arm", "both"], help="机械臂 (默认: both)")
    parser.add_argument('--speed_deg_s', type=float, default=40.0,  help="运动速度 (度/秒, 默认: 20.0)")
    args = parser.parse_args()
    main(args)