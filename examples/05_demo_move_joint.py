"""
Demo: Control robot to move to target joint positions using move_to_joint_state

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Features:
- Support degree and radian input
- Automatic joint angle interpolation
- Adjustable motion speed
"""

from bessica_d_sdk.api import SynriaBessicaRobotAPI
import logging
import time
from bessica_d_sdk.hardware import ServoDriver
logger = logging.getLogger("demo_move_joint")

def main(args):
    """Control robot joint movements.
    
    :param args: Command line arguments
    """
    # Initialize robot instance
    robot = SynriaBessicaRobotAPI(ServoDriver(port=args.port, baudrate=args.baudrate, debug_mode=False))   

    try:
        # Connect to robot
        if not robot.connect():
            print("✗ Connection failed, please check serial port settings")
            return
        left_angles = [10, 20, 20, 10, 45, 20, 0]
        right_angles = [10, 20, 20, 10, 45, 20, 0]
        both_angles = [15, 25, 25, 15, 50, 25, 0]
        robot.set_home()
        robot.set_joint_target(target_joints=left_angles, arm="left_arm")
        time.sleep(2)
        robot.set_joint_target(target_joints=right_angles, arm="right_arm")
        time.sleep(2)
        robot.set_joint_target(target_joints=both_angles,target_joints_second=both_angles, arm="both")
        time.sleep(2)
        robot.set_home()
        time.sleep(2)
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
    parser.add_argument('--baudrate', type=int, default=1000000,  help="波特率 (默认: 1000000)")
    parser.add_argument('--arm', type=str, default="both", choices=["left_arm", "right_arm", "both"], help="机械臂 (默认: both)")
    args = parser.parse_args()
    main(args)