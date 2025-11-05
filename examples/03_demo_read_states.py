"""
Demo: Read and print robot state

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Features:
- Read joint angles (radians or degrees)
- Read end-effector pose
- Read gripper state
- Support single or continuous printing
"""

from bessica_d_sdk.api import SynriaBessicaRobotAPI
import logging
from bessica_d_sdk.hardware import ServoDriver
import time
logger = logging.getLogger("demo_read_states")

def main(args):
    """Read and print robot state.
    
    :param args: Command line arguments containing port, baudrate, version, and gripper_type
    """

    # Initialize robot instance
    robot = SynriaBessicaRobotAPI(ServoDriver(port=args.port, baudrate=args.baudrate, debug_mode=False))

    try:
            # Connect to robot
        if not robot.connect():
            print("✗ Connection failed, please check serial port settings")
            return
        joint_angles = robot.get_joints(arm=args.arm)
        logger.info(f"关节角度: {joint_angles}")
        time.sleep(0.3)
        pose = robot.get_pose(arm=args.arm)
        logger.info(f"末端执行器位姿: {pose}")
        gripper = robot.get_gripper(arm=args.arm)
        logger.info(f"夹爪状态: {gripper}")
        
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        robot.disconnect()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Read robot states")
    
    # Robot configuration
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--baudrate', type=int, default=1000000,  help="波特率 (默认: 1000000)")
    parser.add_argument('--arm', type=str, default="both", choices=["left_arm", "right_arm", "both"], help="机械臂 (默认: both)")
    args = parser.parse_args()

    main(args)