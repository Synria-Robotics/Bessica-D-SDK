"""
Demo: Gripper control

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Features:
- Open/close gripper
- Control gripper to specific angle
- Wait for gripper motion completion
"""

import bessica_d_sdk
from bessica_d_sdk.utils.logger import logger
import time

def main(args):
    """Move gripper.
    
    :param args: Command line arguments containing port, baudrate, version
    """
    # Initialize robot instance
    robot = bessica_d_sdk.create_robot(
        port=args.port,
        baudrate=args.baudrate,
        robot_version=args.robot_version,
        debug_mode=False
    )

    try:
        # Connect to robot
        if not robot.connect():
            print("✗ Connection failed, please check serial port settings")
            return
        robot.set_gripper_target(arm=args.arm, command='open', wait_for_completion=False)
        time.sleep(2)
        robot.set_gripper_target(arm=args.arm, command='close', wait_for_completion=False)
        time.sleep(2)
        robot.set_gripper_target(arm=args.arm, value=50.0, wait_for_completion=False)
        time.sleep(2)
    
    except KeyboardInterrupt:
        print("\n✗ Processing interrupted")
    
    except Exception as e:
        import traceback
        traceback.print_exc()

    finally:
        robot.disconnect()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Gripper Control Demo")
    
    # Serial port settings
    parser.add_argument('--port', type=str, default="/dev/ttyUSB0", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--baudrate', type=int, default=1000000,  help="波特率 (默认: 1000000)")
    parser.add_argument('--robot_version', type=str, default="v1_0",  help="机械臂版本 (默认: v1_0)")
    parser.add_argument('--arm', type=str, default="both", choices=["left_arm", "right_arm", "both"], help="机械臂 (默认: both)")
    args = parser.parse_args()
    main(args)