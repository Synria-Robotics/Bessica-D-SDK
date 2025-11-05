"""
Demo: Switch robot torque control

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Warning: 
- Ensure no obstacles around the robot arm before calibration
- When torque is disabled, manually support the robot arm
"""

from bessica_d_sdk.api import SynriaBessicaRobotAPI
import logging
from bessica_d_sdk.hardware import ServoDriver
logger = logging.getLogger("demo_torque_switch")

def main(args):
    """Switch robot torque control.

    :param args: Command line arguments containing port, baudrate, version, and gripper_type
    """
    # Initialize robot instance
    robot = SynriaBessicaRobotAPI(ServoDriver(port=args.port, baudrate=args.baudrate, debug_mode=False))

    try:
        # Connect to robot
        if not robot.connect():
            print("✗ Connection failed, please check serial port settings")
            return
        logger.info("Please manually hold the robot arm.")
        logger.info("请托住机械臂以免其突然掉落。")
        input("Press Enter to disable torque...")
        robot.torque_control('off',arm="left_arm")
        input("Press Enter to re-enable torque...")
        robot.torque_control('on',arm="left_arm")
        logger.info("Torque re-enabled.")
        
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        robot.disconnect()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Robot torque control")
    
    # Robot configuration
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--baudrate', type=int, default=1000000,  help="波特率 (默认: 1000000)")
    
    args = parser.parse_args()

    main(args)