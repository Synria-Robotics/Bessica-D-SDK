"""
Demo: Read robot states

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0
"""

from bessica_d_sdk.api import SynriaBessicaRobotAPI
import logging
from bessica_d_sdk.hardware import ServoDriver
logger = logging.getLogger("demo_read_states")

def main(args):
    """Read and print robot states.

    :param args: Command line arguments containing port, baudrate, version, and gripper_type
    """
    # Initialize robot instance
    robot = SynriaBessicaRobotAPI(ServoDriver(port=args.port, baudrate=args.baudrate, debug_mode=False))

    try:
        # Connect to robot
        if not robot.connect():
            logger.error("Connection failed, please check serial port settings")
            return
        logger.info("Connection successful")
    except KeyboardInterrupt:
        logger.info("\nOperation interrupted by user")
    except Exception as e:
        logger.error(f"An unexpected error occurred: {e}")
    finally:
        robot.disconnect()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Read robot firmware version")

    # Robot configuration
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--baudrate', type=int, default=1000000,  help="波特率 (默认: 1000000)")

    args = parser.parse_args()

    main(args)