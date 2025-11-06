"""
Demo: Read robot states

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0
"""

import bessica_d_sdk
from bessica_d_sdk.utils.logger import logger

def main(args):
    """Read and print robot states.

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
    parser.add_argument('--robot_version', type=str, default="v1_0",  help="机械臂版本 (默认: v1_0)")

    args = parser.parse_args()

    main(args)