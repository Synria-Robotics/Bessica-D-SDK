"""
Demo: Robot zero calibration

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Warning: 
- Ensure no obstacles around the robot arm before calibration
- When torque is disabled, manually support the robot arm
"""

import bessica_d_sdk
from bessica_d_sdk.utils.logger import logger

def main(args):
    """Execute robot zero calibration.
    
    :param args: Command line arguments containing port, baudrate, version
    """
    # Initialize robot instance
    robot = bessica_d_sdk.create_robot(port=args.port)
    logger.warning("此操作不可逆，将更改出厂零点位置，请谨慎操作")
    logger.warning("Irreversible action, please proceed with caution")
    robot.set_zero(arm=args.arm)
    robot.disconnect()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Robot zero calibration program")
    
    # Robot configuration
    parser.add_argument('--port', type=str, default="", help="Serial port (e.g., /dev/ttyACM0 or COM3)")
    parser.add_argument('--arm', type=str, default='both', choices=['both', 'left', 'right'],
                       help="Arm to control: 'both', 'left', or 'right' (default: both)")
    args = parser.parse_args()

    main(args)