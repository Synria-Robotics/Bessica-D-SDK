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

import bessica_d_sdk
from bessica_d_sdk.utils.logger import logger
import time
import numpy as np
def main(args):
    """Read and print robot state.
    
    :param args: Command line arguments containing port, baudrate, version
    """
    # Initialize robot instance
    robot = bessica_d_sdk.create_robot(
        port=args.port,
        robot_version=args.robot_version
    )

    try:
        
        robot.print_state(continuous=True, output_format='deg')
        # robot.print_state(continuous=False, output_format='deg')

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
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyACM0 或 COM3)")
    parser.add_argument('--robot_version', type=str, default="v1_1",  help="机械臂版本 (默认: v1_0)")
    parser.add_argument('--arm', type=str, default="both", choices=["left", "right", "both"], help="机械臂 (默认: both)")
    args = parser.parse_args()

    main(args)