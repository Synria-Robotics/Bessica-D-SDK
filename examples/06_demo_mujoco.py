"""
Demo: Mujoco

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Features:
- Mujoco simulation
- Mujoco control
- Mujoco display
"""

import bessica_d_sdk
from bessica_d_sdk.utils.logger import logger
import mujoco
import mujoco.viewer
import numpy as np
from robocore.utils.control_utils import InteractiveDualArmIK
from synriard import get_model_path
from robocore.utils.beauty_logger import beauty_print_array, beauty_print

mjcf_path = get_model_path("Bessica_D", version="v1_0", variant="covered_interactive", model_format="mjcf")
# End-effector links
left_end = "left_arm_link7"
right_end = "right_arm_link7"
    
# Create interactive IK controller
controller = InteractiveDualArmIK(mjcf_path, left_end, right_end)

controller.run(mode='independent')

def main(args):
    """Demonstrate mujoco.
    
    :param args: Command line arguments
    """
    logger.info("=== Mujoco demo ===")
    
    robot = bessica_d_sdk.create_robot(
        port=args.port,
        baudrate=args.baudrate,
        robot_version=args.robot_version,
        debug_mode=False
    )
    if not robot.connect():
        return

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Mujoco demo")
    
    # Robot configuration
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--baudrate', type=int, default=1000000,  help="波特率 (默认: 1000000)")
    parser.add_argument('--robot_version', type=str, default="v1_0",  help="机械臂版本 (默认: v1_0)")
    args = parser.parse_args()
    main(args)