"""
Demo: Mujoco

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Features:
- Mujoco simulation
- Mujoco control
- Mujoco display
"""

from bessica_d_sdk.api import SynriaBessicaRobotAPI
import mujoco
import mujoco.viewer
import logging
import numpy as np
from bessica_d_sdk.hardware import ServoDriver
logger = logging.getLogger("demo_mujoco")
from robocore.utils.beauty_logger import beauty_print_array, beauty_print
# from robocore.bridge.sim.mujoco.interactive_dual_arm import InteractiveDualArmIK
# from robocore.utils.interactive_ik import InteractiveDualArmIK
from robocore.utils.control_utils import InteractiveDualArmIK
from robot_descriptions.mjcf.Bessica_D_v1_0 import Bessica_D_Covered
mjcf_path = Bessica_D_Covered.xml
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
    
    robot = SynriaBessicaRobotAPI(ServoDriver(port=args.port, baudrate=args.baudrate, debug_mode=False))
    if not robot.connect():
        return