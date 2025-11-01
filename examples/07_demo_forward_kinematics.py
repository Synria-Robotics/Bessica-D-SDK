"""
Demo: Forward kinematics

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Features:
- Read current joint angles
- Calculate end-effector pose
- Display position, rotation matrix, Euler angles, quaternion
"""

from bessica_d_sdk.api import SynriaBessicaRobotAPI
import logging
import numpy as np
from bessica_d_sdk.hardware import ServoDriver
logger = logging.getLogger("demo_forward_kinematics")
from robocore.utils.beauty_logger import beauty_print_array, beauty_print

def main(cmd_args):
    """Demonstrate forward kinematics. make example as left_arm

    :param cmd_args: Command line arguments
    """
    robot = SynriaBessicaRobotAPI(ServoDriver(port=args.port, baudrate=args.baudrate, debug_mode=False))

    if not robot.connect():
        return

    robot_model = robot.robot_model
    robot_model.summary(show_chain=True)
    robot_model.print_tree(show_fixed=True)

    pose_info = robot.get_pose(arm="left_arm")
    if pose_info is None:
        beauty_print("获取位姿失败")
        return

    position_fk = pose_info['position']
    rotation_fk = pose_info['rotation']
    euler_fk = pose_info['euler_xyz']
    quat_fk = pose_info['quaternion_xyzw']
    T_fk = pose_info['transform']

    beauty_print("End-Effector Position (m):")
    print(f"  p = {beauty_print_array(position_fk)}")
    beauty_print("End-Effector Orientation (Euler XYZ, radians):")
    print(f"  rpy = {beauty_print_array(euler_fk)}")
    beauty_print("End-Effector Orientation (Euler XYZ, degrees):")
    print(f"  rpy = {beauty_print_array(np.rad2deg(euler_fk))}")
    beauty_print("End-Effector Orientation (Quaternion xyzw):")
    print(f"  quat = {beauty_print_array(quat_fk, precision=6)}")
    # Add note about quaternion sign ambiguity
    quat_neg = -quat_fk
    print("  Note: q and -q represent the same rotation")
    print(f"  -quat = {beauty_print_array(quat_neg, precision=6)} (equivalent)")
    beauty_print("Rotation Matrix:")
    print(beauty_print_array(rotation_fk, precision=6))
    beauty_print("Homogeneous Transformation Matrix:")
    print(beauty_print_array(T_fk, precision=6))

    robot.disconnect()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Forward kinematics demo")
    
    # Robot configuration
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--baudrate', type=int, default=1000000,  help="波特率 (默认: 1000000)")
    args = parser.parse_args()
    main(args)