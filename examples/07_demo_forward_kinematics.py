"""
Demo: Forward kinematics

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Features:
- Read current joint angles
- Calculate end-effector pose (single arm or bimanual)
- Display position, rotation matrix, Euler angles, quaternion
"""

import bessica_d_sdk
from bessica_d_sdk.utils.logger import logger
import numpy as np
import robocore as rc
from robocore.utils.beauty_logger import beauty_print_array, beauty_print

def main(args):
    """Demonstrate forward kinematics for single arm or bimanual system.

    :param args: Command line arguments
    """
    robot = bessica_d_sdk.create_robot(
        port=args.port,
        robot_version=args.robot_version,
        debug_mode=False,
        variant=args.variant,
        left_base_link=args.left_base_link,
        left_end_link=args.left_end_link,
        right_base_link=args.right_base_link,
        right_end_link=args.right_end_link,
    )
    rc.set_backend(args.backend)


    robot_model = robot.robot_model
    # robot_model.print_tree(show_fixed=True)

    # Get pose based on arm selection
    pose_info = robot.get_pose(arm=args.arm)
    if pose_info is None:
        beauty_print("获取位姿失败")
        return

    if args.arm == "both":
        # Bimanual mode - display both arms
        beauty_print("=== Left Arm ===")
        position_l = pose_info['position'][0]
        rotation_l = pose_info['rotation'][0]
        euler_l = pose_info['euler_xyz'][0]
        quat_l = pose_info['quaternion_xyzw'][0]
        T_l = pose_info['transform'][0]

        beauty_print("Left Arm End-Effector Position (m):")
        print(f"  p = {beauty_print_array(position_l)}")
        beauty_print("Left Arm End-Effector Orientation (Euler XYZ, radians):")
        print(f"  rpy = {beauty_print_array(euler_l)}")
        beauty_print("Left Arm End-Effector Orientation (Euler XYZ, degrees):")
        print(f"  rpy = {beauty_print_array(np.rad2deg(euler_l))}")
        beauty_print("Left Arm End-Effector Orientation (Quaternion xyzw):")
        print(f"  quat = {beauty_print_array(quat_l, precision=6)}")
        beauty_print("Left Arm Rotation Matrix:")
        print(beauty_print_array(rotation_l, precision=6))
        beauty_print("Left Arm Homogeneous Transformation Matrix:")
        print(beauty_print_array(T_l, precision=6))

        beauty_print("\n=== Right Arm ===")
        position_r = pose_info['position'][1]
        rotation_r = pose_info['rotation'][1]
        euler_r = pose_info['euler_xyz'][1]
        quat_r = pose_info['quaternion_xyzw'][1]
        T_r = pose_info['transform'][1]

        beauty_print("Right Arm End-Effector Position (m):")
        print(f"  p = {beauty_print_array(position_r)}")
        beauty_print("Right Arm End-Effector Orientation (Euler XYZ, radians):")
        print(f"  rpy = {beauty_print_array(euler_r)}")
        beauty_print("Right Arm End-Effector Orientation (Euler XYZ, degrees):")
        print(f"  rpy = {beauty_print_array(np.rad2deg(euler_r))}")
        beauty_print("Right Arm End-Effector Orientation (Quaternion xyzw):")
        print(f"  quat = {beauty_print_array(quat_r, precision=6)}")
        beauty_print("Right Arm Rotation Matrix:")
        print(beauty_print_array(rotation_r, precision=6))
        beauty_print("Right Arm Homogeneous Transformation Matrix:")
        print(beauty_print_array(T_r, precision=6))
    else:
        # Single arm mode
        position_fk = pose_info['position']
        rotation_fk = pose_info['rotation']
        euler_fk = pose_info['euler_xyz']
        quat_fk = pose_info['quaternion_xyzw']
        T_fk = pose_info['transform']

        beauty_print(f"=== {args.arm.upper()} Arm ===")
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
    parser.add_argument('--robot_version', type=str, default="v1_1",  help="机械臂版本 (默认: v1_1)")
    parser.add_argument('--backend', type=str, default='numpy',
                        choices=['numpy', 'torch'],
                        help='计算后端 (默认: numpy)')
    parser.add_argument('--arm', type=str, default='both',
                        choices=['left', 'right', 'both'],
                        help='要查询的机械臂 (默认: both)')
    parser.add_argument('--variant', type=str, default='skeleton',
                        help='模型变体 (默认: skeleton)')
    parser.add_argument('--left-base-link', type=str, default='base_link',
                        help='左臂基座链接名称 (默认: base_link)')
    parser.add_argument('--left-end-link', type=str, default='left_arm_link7',
                        help='左臂末端执行器链接名称 (默认: left_tool0)')
    parser.add_argument('--right-base-link', type=str, default='base_link',
                        help='右臂基座链接名称 (默认: base_link)')
    parser.add_argument('--right-end-link', type=str, default='right_arm_link7',
                        help='右臂末端执行器链接名称 (默认: right_tool0)')
    args = parser.parse_args()
    main(args)