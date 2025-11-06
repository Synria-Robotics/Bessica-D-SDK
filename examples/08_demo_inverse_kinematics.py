"""
Demo: Inverse kinematics

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Features:
- Specify target end-effector pose
- Solve for joint angles
- Move to target position
"""

import bessica_d_sdk
from bessica_d_sdk.utils.logger import logger
import time
import numpy as np
from robocore.utils.beauty_logger import beauty_print_array, beauty_print

def main(args):
    """Demonstrate inverse kinematics. make example as left_arm

    :param args: Command line arguments
    """
    robot = bessica_d_sdk.create_robot(
        port=args.port,
        baudrate=args.baudrate,
        robot_version=args.robot_version,
        debug_mode=False
    )

    if not robot.connect():
        print("✗ 连接失败，请检查串口设置")
        return

    print("设置双臂所有关节角度")
    left_angles = [10, 20, 20, 10, 45, 20, 0]
    robot.set_joint_target(target_joints=left_angles, arm="left_arm")
    pose_target_l = robot.get_pose(arm="left_arm")['output_to_ik']
    print(f"左臂输出 to ik: {pose_target_l}")
    time.sleep(2)
    print("设置双臂回到零位")
    robot.set_home()
    time.sleep(2)
    robot.set_pose_target(target_pose=pose_target_l, arm="left_arm")
    time.sleep(2)
    robot.set_home()
    time.sleep(2)

    robot.disconnect()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Inverse kinematics demo")
    
    # Robot configuration
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--baudrate', type=int, default=1000000,  help="波特率 (默认: 1000000)")
    parser.add_argument('--robot_version', type=str, default="v1_0",  help="机械臂版本 (默认: v1_0)")
    args = parser.parse_args()
    main(args)
    