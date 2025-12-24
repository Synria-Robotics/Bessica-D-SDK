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
        baudrate=args.baudrate,
        robot_version=args.robot_version,
        debug_mode=False
    )

    try:
        # Connect to robot
        if not robot.connect():
            print("✗ Connection failed, please check serial port settings")
            return
        
        logger.info("=" * 70)
        logger.info(f"Reading robot state for: {args.arm.upper()}")
        logger.info("=" * 70)
        
        # Read all data first
        joint_angles = robot.get_joints(arm=args.arm)
        if joint_angles is None:
            logger.error("Failed to read joint angles")
            return
        
        time.sleep(0.3)
        
        pose = robot.get_pose(arm=args.arm)
        if pose is None:
            logger.error("Failed to read end-effector pose")
            return
        
        gripper = robot.get_gripper(arm=args.arm)
        
        # Display information grouped by arm (left first, then right)
        if args.arm == "both":
            # Dual arm mode: print all left arm info, then all right arm info
            if isinstance(joint_angles, list) and len(joint_angles) == 2:
                left_joints = joint_angles[0]
                right_joints = joint_angles[1]
                
                quaternion = pose['quaternion_xyzw']
                position = pose['position']
                
                # Left Arm Information
                logger.info(f"Left Arm 关节角度 (rad): {np.round(left_joints, 3).tolist()}")
                if isinstance(position, list) and len(position) == 2 and \
                   isinstance(quaternion, list) and len(quaternion) == 2:
                    pos_left = np.array(position[0])
                    quat_left = np.array(quaternion[0])
                    logger.info(f"Left Arm 位置(xyz /m): {np.round(pos_left, 3).tolist()}, "
                              f"四元数(qx, qy, qz, qw): {np.round(quat_left, 3).tolist()}")
                if gripper is not None and isinstance(gripper, tuple) and len(gripper) == 2:
                    logger.info(f"Left Arm 夹爪状态 (deg): {gripper[0]:.2f}")
                
                # Right Arm Information
                logger.info(f"Right Arm 关节角度 (rad): {np.round(right_joints, 3).tolist()}")
                if isinstance(position, list) and len(position) == 2 and \
                   isinstance(quaternion, list) and len(quaternion) == 2:
                    pos_right = np.array(position[1])
                    quat_right = np.array(quaternion[1])
                    logger.info(f"Right Arm 位置(xyz /m): {np.round(pos_right, 3).tolist()}, "
                              f"四元数(qx, qy, qz, qw): {np.round(quat_right, 3).tolist()}")
                if gripper is not None and isinstance(gripper, tuple) and len(gripper) == 2:
                    logger.info(f"Right Arm 夹爪状态 (deg): {gripper[1]:.2f}")
            else:
                logger.warning(f"Unexpected joint angles format for both arms: {type(joint_angles)}")
        else:
            # Single arm mode: print all information for the specified arm
            if isinstance(joint_angles, list) and len(joint_angles) == 7:
                logger.info(f"{args.arm.upper()} 关节角度 (rad): {np.round(joint_angles, 3).tolist()}")
            else:
                logger.warning(f"Unexpected joint angles format for {args.arm}: {type(joint_angles)}")
            
            quaternion = pose['quaternion_xyzw']
            position = pose['position']
            pos = np.array(position)
            quat = np.array(quaternion)
            logger.info(f"{args.arm.upper()} 位置(xyz /m): {np.round(pos, 3).tolist()}, "
                      f"四元数(qx, qy, qz, qw): {np.round(quat, 3).tolist()}")
            
            if gripper is not None:
                logger.info(f"{args.arm.upper()} 夹爪状态 (deg): {gripper:.2f}")
            else:
                logger.warning("Failed to read gripper state")
        
        logger.info("=" * 70)
        
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
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--baudrate', type=int, default=1000000,  help="波特率 (默认: 1000000)")
    parser.add_argument('--robot_version', type=str, default="v1_0",  help="机械臂版本 (默认: v1_0)")
    parser.add_argument('--arm', type=str, default="both", choices=["left_arm", "right_arm", "both"], help="机械臂 (默认: both)")
    args = parser.parse_args()

    main(args)