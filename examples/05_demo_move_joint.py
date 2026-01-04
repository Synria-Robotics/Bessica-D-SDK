"""
Demo: Control robot to move to target joint positions

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Features:
- Support degree and radian input
- Automatic joint angle interpolation
- Adjustable motion speed
- Unified API for joint and gripper control
"""

import bessica_d_sdk
from bessica_d_sdk.utils.logger import logger
import time

def main(args):
    """Control robot joint movements.
    
    :param args: Command line arguments
    """
    # Initialize robot instance
    robot = bessica_d_sdk.create_robot(
        port=args.port,
        robot_version=args.robot_version,
        debug_mode=False,
        speed_deg_s=args.speed_deg_s
    )   

    try:
        # Connect to robot
        if not robot.connect():
            print("✗ Connection failed, please check serial port settings")
            return

        # Example joint targets in degrees (7 DOF per arm)
        left_angles_deg = [20, 20, 20, 20, 20, 20, 20]
        # right_angles_deg = [0, 0, 0, 0, 0, 0, 40]
        right_angles_deg = [20, 20, 20, 20, 20, 20, 20]

        # Move to home first
        robot.set_home(arm="both")
        time.sleep(1)

        # # --- Single-arm control examples ---
        # # Move right arm only
        # robot.set_robot_state(
        #     target_joints=right_angles_deg,
        #     arm="right",
        #     joint_format="deg",
        #     speed_deg_s=args.speed_deg_s,
        #     wait_for_completion=False,
        # )
        # time.sleep(1)

        # # Move left arm only
        # robot.set_robot_state(
        #     target_joints=left_angles_deg,
        #     arm="left",
        #     joint_format="deg",
        #     speed_deg_s=args.speed_deg_s,
        #     wait_for_completion=False,
        # )
        # time.sleep(0.1)

        # robot.set_home(arm="both")
        # time.sleep(1)
        # # --- Dual-arm control example ---
        # # Both arms move simultaneously; list-of-two-lists for arm="both"
        # both_angles_deg = [left_angles_deg, right_angles_deg]
        # robot.set_robot_state(
        #     target_joints=both_angles_deg,
        #     arm="both",
        #     joint_format="deg",
        #     speed_deg_s=args.speed_deg_s,
        #     wait_for_completion=False,
        # )
        # time.sleep(1)
        # robot.set_home(arm="both")
        # time.sleep(0.1)
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        robot.disconnect()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Robot joint movement control")
    
    # Robot configuration
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--robot_version', type=str, default="v1_0",  help="机械臂版本 (默认: v1_0)")
    parser.add_argument('--arm', type=str, default="both", choices=["left", "right", "both"], help="机械臂 (默认: both)")
    parser.add_argument('--speed_deg_s', type=float, default=20.0,  help="运动速度 (度/秒, 默认: 20.0)")
    args = parser.parse_args()
    main(args)