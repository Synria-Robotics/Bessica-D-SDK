# Copyright (c) 2025 Synria Robotics Co., Ltd.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
#
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

"""
Demo: Control robot to move to target joint positions

Features:
- Support degree and radian input
- Automatic joint angle interpolation
- Adjustable motion speed
- Unified API for joint and gripper control
- Support single-arm (left/right) and dual-arm (both) control
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
        target_angles_deg = [20, 20, 20, 20, 20, 20, 20]

        # Move to home first
        print(f"Moving {args.arm} arm(s) to home position...")
        robot.set_home(arm=args.arm, speed_deg_s=args.speed_deg_s)
        time.sleep(1)

        # Move to target joint angles
        print(f"Moving {args.arm} arm(s) to target joint angles...")
        if args.arm == "both":
            # Dual-arm: provide list of two lists
            target_joints = [target_angles_deg, target_angles_deg]
        else:
            # Single-arm: provide single list
            target_joints = target_angles_deg

        robot.set_robot_state(
            target_joints=target_joints,
            arm=args.arm,
            joint_format="deg",
            speed_deg_s=args.speed_deg_s,
            wait_for_completion=True,
        )
        time.sleep(1)

        # Return to home
        print(f"Returning {args.arm} arm(s) to home position...")
        robot.set_home(arm=args.arm, speed_deg_s=args.speed_deg_s)
        time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n✗ Processing interrupted")
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
    parser.add_argument('--speed_deg_s', type=float, default=10.0,  help="运动速度 (度/秒, 默认: 20.0)")
    args = parser.parse_args()
    main(args)