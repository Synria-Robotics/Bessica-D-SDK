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
Demo: Gripper control using unified set_robot_state API

Features:
- Open/close gripper (value 0-1000, where 1000 is fully open)
- Control gripper to specific value
- Support single arm and dual arm control
- Unified API for joint and gripper control
"""

import bessica_d_sdk
from bessica_d_sdk.utils.logger import logger
import time

def main(args):
    """Move gripper.
    
    :param args: Command line arguments containing port, baudrate, version
    """
    # Initialize robot instance
    robot = bessica_d_sdk.create_robot(
        port=args.port,
        robot_version=args.robot_version,
        debug_mode=False
    )

    try:
        # Connect to robot
        if not robot.connect():
            print("✗ Connection failed, please check serial port settings")
            return
        
        # Open gripper (value 1000 = fully open)
        robot.set_robot_state(
            target_joints=None,  # Keep current joints
            gripper_value=1000,  # Fully open
            arm=args.arm,
            wait_for_completion=False,
        )
        time.sleep(2)
        
        # Close gripper (value 0 = fully closed)
        robot.set_robot_state(
            target_joints=None,  # Keep current joints
            gripper_value=0,  # Fully closed
            arm=args.arm,
            wait_for_completion=False,
        )
        time.sleep(2)
    
    except KeyboardInterrupt:
        print("\n✗ Processing interrupted")
    
    except Exception as e:
        import traceback
        traceback.print_exc()

    finally:
        robot.disconnect()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Gripper Control Demo")
    
    # Serial port settings
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyACM0 或 COM3)")
    parser.add_argument('--robot_version', type=str, default="v1_0",  help="机械臂版本 (默认: v1_0)")
    parser.add_argument('--arm', type=str, default="both", choices=["left", "right", "both"], help="机械臂 (默认: both)")
    args = parser.parse_args()
    main(args)