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
Demo: Read and print robot state

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
    robot = bessica_d_sdk.create_robot(
        port=args.port,
        robot_version=args.robot_version
    )
    try:
        
        robot.print_state(continuous=True, output_format='deg')
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