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
Demo: Robot zero calibration

Warning: 
- Ensure no obstacles around the robot arm before calibration
- When torque is disabled, manually support the robot arm
"""

import bessica_d_sdk
from bessica_d_sdk.utils.logger import logger

def main(args):
    """Execute robot zero calibration.
    
    :param args: Command line arguments containing port, baudrate, version
    """
    # Initialize robot instance
    robot = bessica_d_sdk.create_robot(port=args.port)
    logger.warning("此操作不可逆，将更改出厂零点位置，请谨慎操作")
    logger.warning("Irreversible action, please proceed with caution")
    robot.set_zero(arm=args.arm)
    robot.disconnect()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Robot zero calibration program")
    
    # Robot configuration
    parser.add_argument('--port', type=str, default="", help="Serial port (e.g., /dev/ttyACM0 or COM3)")
    parser.add_argument('--arm', type=str, default='right', choices=['both', 'left', 'right'],
                       help="Arm to control: 'both', 'left', or 'right' (default: both)")
    args = parser.parse_args()

    main(args)