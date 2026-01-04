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
Hardware Layer - 硬件层

提供底层硬件驱动功能，包括串口通信、数据解析、硬件协议处理等。
"""

from .servo_driver import ServoDriver
from .serial_comm import SerialComm
from .data_parser import DataParser, JointState

ArmController = ServoDriver

__all__ = [
    "ServoDriver",
    "SerialComm", 
    "DataParser",
    "JointState",
    "ArmController"  # Backward compatibility
]