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

"""Trajectory Executor and Drag Teaching for Dual-Arm Robot Control

This module provides specialized executors for joint space and Cartesian space trajectories
and drag teaching functionality for dual-arm robots (Bessica).
"""

from .trajectory_executor import JointTrajectoryExecutor, CartesianTrajectoryExecutor
from .drag_teaching import SimpleDragTeaching, list_available_motions, print_available_motions

__all__ = [
    'JointTrajectoryExecutor',
    'CartesianTrajectoryExecutor',
    'SimpleDragTeaching',
    'list_available_motions',
    'print_available_motions',
]

