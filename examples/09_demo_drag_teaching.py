#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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
Demo: Drag teaching (Dual-Arm Support)

Features:
- Disable torque and drag robot arm(s) to record joint angles
- Support manual, auto, and replay-only modes
- Support single-arm (left/right) and dual-arm (both) modes
- Use different replay strategies based on recording mode

Usage examples:
# See what motions are available
python 09_demo_drag_teaching.py --list-motions

# Record new motions (single arm)
python 09_demo_drag_teaching.py --mode auto --save-motion my_demo --arm right
python 09_demo_drag_teaching.py --mode manual --save-motion key_points --arm left

# Record new motions (dual arm)
python 09_demo_drag_teaching.py --mode auto --save-motion dual_arm_demo --arm both

# Replay existing motions
python 09_demo_drag_teaching.py --mode replay_only --save-motion my_demo

# Get help
python 09_demo_drag_teaching.py --help
"""

import os
import argparse

import bessica_d_sdk
from bessica_d_sdk.execution.drag_teaching import SimpleDragTeaching
from bessica_d_sdk.execution.drag_teaching import print_available_motions, list_available_motions


def main(args):
    """Demonstrate drag teaching functionality with dual-arm support."""
    
    # If the user requests to list motions, display and exit
    if args.list_motions:
        print_available_motions()
        return
    
    # Validate parameters
    if args.mode in ['manual', 'auto'] and not args.save_motion:
        print(f"[错误] {args.mode} 模式需要指定 --save-motion 参数")
        print("使用 --help 查看帮助信息")
        return
    
    if args.mode == 'replay_only' and not args.save_motion:
        print("[错误] 回放模式需要指定 --save-motion 参数")
        print("使用 --list-motions 查看可用动作")
        return

    robot = bessica_d_sdk.create_robot(
        port=args.port,
        robot_version=args.robot_version,
    )

    drag_teaching = SimpleDragTeaching(args, robot, arm=args.arm)
    drag_teaching.run()
    
    robot.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="拖动示教 (支持单臂和双臂)", 
                                   formatter_class=argparse.RawDescriptionHelpFormatter,)
    # Robot configuration
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--robot_version', type=str, default="v1_1", help="机械臂版本 (默认: v1_1)")
    parser.add_argument('--arm', type=str, default='both', choices=['left', 'right', 'both'],
                       help="控制的手臂: 'left', 'right', 或 'both' (默认: both)")
    parser.add_argument('--speed_deg_s', type=int, default=15, help="关节运动速度 (单位: 度/秒，默认: 15，范围: 10-80度/秒)")

    parser.add_argument('--mode', choices=['manual', 'auto', 'replay_only'], default='auto',
                       help="模式: manual(手动插值) 或 auto(自动快速) 或 replay_only(仅回放)")
    parser.add_argument('--save-motion', default='my_demo', help="动作名称 (录制模式: 新动作名; 回放模式: 已有动作名)")
    parser.add_argument('--list-motions', action='store_true', help="列出所有可用的动作并退出")
    parser.add_argument('--sample_hz', type=float, default=50.0, help="自动模式采样频率 (Hz, 默认: 50.0)")
    
    args = parser.parse_args()
    main(args)
