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
Demo: Drag teaching

Features:
- Record right arm joint angles
- Disable torque for manual dragging
- Record target joint angles after dragging
- Execute trajectory between recorded positions

Warning:
- Ensure no obstacles around the robot arm
- Manually support the robot arm when torque is disabled
"""

import bessica_d_sdk
import time
import numpy as np

def main(args):
    """Drag teaching mode demo."""
    robot = bessica_d_sdk.create_robot(
        port=args.port,
        baudrate=args.baudrate,
        robot_version=args.robot_version,
        debug_mode=False
    )

    if not robot.connect():
        print("✗ 连接失败，请检查串口设置")
        return

    # 记录起始位置（右臂）
    print("记录起始位置...")
    input("确保右臂在起始位置，按 Enter 继续...")
    joints_start = robot.get_joints(arm="right")  # [7_joints]
    start_deg = [np.rad2deg(a) for a in joints_start]

    # 关闭扭矩并拖拽
    input("按 Enter 关闭扭矩，拖拽到目标位置...")
    robot.torque_control(command="off", arm="right")
    input("拖拽完成，按 Enter 继续...")
    robot.torque_control(command="on", arm="right")
    time.sleep(1.0)

    # 记录目标位置（右臂）
    print("记录目标位置...")
    joints_goal = robot.get_joints(arm="right")
    goal_deg = [np.rad2deg(a) for a in joints_goal]

    robot.set_home(arm="right")
    # 执行轨迹
    input("按 Enter 执行轨迹...")
    
    robot.set_robot_state(target_joints=start_deg, arm="right", joint_format="deg", wait_for_completion=False)
    time.sleep(2)
    robot.set_robot_state(target_joints=goal_deg, arm="right", joint_format="deg", wait_for_completion=False)
    time.sleep(2)
    
    print("完成")

    robot.disconnect()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="拖拽示教模式")
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--baudrate', type=int, default=1000000, help="波特率 (默认: 1000000)")
    parser.add_argument('--robot_version', type=str, default="v1_0",  help="机械臂版本 (默认: v1_0)")
    args = parser.parse_args()
    main(args)
