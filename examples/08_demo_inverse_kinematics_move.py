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


"""Bimanual Inverse Kinematics 3D Movement Demo

Move the end-effector(s) through 6 target poses around the current target pose:
    - ±0.10 m in X (front / back)
    - ±0.10 m in Y (left / right)
    - ±0.10 m in Z (up / down)

This yields 6 target poses (front, back, left, right, up, down),
each held for about 1 second when executed.
"""

import time

import bessica_d_sdk
from bessica_d_sdk.utils.logger import logger
import numpy as np
import robocore as rc
from robocore.utils.beauty_logger import beauty_print_array, beauty_print


def ik_solve(args, robot, target_left=None, target_right=None):
    """Solve IK for given target pose(s) and optionally execute motion."""
    # Fallback to CLI targets if per-call targets are not provided
    if target_left is None:
        target_left = args.target_left
    if target_right is None and args.arm == "both":
        target_right = args.target_right

    # Solve IK based on arm selection
    if args.arm == "both":
        ik_result = robot.set_pose_target(
            target_pose1=target_left,
            target_pose2=target_right,
            arm="both",
            speed_deg_s=args.speed_deg_s,
            execute=args.execute
        )
        
        print("\n" + "=" * 60)
        beauty_print("IK求解结果 (双臂):")
        print(f"  成功: {ik_result.get('success', False)}")
        if ik_result.get('success', False):
            print(f"  左臂成功: {ik_result.get('success_left', False)}")
            print(f"  右臂成功: {ik_result.get('success_right', False)}")
            if 'res_left' in ik_result and ik_result['res_left']:
                res_left = ik_result['res_left']
                if isinstance(res_left, list) and len(res_left) > 0:
                    res_left = res_left[0]
                print(f"  左臂迭代次数: {res_left.get('iters', 'N/A')}")
                print(f"  左臂位置误差: {res_left.get('pos_err', float('inf')):.6e} m")
                print(f"  左臂姿态误差: {res_left.get('ori_err', float('inf')):.6e} rad")
            if 'res_right' in ik_result and ik_result['res_right']:
                res_right = ik_result['res_right']
                if isinstance(res_right, list) and len(res_right) > 0:
                    res_right = res_right[0]
                print(f"  右臂迭代次数: {res_right.get('iters', 'N/A')}")
                print(f"  右臂位置误差: {res_right.get('pos_err', float('inf')):.6e} m")
                print(f"  右臂姿态误差: {res_right.get('ori_err', float('inf')):.6e} rad")
            beauty_print("左臂关节角度 (弧度):")
            # ik_result.get('q_left', [])[3] = -ik_result.get('q_left', [])[3]
            print(f"  {beauty_print_array(ik_result.get('q_left', []))}")
            beauty_print("右臂关节角度 (弧度):")
            print(f"  {beauty_print_array(ik_result.get('q_right', []))}")
            if ik_result.get('motion_executed', False):
                beauty_print("✓ 机械臂已移动到目标位置")
            else:
                beauty_print("(未执行移动)")
        else:
            print(f"  错误信息: {ik_result.get('message', '未知错误')}")
        print("=" * 60 + "\n")
    else:
        # Single arm: use left or right target only
        if args.arm == "left":
            single_target = target_left or args.target_left
        else:
            single_target = target_right or args.target_right

        ik_result = robot.set_pose_target(
            target_pose1=single_target,
            arm=args.arm,
            speed_deg_s=args.speed_deg_s,
            execute=args.execute
        )
        
        print("\n" + "=" * 60)
        beauty_print(f"IK求解结果 ({args.arm}):")
        print(f"  成功: {ik_result.get('success', False)}")
        if ik_result.get('success', False):
            print(f"  迭代次数: {ik_result.get('iters', 'N/A')}")
            print(f"  位置误差: {ik_result.get('pos_err', float('inf')):.6e} m")
            print(f"  姿态误差: {ik_result.get('ori_err', float('inf')):.6e} rad")
            beauty_print("关节角度 (弧度):")
            print(f"  {beauty_print_array(ik_result.get('q', []))}")
            beauty_print("关节角度 (角度):")
            print(f"  {beauty_print_array(np.rad2deg(ik_result.get('q', [])))}")

        else:
            print(f"  错误信息: {ik_result.get('message', '未知错误')}")
        print("=" * 60 + "\n")



def main(args):
    """Demonstrate inverse kinematics around a square of target poses.

    :param args: Command line arguments
    """
    robot = bessica_d_sdk.create_robot(
        port=args.port,
        robot_version=args.robot_version,
        debug_mode=False,
        variant=args.variant,
        left_base_link=args.left_base_link,
        left_end_link=args.left_end_link,
        right_base_link=args.right_base_link,
        right_end_link=args.right_end_link,
    )
    rc.set_backend(args.backend)

    # Base target poses from arguments (position [0:3], quaternion [3:7])
    base_left = np.array(args.target_left, dtype=float)
    base_right = np.array(args.target_right, dtype=float)

    # Offsets (m) along X, Y, Z axes around the base target
    # 6 targets: front, back, left, right, up, down
    delta = 0.10  # 10 cm
    offsets_xyz = [
        np.array([+delta, 0.0, 0.0]),  # front (+X)
        np.array([-delta, 0.0, 0.0]),  # back (-X)
        np.array([0.0, +delta, 0.0]),  # left (+Y)
        np.array([0.0, -delta, 0.0]),  # right (-Y)
        np.array([0.0, 0.0, +delta]),  # up (+Z)
        np.array([0.0, 0.0, -delta]),  # down (-Z)
    ]
    
    direction_names = ["front (+X)", "back (-X)", "left (+Y)", "right (-Y)", "up (+Z)", "down (-Z)"]

    beauty_print("Starting IK 3D movement demo around target pose", type="module")

    for i, (offset, direction) in enumerate(zip(offsets_xyz, direction_names), start=1):
        beauty_print(f"Target {i}/6: {direction}, offset = {offset.tolist()} m")

        # Build new target poses by offsetting position only (keep orientation)
        if args.arm == "both":
            tl = base_left.copy()
            tr = base_right.copy()
            tl[:3] += offset
            tr[:3] += offset

            ik_solve(args, robot, tl.tolist(), tr.tolist())
        else:
            # Single arm: offset chosen arm only
            if args.arm == "left":
                tl = base_left.copy()
                tl[:3] += offset
                ik_solve(args, robot, tl.tolist(), None)
            elif args.arm == "right":
                tr = base_right.copy()
                tr[:3] += offset
                ik_solve(args, robot, None, tr.tolist())

        # Hold each pose for about 1 second if motion is executed
        if args.execute:
            time.sleep(2.0)



if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Inverse kinematics square demo")
    
    # Robot configuration
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--robot_version', type=str, default="v1_1",  help="机械臂版本 (默认: v1_1)")

    parser.add_argument('--arm', type=str, default='both',
                        choices=['left', 'right', 'both'],
                        help='要查询的机械臂 (默认: both)')
    parser.add_argument('--variant', type=str, default='skeleton',
                        help='模型变体 (默认: skeleton)')
    parser.add_argument('--left-base-link', type=str, default='base_link',
                        help='左臂基座链接名称 (默认: base_link)')
    parser.add_argument('--left-end-link', type=str, default='left_arm_link7',
                        help='左臂末端执行器链接名称 (默认: left_tool0)')
    parser.add_argument('--right-base-link', type=str, default='base_link',
                        help='右臂基座链接名称 (默认: base_link)')
    parser.add_argument('--right-end-link', type=str, default='right_arm_link7',
                        help='右臂末端执行器链接名称 (默认: right_tool0)')

    parser.add_argument('--target-left', type=float, nargs='+',
                        default=[0.213, 0.247, 0.276, -0.704, -0.017, -0.016, 0.71],
                        # 0.213, 0.247, 0.276
                        # default=[0.21704, +0.48451, +0.43961, -0.581520, -0.557668, -0.181004, +0.563984],
                        help='Target left end-effector pose as 7 floats (px, py, pz, qx, qy, qz, qw)')
    parser.add_argument('--target-right', type=float, nargs='+',
                        default=[0.21, -0.246, 0.273, 0.709, -0.013, 0.015, 0.705],
                        # default=[0.21707, -0.38424, +0.43838, 0.576014, -0.559190, +0.180929, +0.568136],
                        help='Target right end-effector pose as 7 floats (px, py, pz, qx, qy, qz, qw)')
    parser.add_argument('--coordination', type=str, default='indep',
                        choices=['indep', 'relative_pose', 'relative_pos', 'relative_ori', 'mirror'],
                        help='Coordination mode: indep (independent), relative_pose, relative_pos, relative_ori, mirror')
    parser.add_argument('--num-inits', type=int, default=1,
                        help='Number of initial guesses to try per target (default: 1)')
    parser.add_argument('--init-strategy', type=str, default='random',
                        choices=['zero', 'random', 'sobol', 'latin', 'center', 'uniform'],
                        help='Strategy for generating initial guesses (default: random)')
    parser.add_argument('--init-scale', type=float, default=1.0,
                        help='Scale factor for joint limits when generating guesses (0.0 to 1.0, default: 1.0)')
    parser.add_argument('--seed', type=int, default=None,
                        help='Random seed for reproducibility (default: None)')
    parser.add_argument('--backend', type=str, default='numpy', choices=['numpy', 'torch'],
                        help='Backend to use for computation (default: numpy, ignored - both are tested)')
    parser.add_argument('--speed-deg-s', type=float, default=20.0,
                        help='Joint motion speed in degrees per second (default: 10.0, range: 5-400)')
    parser.add_argument('--execute', action='store_true', help='执行移动到求解的位置')
    args = parser.parse_args()

    main(args)
