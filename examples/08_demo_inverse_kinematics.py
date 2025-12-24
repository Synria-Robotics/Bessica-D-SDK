"""Bimanual Inverse Kinematics Demo

Copyright (c) 2025 Synria Robotics Co., Ltd.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.

Author: Synria Robotics Team
Website: https://synriarobotics.ai
"""

import bessica_d_sdk
from bessica_d_sdk.utils.logger import logger
import numpy as np
import robocore as rc
from robocore.utils.beauty_logger import beauty_print_array, beauty_print




def main(args):
    """Demonstrate forward kinematics for single arm or bimanual system.

    :param args: Command line arguments
    """
    robot = bessica_d_sdk.create_robot(
        port=args.port,
        baudrate=args.baudrate,
        robot_version=args.robot_version,
        debug_mode=False,
        variant=args.variant,
        left_base_link=args.left_base_link,
        left_end_link=args.left_end_link,
        right_base_link=args.right_base_link,
        right_end_link=args.right_end_link,
    )
    rc.set_backend(args.backend)
    if not robot.connect():
        return
    
    # Solve IK based on arm selection
    if args.arm == "both":
        ik_result = robot.set_pose_target(
            target_pose1=args.target_left,
            target_pose2=args.target_right,
            arm="both",
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
        ik_result = robot.set_pose_target(
            target_pose1=args.target_left,
            arm=args.arm,
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




if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Forward kinematics demo")
    
    # Robot configuration
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--baudrate', type=int, default=1000000,  help="波特率 (默认: 1000000)")
    parser.add_argument('--robot_version', type=str, default="v1_1",  help="机械臂版本 (默认: v1_1)")

    parser.add_argument('--arm', type=str, default='left_arm',
                        choices=['left_arm', 'right_arm', 'both'],
                        help='要查询的机械臂 (默认: both)')
    parser.add_argument('--variant', type=str, default='skeleton',
                        help='模型变体 (默认: skeleton)')
    parser.add_argument('--left-base-link', type=str, default='base_link',
                        help='左臂基座链接名称 (默认: base_link)')
    parser.add_argument('--left-end-link', type=str, default='left_arm_link7',
    # parser.add_argument('--left-end-link', type=str, default='left_tool0',
                        help='左臂末端执行器链接名称 (默认: left_tool0)')
    parser.add_argument('--right-base-link', type=str, default='base_link',
                        help='右臂基座链接名称 (默认: base_link)')
    parser.add_argument('--right-end-link', type=str, default='right_arm_link7',
    # parser.add_argument('--right-end-link', type=str, default='right_tool0',
                        help='右臂末端执行器链接名称 (默认: right_tool0)')

    parser.add_argument('--target-left', type=float, nargs='+',
                        default=[0.22016, +0.38612, +0.43572, -0.586009, -0.550986, -0.173321, +0.56830],
                        help='Target left end-effector pose as 7 floats (px, py, pz, qx, qy, qz, qw)')
    parser.add_argument('--target-right', type=float, nargs='+',
                        default=[0.22049, -0.38466, +0.43536, 0.582590, -0.551678, +0.173164, +0.571188],
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
    parser.add_argument('--execute', action='store_true', help='执行移动到求解的位置')
    args = parser.parse_args()

    main(args)
