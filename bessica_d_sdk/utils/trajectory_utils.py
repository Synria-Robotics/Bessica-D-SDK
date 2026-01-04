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

"""Shared utilities for trajectory planning demos (Dual-Arm Support).

This module provides common functions for waypoint loading, saving, and recording
that are shared between joint space and Cartesian space trajectory demos.
Adapted for dual-arm robots (Bessica).
"""

import numpy as np
import json
import os
import time
from typing import Optional, Tuple, List, Callable, Any, Dict, Union

from robocore.utils.beauty_logger import beauty_print, beauty_print_array
from robocore.utils.backend import to_numpy
from robocore.transform import make_transform, quaternion_to_matrix, rpy_to_matrix, matrix_to_quaternion


def record_waypoints_manual(controller,
                            get_state_fn: Optional[Callable] = None,
                            format_fn: Optional[Callable] = None,
                            arm: str = "both") -> List[Any]:
    """
    General manual waypoint recording function for dual-arm robots.

    :param controller: Robot controller
    :param get_state_fn: Custom state getter function, returns data to record. If None, uses default joint+gripper
    :param format_fn: Custom formatting function for log output. If None, uses default format
    :param arm: Arm to record, "left", "right", or "both"
    :return: List of recorded waypoints
    """
    print("\n=== 手动记录模式 ===")
    print("关闭扭矩后，拖动到目标位置按回车记录")
    if arm == "both":
        print("记录双臂waypoints")

    input("按回车开始...")
    controller.torque_control('off')
    print("[安全] 扭矩已关闭，可以拖动机械臂")

    waypoints = []

    try:
        while True:
            cmd = input(f"\n拖动到位置后按回车记录第{len(waypoints) + 1}个点，输入'q'结束: ").strip()
            if cmd.lower() == 'q':
                break

            # 使用自定义或默认的状态获取函数
            if get_state_fn:
                state = get_state_fn(controller, arm=arm)
            else:
                # 默认：记录关节角度和夹爪状态（使用统一API一次性获取）
                if arm == "both":
                    robot_state = controller.get_robot_state("joint_gripper")
                    if robot_state is not None:
                        joints_dict = robot_state.angles if isinstance(robot_state.angles, dict) else {'left': robot_state.angles, 'right': robot_state.angles}
                        gripper_dict = robot_state.gripper if isinstance(robot_state.gripper, dict) else {'left': robot_state.gripper, 'right': robot_state.gripper}
                        state = {"t": time.time(), "q_left": joints_dict.get('left'), "q_right": joints_dict.get('right'), 
                                "grip_left": gripper_dict.get('left', 0.0), "grip_right": gripper_dict.get('right', 0.0)}
                    else:
                        state = None
                else:
                    robot_state = controller.get_robot_state("joint_gripper", arm=arm)
                    if robot_state is not None:
                        joints = robot_state.angles
                        gripper = robot_state.gripper
                        state = {"t": time.time(), "q": joints, "grip": gripper} if joints is not None else None
                    else:
                        state = None

            if state:
                waypoints.append(state)

                # 使用自定义或默认的格式化函数输出日志
                if format_fn:
                    format_fn(len(waypoints), state, arm=arm)  # Call function, don't print return value
                else:
                    # 默认格式
                    if arm == "both":
                        if isinstance(state, dict) and 'q_left' in state:
                            print(f"[记录] 第{len(waypoints)}个点: 左臂关节{[round(j, 3) for j in state['q_left']]}, 右臂关节{[round(j, 3) for j in state['q_right']]}")
                            print(f"        左夹爪{state.get('grip_left', 0):.3f}, 右夹爪{state.get('grip_right', 0):.3f}")
                    else:
                        if isinstance(state, dict) and 'q' in state:
                            print(f"[记录] 第{len(waypoints)}个点: 关节{[round(j, 3) for j in state['q']]}, 夹爪{state.get('grip', 0):.3f}")
                        else:
                            print(f"[记录] 第{len(waypoints)}个点")

    finally:
        controller.torque_control('on')
        print("[安全] 扭矩已重新开启")

    return waypoints


def load_joint_waypoints_from_file(file_path: str, arm: str = "both") -> Tuple[Union[np.ndarray, Dict[str, np.ndarray]], Optional[Union[np.ndarray, Dict[str, np.ndarray]]]]:
    """Load joint waypoints from JSON file (supports dual-arm).
    
    :param file_path: Path to JSON file containing joint waypoints
    :param arm: Arm to load, "left", "right", or "both"
    :return: Tuple of (waypoints_array, gripper_values) where:
             - For single arm: waypoints_array [n_waypoints, n_dof], gripper_values [n_waypoints] or None
             - For dual arm: waypoints_dict {'left': [n_waypoints, n_dof], 'right': [n_waypoints, n_dof]}, 
                            gripper_dict {'left': [n_waypoints], 'right': [n_waypoints]} or None
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Waypoint file not found: {file_path}")
    
    with open(file_path, 'r') as f:
        data = json.load(f)
    
    waypoints_left = []
    waypoints_right = []
    gripper_values_left = []
    gripper_values_right = []
    has_gripper = False
    
    # Check if data is a list
    if not isinstance(data, list):
        raise ValueError("JSON file must contain a list of waypoints")
    
    for i, wp in enumerate(data):
        if isinstance(wp, list):
            # Old format: list of joint angles only (single arm)
            if arm != "both":
                waypoints_left.append(np.array(wp, dtype=np.float64))
                gripper_values_left.append(None)
            else:
                raise ValueError(f"Waypoint {i+1}: Dual-arm mode requires dict format with 'left' and 'right' keys")
        elif isinstance(wp, dict):
            # New format: dict with 'joints' or 'left'/'right' keys
            if arm == "both":
                if 'left' in wp and 'right' in wp:
                    waypoints_left.append(np.array(wp['left'], dtype=np.float64))
                    waypoints_right.append(np.array(wp['right'], dtype=np.float64))
                    if 'gripper_left' in wp:
                        gripper_values_left.append(float(wp['gripper_left']))
                        has_gripper = True
                    else:
                        gripper_values_left.append(None)
                    if 'gripper_right' in wp:
                        gripper_values_right.append(float(wp['gripper_right']))
                        has_gripper = True
                    else:
                        gripper_values_right.append(None)
                else:
                    raise ValueError(f"Waypoint {i+1}: Dual-arm mode requires 'left' and 'right' keys")
            else:
                # Single arm mode
                if 'joints' in wp:
                    waypoints_left.append(np.array(wp['joints'], dtype=np.float64))
                    if 'gripper' in wp:
                        gripper_values_left.append(float(wp['gripper']))
                        has_gripper = True
                    else:
                        gripper_values_left.append(None)
                elif arm in wp:
                    waypoints_left.append(np.array(wp[arm], dtype=np.float64))
                    if f'gripper_{arm}' in wp:
                        gripper_values_left.append(float(wp[f'gripper_{arm}']))
                        has_gripper = True
                    else:
                        gripper_values_left.append(None)
                else:
                    raise ValueError(f"Waypoint {i+1}: dict must contain 'joints' key or '{arm}' key")
        else:
            raise ValueError(f"Waypoint {i+1}: must be a list of joint angles or a dict")
    
    if len(waypoints_left) < 2:
        raise ValueError("At least 2 waypoints are required")
    
    # Convert to numpy arrays
    waypoints_left_array = np.array(waypoints_left)
    n_dof = waypoints_left_array.shape[1]
    
    # Check all waypoints have same DOF
    for i, wp in enumerate(waypoints_left):
        if len(wp) != n_dof:
            raise ValueError(f"Waypoint {i+1}: has {len(wp)} joints, expected {n_dof}")
    
    if arm == "both":
        waypoints_right_array = np.array(waypoints_right)
        if waypoints_right_array.shape[1] != n_dof:
            raise ValueError("Left and right waypoints must have same DOF")
        
        # Convert gripper values to arrays if all waypoints have gripper info
        if has_gripper:
            gripper_left_array = np.array([g if g is not None else 0.0 for g in gripper_values_left], dtype=np.float64)
            gripper_right_array = np.array([g if g is not None else 0.0 for g in gripper_values_right], dtype=np.float64)
            return {'left': waypoints_left_array, 'right': waypoints_right_array}, {'left': gripper_left_array, 'right': gripper_right_array}
        else:
            return {'left': waypoints_left_array, 'right': waypoints_right_array}, None
    else:
        # Single arm
        if has_gripper:
            gripper_array = np.array([g if g is not None else 0.0 for g in gripper_values_left], dtype=np.float64)
            return waypoints_left_array, gripper_array
        else:
            return waypoints_left_array, None


def save_joint_waypoints_to_file(waypoints: Union[np.ndarray, Dict[str, np.ndarray]], 
                                 file_path: str, 
                                 gripper_values: Optional[Union[np.ndarray, Dict[str, np.ndarray]]] = None,
                                 arm: str = "both"):
    """Save joint waypoints to JSON file (supports dual-arm).
    
    :param waypoints: For single arm: Array [n_waypoints, n_dof]. For dual arm: Dict with 'left' and 'right' keys
    :param file_path: Path to save JSON file
    :param gripper_values: For single arm: Optional array [n_waypoints]. For dual arm: Optional Dict with 'left' and 'right' keys
    :param arm: Arm mode, "left", "right", or "both"
    """
    waypoints_list = []
    
    if isinstance(waypoints, dict):
        # Dual-arm mode
        n_waypoints = len(waypoints['left'])
        for i in range(n_waypoints):
            wp_dict = {
                'left': waypoints['left'][i].tolist() if isinstance(waypoints['left'][i], np.ndarray) else list(waypoints['left'][i]),
                'right': waypoints['right'][i].tolist() if isinstance(waypoints['right'][i], np.ndarray) else list(waypoints['right'][i])
            }
            if gripper_values is not None:
                if isinstance(gripper_values, dict):
                    wp_dict['gripper_left'] = float(gripper_values['left'][i]) if i < len(gripper_values['left']) else 0.0
                    wp_dict['gripper_right'] = float(gripper_values['right'][i]) if i < len(gripper_values['right']) else 0.0
            waypoints_list.append(wp_dict)
    else:
        # Single-arm mode
        for i, wp in enumerate(waypoints):
            wp_list = wp.tolist() if isinstance(wp, np.ndarray) else list(wp)
            if gripper_values is not None and i < len(gripper_values):
                waypoints_list.append({
                    'joints': wp_list,
                    'gripper': float(gripper_values[i])
                })
            else:
                waypoints_list.append(wp_list)
    
    with open(file_path, 'w') as f:
        json.dump(waypoints_list, f, indent=2)
    
    gripper_info = f" (with gripper)" if gripper_values is not None else ""
    arm_info = f" ({arm})" if arm != "both" else " (dual-arm)"
    beauty_print(f"Saved {len(waypoints_list)} waypoints{gripper_info}{arm_info} to {file_path}", type="success")


def record_joint_waypoints_manual(robot, arm: str = "both") -> Tuple[Optional[Union[np.ndarray, Dict[str, np.ndarray]]], Optional[Union[np.ndarray, Dict[str, np.ndarray]]]]:
    """Record joint waypoints by manually dragging robot (supports dual-arm).
    
    :param robot: Robot controller instance
    :param arm: Arm to record, "left", "right", or "both"
    :return: Tuple of (waypoints, gripper_values) where:
             - For single arm: waypoints [n_waypoints, n_dof], gripper_values [n_waypoints]
             - For dual arm: waypoints dict {'left': [n_waypoints, n_dof], 'right': [n_waypoints, n_dof]}, 
                            gripper_values dict {'left': [n_waypoints], 'right': [n_waypoints]}
    """
    # Define custom state getter for joint waypoints
    def get_joint_state(controller, arm: str = "both"):
        if arm == "both":
            # Get both arms
            robot_state = controller.get_robot_state("joint_gripper")
            if robot_state is None:
                beauty_print("✗ 无法获取当前关节角度和夹爪状态", type="warning")
                return None
            
            joints_dict = robot_state.angles if isinstance(robot_state.angles, dict) else {'left': robot_state.angles, 'right': robot_state.angles}
            gripper_dict = robot_state.gripper if isinstance(robot_state.gripper, dict) else {'left': robot_state.gripper, 'right': robot_state.gripper}
            
            return {
                "q_left": to_numpy(joints_dict.get('left')),
                "q_right": to_numpy(joints_dict.get('right')),
                "grip_left": float(gripper_dict.get('left', 0.0)),
                "grip_right": float(gripper_dict.get('right', 0.0))
            }
        else:
            # Single arm
            robot_state = controller.get_robot_state("joint_gripper", arm=arm)
            if robot_state is None:
                beauty_print("✗ 无法获取当前关节角度和夹爪状态", type="warning")
                return None
            
            joints = to_numpy(robot_state.angles)
            gripper = robot_state.gripper
            return {"q": joints, "grip": float(gripper)}
    
    # Define custom formatter for joint waypoints
    def format_joint_state(count, state, arm: str = "both"):
        if arm == "both":
            q_left = state.get('q_left')
            q_right = state.get('q_right')
            grip_left = state.get('grip_left', 0.0)
            grip_right = state.get('grip_right', 0.0)
            beauty_print(f"[记录] 第{count}个点:")
            print(f"  左臂关节角度 (rad): {beauty_print_array(q_left)}")
            print(f"  左臂关节角度 (deg): {beauty_print_array(np.rad2deg(q_left))}")
            print(f"  左夹爪状态: {grip_left:.1f} (0-1000)")
            print(f"  右臂关节角度 (rad): {beauty_print_array(q_right)}")
            print(f"  右臂关节角度 (deg): {beauty_print_array(np.rad2deg(q_right))}")
            print(f"  右夹爪状态: {grip_right:.1f} (0-1000)")
        else:
            joints = state.get('q')
            gripper = state.get('grip', 0.0)
            beauty_print(f"[记录] 第{count}个点 ({arm}):")
            print(f"  关节角度 (rad): {beauty_print_array(joints)}")
            print(f"  关节角度 (deg): {beauty_print_array(np.rad2deg(joints))}")
            print(f"  夹爪状态: {gripper:.1f} (0-1000, 0=闭合, 1000=张开)")
        return None  # Formatting is done via print
    
    # Use the general recording function
    beauty_print("\n=== 手动记录模式 ===", type="module", centered=False)
    beauty_print("关闭扭矩后，拖动到目标位置按回车记录")
    
    recorded_data = record_waypoints_manual(robot, get_state_fn=get_joint_state, format_fn=format_joint_state, arm=arm)
    
    if not recorded_data or len(recorded_data) < 2:
        beauty_print("至少需要2个waypoint才能生成轨迹", type="warning")
        return None, None
    
    # Convert to numpy arrays
    if arm == "both":
        waypoints_left = []
        waypoints_right = []
        gripper_values_left = []
        gripper_values_right = []
        for point in recorded_data:
            if point and 'q_left' in point:
                waypoints_left.append(to_numpy(point['q_left']))
                waypoints_right.append(to_numpy(point['q_right']))
                gripper_values_left.append(float(point['grip_left']))
                gripper_values_right.append(float(point['grip_right']))
            else:
                beauty_print("✗ 无法获取当前关节角度和夹爪状态", type="warning")
                return None, None
        return waypoints_left, waypoints_right, gripper_values_left, gripper_values_right
    else:
        waypoints = []
        gripper_values = []
        for point in recorded_data:
            if point and 'q' in point:
                waypoints.append(to_numpy(point['q']))