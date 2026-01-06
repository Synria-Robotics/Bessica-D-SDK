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


def _get_motion_file_dir() -> str:
    """Get the motion_file directory path relative to examples folder.
    
    :return: Absolute path to motion_files directory
    """
    # Get the directory where this file is located
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # Go up to bessica_d_sdk, then to Bessica-D-SDK, then to examples/motion_files
    # Structure: Bessica-D-SDK/bessica_d_sdk/utils/trajectory_utils.py
    # Target: Bessica-D-SDK/examples/motion_files
    # current_dir = .../Bessica-D-SDK/bessica_d_sdk/utils
    utils_dir = current_dir  # .../Bessica-D-SDK/bessica_d_sdk/utils
    sdk_dir = os.path.dirname(utils_dir)  # .../Bessica-D-SDK/bessica_d_sdk
    sdk_root = os.path.dirname(sdk_dir)  # .../Bessica-D-SDK
    motion_file_dir = os.path.join(sdk_root, "examples", "motion_files")
    return motion_file_dir


def _resolve_waypoints_path(file_path: str, create_dir: bool = False) -> str:
    """Resolve waypoints file path, handling relative paths and default motion_file folder.
    
    :param file_path: File path (can be relative or absolute)
    :param create_dir: If True, create the directory if it doesn't exist
    :return: Resolved absolute file path
    """
    # If absolute path, use as-is
    if os.path.isabs(file_path):
        if create_dir:
            os.makedirs(os.path.dirname(file_path), exist_ok=True)
        return file_path
    
    # If relative path, check if it's just a filename
    if os.path.dirname(file_path) == "":
        # Just a filename, put it in motion_file folder
        motion_file_dir = _get_motion_file_dir()
        if create_dir:
            os.makedirs(motion_file_dir, exist_ok=True)
        return os.path.join(motion_file_dir, file_path)
    else:
        # Relative path with directory, resolve relative to current working directory
        resolved = os.path.abspath(file_path)
        if create_dir:
            os.makedirs(os.path.dirname(resolved), exist_ok=True)
        return resolved


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
                    if robot_state is not None and isinstance(robot_state, dict):
                        left_js = robot_state.get('left')
                        right_js = robot_state.get('right')
                        if left_js is not None and right_js is not None:
                            state = {
                                "t": time.time(), 
                                "q_left": left_js.angles, 
                                "q_right": right_js.angles, 
                                "grip_left": left_js.gripper, 
                                "grip_right": right_js.gripper
                            }
                        else:
                            state = None
                    else:
                        state = None
                else:
                    robot_state = controller.get_robot_state("joint_gripper")
                    if robot_state is not None and isinstance(robot_state, dict):
                        js = robot_state.get(arm)
                        if js is not None:
                            state = {"t": time.time(), "q": js.angles, "grip": js.gripper}
                        else:
                            state = None
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
    
    :param file_path: Path to JSON file containing joint waypoints (relative paths will be searched in examples/motion_files/)
    :param arm: Arm to load, "left", "right", or "both"
    :return: Tuple of (waypoints_array, gripper_values) where:
             - For single arm: waypoints_array [n_waypoints, n_dof], gripper_values [n_waypoints] or None
             - For dual arm: waypoints_dict {'left': [n_waypoints, n_dof], 'right': [n_waypoints, n_dof]}, 
                            gripper_dict {'left': [n_waypoints], 'right': [n_waypoints]} or None
    """
    # Resolve path (try motion_files folder for relative paths)
    resolved_path = _resolve_waypoints_path(file_path, create_dir=False)
    
    # If file not found in motion_files, try original path (for backward compatibility)
    if not os.path.exists(resolved_path):
        # Try original path if it was a relative path
        if not os.path.isabs(file_path):
            original_path = os.path.abspath(file_path)
            if os.path.exists(original_path):
                resolved_path = original_path
            else:
                raise FileNotFoundError(f"Waypoint file not found: {resolved_path} (also tried: {original_path})")
        else:
            raise FileNotFoundError(f"Waypoint file not found: {resolved_path}")
    
    with open(resolved_path, 'r') as f:
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
    :param file_path: Path to save JSON file (relative paths will be saved to examples/motion_file/)
    :param gripper_values: For single arm: Optional array [n_waypoints]. For dual arm: Optional Dict with 'left' and 'right' keys
    :param arm: Arm mode, "left", "right", or "both"
    """
    # Resolve path and create directory if needed
    resolved_path = _resolve_waypoints_path(file_path, create_dir=True)
    
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
    
    with open(resolved_path, 'w') as f:
        json.dump(waypoints_list, f, indent=2)
    
    gripper_info = f" (with gripper)" if gripper_values is not None else ""
    arm_info = f" ({arm})" if arm != "both" else " (dual-arm)"
    beauty_print(f"Saved {len(waypoints_list)} waypoints{gripper_info}{arm_info} to {resolved_path}", type="success")


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
            # Get both arms - returns dict with 'left' and 'right' keys, each containing JointState
            robot_state = controller.get_robot_state("joint_gripper")
            if robot_state is None:
                beauty_print("✗ 无法获取当前关节角度和夹爪状态", type="warning")
                return None
            
            # robot_state is a dict: {'left': JointState(...), 'right': JointState(...)}
            if not isinstance(robot_state, dict):
                beauty_print("✗ 无法获取当前关节角度和夹爪状态", type="warning")
                return None
            
            left_js = robot_state.get('left')
            right_js = robot_state.get('right')
            
            if left_js is None or right_js is None:
                beauty_print("✗ 无法获取当前关节角度和夹爪状态", type="warning")
                return None
            
            # Extract from JointState objects
            return {
                "q_left": to_numpy(left_js.angles),
                "q_right": to_numpy(right_js.angles),
                "grip_left": float(left_js.gripper),
                "grip_right": float(right_js.gripper)
            }
        else:
            # Single arm - get_robot_state returns dict, extract the specific arm
            robot_state = controller.get_robot_state("joint_gripper")
            if robot_state is None:
                beauty_print("✗ 无法获取当前关节角度和夹爪状态", type="warning")
                return None
            
            # robot_state is a dict: {'left': JointState(...), 'right': JointState(...)}
            if not isinstance(robot_state, dict):
                beauty_print("✗ 无法获取当前关节角度和夹爪状态", type="warning")
                return None
            
            js = robot_state.get(arm)
            if js is None:
                beauty_print("✗ 无法获取当前关节角度和夹爪状态", type="warning")
                return None
            
            joints = to_numpy(js.angles)
            gripper = js.gripper
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
        waypoints = {'left': np.array(waypoints_left), 'right': np.array(waypoints_right)}
        gripper_waypoints = {'left': np.array(gripper_values_left), 'right': np.array(gripper_values_right)}
        return waypoints, gripper_waypoints
    else:
        waypoints = []
        gripper_values = []
        for point in recorded_data:
            if point and 'q' in point:
                waypoints.append(to_numpy(point['q']))
                gripper_values.append(float(point.get('grip', 0.0)))
        
        return np.array(waypoints), np.array(gripper_values) if gripper_values else None


def load_or_generate_joint_waypoints(robot, robot_model, args, arm: str = "both") -> Tuple[Union[np.ndarray, Dict[str, np.ndarray]], Optional[Union[np.ndarray, Dict[str, np.ndarray]]]]:
    """Load or generate joint waypoints (supports single-arm and dual-arm).
    
    :param robot: Robot controller instance
    :param robot_model: Robot model instance (BimanualRobotModel for dual-arm)
    :param args: Command line arguments
    :param arm: Arm mode, "left", "right", or "both"
    :return: Tuple of (waypoints, gripper_waypoints)
    """
    from robocore.utils.backend import to_numpy
    
    if args.waypoints_file:
        beauty_print(f"Loading waypoints from file: {args.waypoints_file}")
        waypoints, gripper_waypoints = load_joint_waypoints_from_file(args.waypoints_file, arm=arm)
        beauty_print(f"Successfully loaded waypoints from file")
        if gripper_waypoints is not None:
            if isinstance(gripper_waypoints, dict):
                beauty_print(f"Gripper values loaded: left={len(gripper_waypoints['left'])}, right={len(gripper_waypoints['right'])}")
            else:
                beauty_print(f"Gripper values loaded: {len(gripper_waypoints)} waypoints")
        return waypoints, gripper_waypoints
    
    # Generate random waypoints
    beauty_print(f"Generating {args.num_waypoints} random waypoints within joint limits...")
    
    if arm == "both":
        waypoints_left = []
        waypoints_right = []
        gripper_values_left = []
        gripper_values_right = []
        
        # Get current joint angles as starting point (optional)
        if args.use_current_joints:
            robot_state = robot.get_robot_state("joint_gripper")
            if robot_state is not None and isinstance(robot_state, dict):
                left_js = robot_state.get('left')
                right_js = robot_state.get('right')
                if left_js is not None and right_js is not None:
                    q_start_left = to_numpy(left_js.angles)
                    q_start_right = to_numpy(right_js.angles)
                    g_start_left = float(left_js.gripper)
                    g_start_right = float(right_js.gripper)
                
                waypoints_left.append(q_start_left)
                waypoints_right.append(q_start_right)
                gripper_values_left.append(g_start_left)
                gripper_values_right.append(g_start_right)
                
                beauty_print(f"Using current joint angles and gripper as first waypoint:")
                print(f"  Left joints (rad): {beauty_print_array(q_start_left)}")
                print(f"  Right joints (rad): {beauty_print_array(q_start_right)}")
                print(f"  Left gripper: {g_start_left:.1f}, Right gripper: {g_start_right:.1f}")
                num_random = args.num_waypoints - 1
            else:
                beauty_print("✗ 无法获取当前关节角度，使用随机生成", type="warning")
                num_random = args.num_waypoints
        else:
            num_random = args.num_waypoints
        
        # Generate random waypoints
        for i in range(num_random):
            waypoint_seed = args.seed + i if args.seed is not None else None
            # Use left_model for generating waypoints (both arms have same structure)
            q_left = to_numpy(robot_model.left_model.random_q(seed=waypoint_seed, scale=args.joint_scale))
            q_right = to_numpy(robot_model.right_model.random_q(seed=waypoint_seed, scale=args.joint_scale))
            waypoints_left.append(q_left)
            waypoints_right.append(q_right)
            
            # Random gripper values
            if waypoint_seed is not None:
                np.random.seed(waypoint_seed)
            gripper_values_left.append(float(np.random.uniform(0, 1000)))
            gripper_values_right.append(float(np.random.uniform(0, 1000)))
        
        waypoints = {'left': np.array(waypoints_left), 'right': np.array(waypoints_right)}
        gripper_waypoints = {'left': np.array(gripper_values_left), 'right': np.array(gripper_values_right)}
        return waypoints, gripper_waypoints
    else:
        # Single-arm mode
        waypoints = []
        gripper_waypoints = []
        
        # Get current joint angles as starting point (optional)
        if args.use_current_joints:
            robot_state = robot.get_robot_state("joint_gripper")
            if robot_state is not None and isinstance(robot_state, dict):
                js = robot_state.get(arm)
                if js is not None:
                    q_start = to_numpy(js.angles)
                    g_start = float(js.gripper) if js.gripper is not None else 500.0
                    waypoints.append(q_start)
                    gripper_waypoints.append(g_start)
                    beauty_print(f"Using current joint angles and gripper as first waypoint ({arm}):")
                    print(f"  Current joints (rad): {beauty_print_array(q_start)}")
                    print(f"  Current joints (deg): {beauty_print_array(np.rad2deg(q_start))}")
                    print(f"  Current gripper: {g_start:.1f} (0-1000)")
                    num_random = args.num_waypoints - 1
                else:
                    beauty_print("✗ 无法获取当前关节角度，使用随机生成", type="warning")
                    num_random = args.num_waypoints
            else:
                beauty_print("✗ 无法获取当前关节角度，使用随机生成", type="warning")
                num_random = args.num_waypoints
        else:
            num_random = args.num_waypoints
        
        # Generate random waypoints
        model = robot_model.left_model if arm == "left" else robot_model.right_model
        for i in range(num_random):
            waypoint_seed = args.seed + i if args.seed is not None else None
            q = to_numpy(model.random_q(seed=waypoint_seed, scale=args.joint_scale))
            waypoints.append(q)
            # Random gripper value
            if waypoint_seed is not None:
                np.random.seed(waypoint_seed)
            gripper_waypoints.append(float(np.random.uniform(0, 1000)))
        
        return np.array(waypoints), np.array(gripper_waypoints) if gripper_waypoints else None


def display_joint_waypoints(waypoints: Union[np.ndarray, Dict[str, np.ndarray]], 
                           gripper_waypoints: Optional[Union[np.ndarray, Dict[str, np.ndarray]]] = None):
    """Display joint waypoints information (supports single-arm and dual-arm).
    
    :param waypoints: For single arm: Array [n_waypoints, n_dof].
                      For dual arm: Dict with 'left' and 'right' keys
    :param gripper_waypoints: For single arm: Optional array [n_waypoints].
                              For dual arm: Optional Dict with 'left' and 'right' keys
    """
    if isinstance(waypoints, dict):
        beauty_print(f"Waypoints (dual-arm): {len(waypoints['left'])}")
        for i in range(len(waypoints['left'])):
            print(f"  Waypoint {i+1}:")
            print(f"    Left:  {beauty_print_array(waypoints['left'][i])} (rad)")
            print(f"           {beauty_print_array(np.rad2deg(waypoints['left'][i]))} (deg)")
            print(f"    Right: {beauty_print_array(waypoints['right'][i])} (rad)")
            print(f"           {beauty_print_array(np.rad2deg(waypoints['right'][i]))} (deg)")
            if gripper_waypoints is not None and isinstance(gripper_waypoints, dict):
                if i < len(gripper_waypoints['left']):
                    print(f"    Gripper: Left={gripper_waypoints['left'][i]:.1f}, Right={gripper_waypoints['right'][i]:.1f}")
    else:
        beauty_print(f"Waypoints: {len(waypoints)}")
        for i, wp in enumerate(waypoints):
            print(f"  Waypoint {i+1}: {beauty_print_array(wp)} (rad)")
            print(f"              {beauty_print_array(np.rad2deg(wp))} (deg)")
            if gripper_waypoints is not None and i < len(gripper_waypoints):
                print(f"              夹爪: {gripper_waypoints[i]:.1f} (0-1000)")


def display_joint_trajectory_stats(trajectory: dict, arm: str = "both"):
    """Display joint trajectory statistics (supports single-arm and dual-arm).
    
    :param trajectory: Trajectory dictionary with 't', 'q', 'qd', 'qdd', etc.
    :param arm: Arm mode, "left", "right", or "both"
    """
    beauty_print(f"Trajectory generated:")
    print(f"  Duration: {trajectory['t'][-1]:.3f} s")
    
    if arm == "both" and 'q_left' in trajectory:
        print(f"  Points: {len(trajectory['t'])}")
        print(f"  Left arm - Max velocity: {np.max(np.abs(trajectory['qd_left'])):.3f} rad/s")
        print(f"  Left arm - Max acceleration: {np.max(np.abs(trajectory['qdd_left'])):.3f} rad/s²")
        print(f"  Right arm - Max velocity: {np.max(np.abs(trajectory['qd_right'])):.3f} rad/s")
        print(f"  Right arm - Max acceleration: {np.max(np.abs(trajectory['qdd_right'])):.3f} rad/s²")
        
        if 'qddd_left' in trajectory:
            print(f"  Left arm - Max jerk: {np.max(np.abs(trajectory['qddd_left'])):.3f} rad/s³")
        if 'qddd_right' in trajectory:
            print(f"  Right arm - Max jerk: {np.max(np.abs(trajectory['qddd_right'])):.3f} rad/s³")
        
        # Display gripper trajectory if available
        if 'gripper_left' in trajectory or 'gripper_right' in trajectory:
            beauty_print(f"Gripper trajectory interpolated: {len(trajectory['t'])} points")
            if 'gripper_left' in trajectory:
                print(f"  Left gripper range: [{np.min(trajectory['gripper_left']):.1f}, {np.max(trajectory['gripper_left']):.1f}]")
            if 'gripper_right' in trajectory:
                print(f"  Right gripper range: [{np.min(trajectory['gripper_right']):.1f}, {np.max(trajectory['gripper_right']):.1f}]")
    else:
        print(f"  Points: {len(trajectory['t'])}")
        print(f"  Max velocity: {np.max(np.abs(trajectory['qd'])):.3f} rad/s")
        print(f"  Max acceleration: {np.max(np.abs(trajectory['qdd'])):.3f} rad/s²")
        if 'qddd' in trajectory:
            print(f"  Max jerk: {np.max(np.abs(trajectory['qddd'])):.3f} rad/s³")
        
        # Display gripper trajectory if available
        if 'gripper' in trajectory:
            beauty_print(f"Gripper trajectory interpolated: {len(trajectory['gripper'])} points")
            print(f"  Gripper range: [{np.min(trajectory['gripper']):.1f}, {np.max(trajectory['gripper']):.1f}]")


def handle_waypoint_recording(robot, args, waypoint_type: str = 'joint', arm: str = "both") -> Tuple[Optional[Union[np.ndarray, Dict[str, np.ndarray]]], Optional[Union[np.ndarray, Dict[str, np.ndarray]]]]:
    """Handle waypoint recording mode (common logic for both joint and Cartesian).
    
    :param robot: Robot controller instance
    :param args: Command line arguments
    :param waypoint_type: 'joint' or 'cartesian'
    :param arm: Arm mode, "left", "right", or "both"
    :return: Tuple of (waypoints, gripper_values) where gripper_values is None for Cartesian
    """
    waypoints = None
    gripper_waypoints = None
    
    # Record by default unless --no-record is specified or --waypoints-file is provided
    should_record = not args.no_record and args.waypoints_file is None
    if should_record:
        if waypoint_type == 'joint':
            beauty_print("[0] Recording Joint Waypoints", type="module", centered=False)
            waypoints, gripper_waypoints = record_joint_waypoints_manual(robot, arm=arm)
        
        if waypoints is None:
            robot.disconnect()
            return None, None
        
        # Save to file if specified
        if args.save_file:
            if waypoint_type == 'joint':
                save_joint_waypoints_to_file(waypoints, args.save_file, gripper_waypoints, arm=arm)
            beauty_print(f"Waypoints saved to {args.save_file}", type="success")
        else:
            beauty_print("No save file specified. Use --save-file to save waypoints.", type="warning")
        
        # Ask if user wants to execute trajectory
        user_input = input("\nExecute trajectory with recorded waypoints? (y/n): ").strip().lower()
        if user_input != 'y':
            beauty_print("Exiting without execution.", type="info")
            robot.disconnect()
            return None, None
    
    return waypoints, gripper_waypoints


def plot_trajectory(trajectory: dict, waypoints: Union[np.ndarray, Dict[str, np.ndarray]], 
                   plot_type: str = 'joint', 
                   joint_angles: Optional[Union[np.ndarray, Dict[str, np.ndarray]]] = None, 
                   ik_results: Optional[list] = None,
                   arm: str = "both"):
    """Plot trajectory visualization (supports single-arm and dual-arm).
    
    :param trajectory: Trajectory dictionary
    :param waypoints: Waypoints array or dict
    :param plot_type: 'joint' or 'cartesian'
    :param joint_angles: Optional joint angles for Cartesian plotting
    :param ik_results: Optional IK results for Cartesian plotting
    :param arm: Arm mode, "left", "right", or "both"
    """
    try:
        import matplotlib.pyplot as plt
        if plot_type == 'joint':
            from robocore.planning import plot_joint_trajectory
            if isinstance(waypoints, dict):
                # Dual-arm: create separate trajectory dicts for each arm and plot separately
                traj_left = {
                    't': trajectory['t'],
                    'q': trajectory['q_left'],
                    'qd': trajectory['qd_left'],
                    'qdd': trajectory['qdd_left']
                }
                traj_right = {
                    't': trajectory['t'],
                    'q': trajectory['q_right'],
                    'qd': trajectory['qd_right'],
                    'qdd': trajectory['qdd_right']
                }
                # Plot left arm first
                plot_joint_trajectory(traj_left, waypoints['left'])
                # Plot right arm (will be on same figure)
                plot_joint_trajectory(traj_right, waypoints['right'])
            else:
                plot_joint_trajectory(trajectory, waypoints)
        else:  # cartesian
            from robocore.planning import plot_cartesian_with_ik
            plot_cartesian_with_ik(trajectory, waypoints, joint_angles, ik_results)
        plt.show(block=False)
        plt.pause(0.1)
    except ImportError:
        beauty_print("matplotlib not installed. Skipping plots.", type="warning")