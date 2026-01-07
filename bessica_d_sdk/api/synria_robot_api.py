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
SynriaBessicaRobotAPI - User-level API (Bessica)

职责:
- 提供统一的上层用户接口
- 关节/夹爪/云台等高层控制封装
- 状态查询接口
- 系统控制函数
- 参数校验与最小插值运动

说明:
- 适配 Bessica 的 ServoDriver：单臂7关节、需显式传入 arm("left"/"right")；支持双臂 both
- 尽量对齐 Alicia 的 SynriaRobotAPI 方法命名，以便后续规范化统一
"""

import time
from typing import List, Optional, Dict, Union, Tuple, Any
import numpy as np
import json
import os
import logging
from robocore.kinematics import inverse_kinematics
from robocore.kinematics.bimanual import bimanual_inverse_kinematics
from robocore.modeling.robot_model import BimanualRobotModel
from robocore.transform import make_transform, quaternion_to_matrix, quat2mat
from robocore.kinematics import forward_kinematics
from robocore.kinematics.bimanual import bimanual_forward_kinematics
from robocore.transform import matrix_to_euler, matrix_to_quaternion
from synriard import get_model_path
from bessica_d_sdk.hardware.data_parser import JointState

from ..utils.logger import logger
from ..utils.fps_utils import precise_sleep
from ..hardware import ServoDriver

class SynriaBessicaRobotAPI:
    """Bessica 机器人 API - 与 ServoDriver 协作，面向双臂/单臂控制。"""

    def __init__(
        self,
        servo_driver: ServoDriver,
        speed_deg_s: float = 20.0,
        robot_version: str = "v1_0",
        variant: str = "skeleton",
        left_base_link: str = 'base_link',
        left_end_link: str = 'left_arm_link7',
        right_base_link: str = 'base_link',
        right_end_link: str = 'right_arm_link7',
        auto_connect: bool = True,
    ):
        """Initialize robot API.

        :param servo_driver: Servo driver instance
        :param speed_deg_s: Default speed in degrees per second
        :param robot_version: Robot version string, e.g., "v1_0"
        :param variant: Model variant string, e.g., "skeleton"
        :param left_base_link: Left arm base link name
        :param left_end_link: Left arm end-effector link name
        :param right_base_link: Right arm base link name
        :param right_end_link: Right arm end-effector link name
        """
        self.servo_driver = servo_driver
        self.data_parser = servo_driver.data_parser  # Direct access to data parser

        model_path = str(get_model_path("Bessica_D", version=robot_version, variant=variant))

        self.robot_model = BimanualRobotModel(
            model_path, 
            left_end_link=left_end_link, 
            right_end_link=right_end_link,
            base_link=left_base_link  # Both arms share the same base_link
        )
        
        # Access left and right arm models from bimanual model
        self.left_model = self.robot_model.left_model
        self.right_model = self.robot_model.right_model
        
        self.speed_deg_s = speed_deg_s
        self.home_angles = [0.0] * 7
        if auto_connect:
            self.connect()

    # ==================== Connection Management ====================

    def connect(self) -> bool:
        """Connect to robot.

        :return: True if connection successful
        """
        result = self.servo_driver.connect()
        state = self.get_robot_state("joint_gripper")
        time.sleep(0.001)
        return result

    def disconnect(self):
        """Disconnect from robot and stop update threads."""
        self.servo_driver.stop_update_thread()
        self.servo_driver.disconnect()


    # ==================== Get Robot Information ====================

    def get_robot_state(self, info_type: str = "joint_gripper", timeout: float = 1.0) -> Optional[Union[JointState, Dict, List[float], str, float]]:
        """
        Unified API to get robot state information.
        
        :param info_type: Type of information to get. Options:
            - "joint_gripper": Returns JointState (arm joint angles, gripper value, timestamp, run_status_text)
            - "joint": Returns List[float] of arm joint angles (radians) only
            - "gripper": Returns float gripper value (0-1000) only
            - "version": Returns Dict with serial_number, hardware_version, firmware_version
            - "temperature": Returns List[float] of temperatures in Celsius
            - "velocity": Returns List[float] of velocities in degrees per second
            - "gripper_type": Returns str (e.g., "50mm" or "100mm") or None if unavailable
            - "self_check": Returns Dict with self-check data (or None if failed)
        :param timeout: Maximum time to wait for response in seconds
        :return: Requested data or None if failed
        """
        # Special handling for gripper_type: try cache first, then hardware query
        if info_type == "gripper_type":
            return self._get_gripper_type_with_cache(timeout)

        # Joint and gripper are acquired together from hardware using the "joint" command
        if info_type in ("joint_gripper", "joint", "gripper"):
            if not self.servo_driver.acquire_info("joint_gripper", wait=True, timeout=timeout):
                logger.error(f"Failed to get joint/gripper data within timeout period")
                return None
            return self.data_parser.get_info(info_type)
        
        # Other info types map directly to hardware commands
        if not self.servo_driver.acquire_info(info_type, wait=True, timeout=timeout):
            logger.error(f"Failed to get {info_type} data within timeout period")
            return None
        result = self.data_parser.get_info(info_type)
        return result

    # ==================== 关节控制 ====================
    def set_home(self, arm: str = "both", speed_deg_s: float = 20.0) -> bool:
        """Move robot to home position.

        :param arm: Arm to control, "left", "right", or "both"
        :param speed_deg_s: Speed in degrees per second
        :return: True if successful
        """
        logger.info(f"set_home: arm={arm}")
        home_angles = [0.0] * 7

        if arm == "both":
            # Dual-arm: use unified set_robot_state
            return self.set_robot_state(
                target_joints=[home_angles, home_angles],
                gripper_value=[0, 0],  # Open both grippers
                arm="both",
                joint_format="rad",
                speed_deg_s=speed_deg_s,
                wait_for_completion=False,
            )

        elif arm in ("left", "right"):
            # Single arm: move to home and open gripper
            return self.set_robot_state(
                target_joints=home_angles,
                gripper_value=1000,  # Open gripper
                arm=arm,
                joint_format="rad",
                speed_deg_s=speed_deg_s,
                wait_for_completion=False,
            )

        else:
            logger.error(f"set_home: 非法 arm={arm}")
            return False

    def set_robot_state(
        self,
        target_joints: Optional[Union[List[float], List[List[float]]]] = None,
        gripper_value: Optional[Union[float, List[float], Tuple[float, float]]] = None,
        arm: Optional[str] = None,
        joint_format: str = "rad",
        speed_deg_s: float = 20.0,
        tolerance: float = 0.0524,
        timeout: float = 10.0,
        wait_for_completion: bool = True,
    ) -> bool:
        """Set joint angles and/or gripper in a single combined command.

        :param target_joints: Optional target joint angles. 
            - For single arm: List[float] (7 angles). If None, keeps current
            - For dual arm: List[List[float]] (2x7 angles). If None, keeps current
        :param gripper_value: Optional gripper value (0-1000, where 1000 is fully open).
            - For single arm: float. If None, keeps current
            - For dual arm: List[float] or Tuple[float, float] (left, right). If None, keeps current
        :param arm: Arm to control, "left", "right", or "both" (default: "both")
        :param joint_format: Unit format for joints, "deg" or "rad" (default: "rad")
        :param speed_deg_s: Speed in degrees per second (default: 20.0)
        :param tolerance: Maximum allowed error per joint in radians (default: 0.0524 rad ≈ 3.0 deg)
        :param timeout: Maximum wait time in seconds (default: 10.0)
        :param wait_for_completion: If True, wait until target reached (default: True)
        :return: True if successful, False otherwise
        """
        arm = arm or "both"
        # Validate and convert joint format
        is_deg = joint_format.lower() in ("deg", "degree", "degrees")
        is_rad = joint_format.lower() in ("rad", "radian", "radians")
        if not (is_deg or is_rad):
            logger.error(f"set_robot_state: joint_format 必须是 'deg' 或 'rad'，当前: {joint_format}")
            return False
        
        convert = self.servo_driver.DEG_TO_RAD if is_deg else 1.0
        
        # Convert joints to radians if needed
        if target_joints is not None:
            if arm == "both":
                if not isinstance(target_joints, list) or len(target_joints) != 2:
                    logger.error("set_robot_state: arm='both' 时，target_joints 必须是包含2个列表的列表")
                    return False
                if is_deg:
                    target_joints = [[a * convert for a in target_joints[0]], 
                                    [a * convert for a in target_joints[1]]]
            elif arm in ("left", "right"):
                if not isinstance(target_joints, list) or len(target_joints) != 7:
                    logger.error(f"set_robot_state: 单臂模式必须提供7个关节角度，但得到 {len(target_joints) if isinstance(target_joints, list) else '非列表'}")
                    return False
                if is_deg:
                    target_joints = [a * convert for a in target_joints]
        
        # Normalize gripper value format
        if gripper_value is not None:
            if arm == "both":
                if isinstance(gripper_value, (list, tuple)) and len(gripper_value) == 2:
                    gripper_value = list(gripper_value)
                else:
                    # If single value provided for both arms, use it for both
                    gripper_value = [gripper_value, gripper_value]
            else:
                # Single arm: take first value if list/tuple provided
                if isinstance(gripper_value, (list, tuple)):
                    logger.warning(f"单臂模式但提供了列表形式的gripper_value，使用第一个值")
                    gripper_value = gripper_value[0]
        
        # Use unified method in servo_driver
        success = self.servo_driver.set_joint_and_gripper(
            joint_angles=target_joints,
            gripper_value=gripper_value,
            arm=arm,
            speed_deg_s=speed_deg_s
        )
        
        if not success:
            logger.error("Failed to set robot target")
            return False
        
        # Wait for completion if requested
        if wait_for_completion and target_joints is not None:
            return self._wait_for_joint_target(
                target_joints=target_joints,
                arm=arm,
                tolerance=tolerance,
                timeout=timeout,
                log_prefix="等待关节接近目标"
            )
        
        return True

    def set_pose_target(self,
                        target_pose1: List[float],
                        target_pose2: Optional[List[float]] = None,
                        arm: Optional[str] = None,
                        method: str = 'dls',
                        speed_deg_s: float = 5.0,
                        tolerance: float = 1e-3,
                        max_iters: int = 100,
                        execute: bool = True) -> Dict:
        """Move end-effector to target pose using inverse kinematics.

        :param target_pose1: Target pose as [x, y, z, qx, qy, qz, qw]. For single arm: used for specified arm. For both: left arm.
        :param target_pose2: Target pose for right arm as [x, y, z, qx, qy, qz, qw] (required when arm="both")
        :param arm: Arm to control, "left", "right", or "both" (default: "both")
        :param method: IK solver method, 'dls', 'pinv', or 'transpose'
        :param tolerance: Position and orientation tolerance
        :param max_iters: Maximum number of iterations
        :param execute: Execute motion if True
        :return: Dictionary with success, q, iters, pos_err, ori_err, message
        """
        if not hasattr(self, 'robot_model') or self.robot_model is None:
            return {'success': False, 'message': 'robot_model not available', 'q': None}
        
        arm = arm or "both"
        
        # Get current joint positions as initial guess
        joints_dict = self.get_robot_state("joint")
        if joints_dict is None:
            current_joints = [0.0] * 7 if arm != "both" else [[0.0] * 7, [0.0] * 7]
        elif arm == "both":
            current_joints = [joints_dict.get('left', [0.0] * 7), joints_dict.get('right', [0.0] * 7)]
        else:
            current_joints = joints_dict.get(arm, [0.0] * 7)
        
        # Convert pose to transformation matrix
        def pose_to_matrix(pose: List[float]) -> np.ndarray:
            pos = np.array(pose[:3])
            quat = np.array(pose[3:])
            rot = quaternion_to_matrix(quat)
            return make_transform(rot, pos)
        
        # Handle single arm case
        if arm in ("left", "right"):
            T_target = pose_to_matrix(target_pose1)
            q0 = np.array(current_joints) if isinstance(current_joints, list) else np.array(current_joints)
            print(f"q0: {q0}")
            print(f"T_target: {T_target}")
            # Solve IK for single arm
            ik_result = self.robot_model.ik(
                target_left=T_target,
                target_right=T_target,
                q0_left=q0, 
                q0_right=q0,  
                method=method,
                coordination='indep',
                max_iters=max_iters,
                pos_tol=tolerance,
                ori_tol=tolerance,
            )
            # ik_result['res_left']['q'][3] = -ik_result['res_left']['q'][3]
            result = ik_result['res_left'] if arm == "left" else ik_result['res_right']
            
            # Execute motion if requested
            if execute and result.get('success', False):
                print(f"result: {result}")
                self.set_robot_state(target_joints=result['q'], arm=arm, joint_format="rad", wait_for_completion=False, speed_deg_s=speed_deg_s)
            
            return result
        
        # Handle both arms case
        elif arm == "both":
            if target_pose2 is None:
                return {'success': False, 'message': 'target_pose2 required for arm="both"', 'q': None}
            
            T_left = pose_to_matrix(target_pose1)
            T_right = pose_to_matrix(target_pose2)
            q0_left = np.array(current_joints[0]) if isinstance(current_joints[0], list) else np.array(current_joints[0])
            q0_right = np.array(current_joints[1]) if isinstance(current_joints[1], list) else np.array(current_joints[1])
            
            # Solve IK for both arms
            ik_result = self.robot_model.ik(
                target_left=T_left,
                target_right=T_right,
                q0_left=q0_left,
                q0_right=q0_right,
                method=method,
                coordination='indep',
                max_iters=max_iters,
                pos_tol=tolerance,
                ori_tol=tolerance,
            )
            
            # Execute motion if requested
            if execute and ik_result.get('success_left', False) and ik_result.get('success_right', False):
                # ik_result['q_left'][3] = -ik_result['q_left'][3]
                self.set_robot_state(target_joints=[ik_result['q_left'], ik_result['q_right']], arm="both", wait_for_completion=False, speed_deg_s=speed_deg_s)
            
            return {
                'success': ik_result.get('success_left', False) and ik_result.get('success_right', False),
                'success_left': ik_result.get('success_left', False),
                'success_right': ik_result.get('success_right', False),
                'q_left': ik_result.get('q_left'),
                'q_right': ik_result.get('q_right'),
                'res_left': ik_result.get('res_left'),
                'res_right': ik_result.get('res_right'),
            }
        
        else:
            return {'success': False, 'message': f'invalid arm: {arm}', 'q': None}




    # ==================== 夹爪控制 ====================

    def get_pose(self, arm: Optional[str] = None) -> Optional[Dict]:
        """Get current end-effector pose.

        :param arm: Arm to query, "left", "right", or "both" (default: "both")
        :return: Dictionary with transform, position, rotation, euler_xyz, quaternion_xyzw, output_to_ik. None if unavailable
        """
        if self.robot_model is None:
            logger.error("未安装 RoboCore 或未提供 robot_model，无法计算位姿")
            return None
        arm = arm or "both"
        if arm == "both":
            # Use get_robot_state to get joint angles
            joints_dict = self.get_robot_state("joint")
            if not joints_dict or not isinstance(joints_dict, dict):
                logger.error("无法获取关节角度")
                return None
            joints_l = joints_dict.get('left')
            joints_r = joints_dict.get('right')
            if not joints_l or not joints_r or not isinstance(joints_l, list) or not isinstance(joints_r, list):
                logger.error("无法获取关节角度")
                return None
            # Joint angles are already in radians from get_robot_state
            result = bimanual_forward_kinematics(
                self.left_model, self.right_model,
                np.array(joints_l), np.array(joints_r),
                return_end=True, mode='indep'
            )
            
            T_l, T_r = result['left'], result['right']
            pos_l, pos_r = T_l[:3, 3], T_r[:3, 3]
            rot_l, rot_r = T_l[:3, :3], T_r[:3, :3]
            euler_l, euler_r = matrix_to_euler(rot_l, seq='xyz'), matrix_to_euler(rot_r, seq='xyz')
            quat_l, quat_r = matrix_to_quaternion(rot_l), matrix_to_quaternion(rot_r)
            
            return {
                'transform': [T_l, T_r],
                'position': [pos_l, pos_r],
                'rotation': [rot_l, rot_r],
                'euler_xyz': [euler_l, euler_r],
                'quaternion_xyzw': [quat_l, quat_r],
                'output_to_ik': [
                    [pos_l[0], pos_l[1], pos_l[2], quat_l[0], quat_l[1], quat_l[2], quat_l[3]],
                    [pos_r[0], pos_r[1], pos_r[2], quat_r[0], quat_r[1], quat_r[2], quat_r[3]]
                ],
            }
        else:
            # Use get_robot_state to get joint angles
            joints_dict = self.get_robot_state("joint")
            if not joints_dict or not isinstance(joints_dict, dict):
                logger.error("无法获取关节角度")
                return None
            joints = joints_dict.get(arm)
            if not joints or not isinstance(joints, list):
                logger.error("无法获取关节角度")
                return None
            
            model = self.left_model if arm == "left" else self.right_model
            # Joint angles are already in radians from get_robot_state
            T_fk = forward_kinematics(model, np.array(joints), return_end=True)
            pos = T_fk[:3, 3]
            rot = T_fk[:3, :3]
            euler = matrix_to_euler(rot, seq='xyz')
            quat = matrix_to_quaternion(rot)
            
            return {
                'transform': T_fk,
                'position': pos,
                'rotation': rot,
                'euler_xyz': euler,
                'quaternion_xyzw': quat,
                'output_to_ik': [pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]],
            }

    def print_state(self, arm: Optional[str] = None, continuous: bool = False, output_format: str = "rad", fps: float = 200.0):
        """Print current robot state.

        :param arm: Arm to query, "left", "right", or "both" (default: "both")
        :param continuous: Print continuously if True, once if False
        :param output_format: Angle format, 'deg' or 'rad' (default: 'rad')
        :param fps: Target frames per second for continuous mode. Default 200 Hz
        """
        arm = arm or "both"
        
        def _print_once(arm):
            """Print robot state once."""
            # Get joint and gripper state together (like Alicia API)
            state = self.get_robot_state("joint_gripper")
            if state is None:
                logger.warning("无法获取关节状态")
                return
            
            # Extract joint angles and gripper from state
            # state is a dict with 'left' and 'right' JointState objects
            if arm == "both":
                if 'left' not in state or 'right' not in state:
                    logger.warning("无法获取双臂关节状态")
                    return
                left_state = state['left']
                right_state = state['right']
                joint_angles = [left_state.angles, right_state.angles]
                gripper = (left_state.gripper, right_state.gripper)
            elif arm == "left":
                if 'left' not in state:
                    logger.warning("无法获取左臂关节状态")
                    return
                left_state = state['left']
                joint_angles = left_state.angles
                gripper = left_state.gripper
            elif arm == "right":
                if 'right' not in state:
                    logger.warning("无法获取右臂关节状态")
                    return
                right_state = state['right']
                joint_angles = right_state.angles
                gripper = right_state.gripper
            else:
                logger.warning(f"Invalid arm parameter: {arm}")
                return
            
            # Get pose information
            pose = self.get_pose(arm=arm)
            if pose is None:
                logger.warning("无法获取末端执行器位姿")
                # Continue without pose data
            
            # Format conversion
            if output_format == 'deg':
                unit = "°"
                convert = 180.0 / np.pi
            else:
                unit = "rad"
                convert = 1.0
            
            # Display information grouped by arm
            if arm == "both":
                _print_dual_arm_state(joint_angles, pose, gripper, convert, unit, output_format)
            else:
                _print_single_arm_state(arm, joint_angles, pose, gripper, convert, unit, output_format)
            
            print("\n")
        
        def _print_dual_arm_state(joint_angles, pose, gripper, convert, unit, output_format):
            """Print state for both arms."""
            # Validate joint angles format
            if not isinstance(joint_angles, list) or len(joint_angles) != 2:
                logger.warning(f"Unexpected joint angles format for both arms: {type(joint_angles)}")
                return
            
            left_joints = np.array(joint_angles[0])
            right_joints = np.array(joint_angles[1])
            
            if len(left_joints) != 7 or len(right_joints) != 7:
                logger.warning(f"Invalid joint angles length: left={len(left_joints)}, right={len(right_joints)}")
                return
            
            # Left Arm Information
            left_joints_out = np.round(left_joints * convert, 2 if output_format == 'deg' else 3)
            logger.info(f"Left Arm 关节角度（{unit}): {left_joints_out.tolist()}")
            
            if pose is not None:
                position = pose.get('position', [])
                quaternion = pose.get('quaternion_xyzw', [])
                if isinstance(position, list) and len(position) == 2 and \
                   isinstance(quaternion, list) and len(quaternion) == 2:
                    pos_left = np.array(position[0])
                    quat_left = np.array(quaternion[0])
                    logger.info(f"Left Arm 位置(xyz /m): {np.round(pos_left, 3).tolist()}, "
                              f"四元数(qx, qy, qz, qw): {np.round(quat_left, 3).tolist()}")
            
            # Display gripper (no conversion, already in 0-100 range like Alicia API)
            if gripper is not None:
                if isinstance(gripper, tuple) and len(gripper) == 2:
                    logger.info(f"Left Arm 夹爪状态 (0-1000): {gripper[0]}")
                else:
                    logger.warning(f"Unexpected gripper format: {type(gripper)}")
            
            # Right Arm Information
            right_joints_out = np.round(right_joints * convert, 2 if output_format == 'deg' else 3)
            logger.info(f"Right Arm 关节角度（{unit}): {right_joints_out.tolist()}")
            
            if pose is not None:
                position = pose.get('position', [])
                quaternion = pose.get('quaternion_xyzw', [])
                if isinstance(position, list) and len(position) == 2 and \
                   isinstance(quaternion, list) and len(quaternion) == 2:
                    pos_right = np.array(position[1])
                    quat_right = np.array(quaternion[1])
                    logger.info(f"Right Arm 位置(xyz /m): {np.round(pos_right, 3).tolist()}, "
                              f"四元数(qx, qy, qz, qw): {np.round(quat_right, 3).tolist()}")
            
            if gripper is not None:
                if isinstance(gripper, tuple) and len(gripper) == 2:
                    logger.info(f"Right Arm 夹爪状态 (0-1000): {gripper[1]}")
        
        def _print_single_arm_state(arm, joint_angles, pose, gripper, convert, unit, output_format):
            """Print state for single arm."""
            # Validate joint angles format
            if not isinstance(joint_angles, list) or len(joint_angles) != 7:
                logger.warning(f"Unexpected joint angles format for {arm}: {type(joint_angles)}, length={len(joint_angles) if isinstance(joint_angles, list) else 'N/A'}")
                return
            
            joints_out = np.round(np.array(joint_angles) * convert, 2 if output_format == 'deg' else 3)
            arm_name = "Left" if arm == "left" else "Right"
            logger.info(f"{arm_name} Arm 关节角度（{unit}): {joints_out.tolist()}")
            
            if pose is not None:
                position = pose.get('position', [])
                quaternion = pose.get('quaternion_xyzw', [])
                if isinstance(position, (list, np.ndarray)) and isinstance(quaternion, (list, np.ndarray)):
                    pos = np.array(position)
                    quat = np.array(quaternion)
                    arm_name = "Left" if arm == "left" else "Right"
                    logger.info(f"{arm_name} Arm 位置(xyz /m): {np.round(pos, 3).tolist()}, "
                              f"四元数(qx, qy, qz, qw): {np.round(quat, 3).tolist()}")
            
            # Display gripper (no conversion, already in 0-100 range like Alicia API)
            if gripper is not None:
                if isinstance(gripper, (int, float)):
                    arm_name = "Left" if arm == "left" else "Right"
                    logger.info(f"{arm_name} Arm 夹爪状态 (0-1000): {gripper}")
                else:
                    logger.warning(f"Unexpected gripper format for {arm}: {type(gripper)}")
            else:
                logger.warning("无法获取夹爪状态")
        
        if continuous:
            # For high frequency (>= 100 Hz), use smaller spin_threshold for better efficiency
            # For 200 Hz (5ms interval), use 2ms spin_threshold to allow some sleep time
            interval = 1 / fps
            spin_threshold = 0.002 if interval <= 0.010 else 0.010  # 2ms for high freq, 10ms for low freq
            logger.info(f"开始连续状态打印，按 Ctrl+C 停止 (目标FPS: {fps})")
            try:
                while True:
                    start_time = time.perf_counter()
                    _print_once(arm)
                    dt_time = time.perf_counter() - start_time
                    precise_sleep(interval - dt_time, spin_threshold=spin_threshold)
            except KeyboardInterrupt:
                logger.info("停止连续状态打印")
        else:
            _print_once(arm)

    # ==================== 系统控制 ====================
    def set_speed(self, speed_deg_s: float) -> bool:
        """Set motion speed.

        :param speed_deg_s: Speed in degrees per second
        :return: True if successful
        """
        return self.servo_driver.set_speed_deg_s(speed_deg_s)


    def torque_control(self, command: str, arm: str = 'both') -> bool:
        """Enable or disable torque control.

        :param command: Command string, 'on' to enable or 'off' to disable
        :param arm: Arm to control, "left", "right", or "both"
        :return: True if successful
        """
        if command == 'on':
            return self.servo_driver.enable_torque(arm)
        elif command == 'off':
            return self.servo_driver.disable_torque(arm)
        else:
            logger.error("command 必须为 'on' 或 'off'")
            return False

    def set_zero(self, arm: str = 'both') -> bool:
        """Set zero position for specified arm(s).

        :param arm: Arm to set zero, "left", "right", or "both"
        :return: True if successful
        """
        logger.info(f"即将将{arm}关闭扭矩，请确定环境正常,输入enter继续...")
        #logging.info(f"即将将{arm}关闭扭矩，请确定环境正常,输入enter继续...")
        input()
        self.torque_control(command="off", arm=arm)
        logger.info(f"{arm}扭矩已关闭，请手动拖动机械臂到零点位置，然后按enter继续来设置该位置为零点...")
        input()
        result = self.servo_driver.set_zero_position(arm="both")
        self.torque_control(command="on", arm=arm)
        
        logger.info(f"{arm}归零成功: {result}")

    # ==================== 云台控制 ====================
    def set_gimbal(
        self,
        x_angle: float,
        y_angle: float,
        angle_format: str = "deg",
        wait_for_completion: bool = False,
        timeout: float = 5.0,
        tolerance: float = 3.0,
    ) -> bool:
        """Set gimbal X/Y axis angles.

        :param x_angle: X-axis angle (pan)
        :param y_angle: Y-axis angle (tilt)
        :param angle_format: Angle unit format, "deg" (degrees) or "rad" (radians). Default: "deg"
        :param wait_for_completion: Wait until gimbal reaches target position if True
        :param timeout: Maximum wait time in seconds (only used if wait_for_completion=True)
        :param tolerance: Acceptable difference to target value. Unit depends on angle_format:
                          - If angle_format="deg": tolerance in degrees (default: 3.0)
                          - If angle_format="rad": tolerance in radians (default: 0.05)
        :return: True if command sent successfully
        """
        if not isinstance(x_angle, (int, float)) or not isinstance(y_angle, (int, float)):
            logger.error("set_gimbal: 角度应为 float 类型")
            return False
        
        if angle_format.lower() not in ("deg", "degree", "degrees", "rad", "radian", "radians"):
            logger.error(f"set_gimbal: angle_format 必须为 'deg' 或 'rad'，当前: {angle_format}")
            return False
        
        # 根据 angle_format 转换为弧度
        if angle_format.lower() in ("deg", "degree", "degrees"):
            # 度制输入，转换为弧度
            x_angle_rad = x_angle * self.servo_driver.DEG_TO_RAD
            y_angle_rad = y_angle * self.servo_driver.DEG_TO_RAD
            tolerance_rad = tolerance * self.servo_driver.DEG_TO_RAD
        else:
            # 弧度制输入，直接使用
            x_angle_rad = x_angle
            y_angle_rad = y_angle
            tolerance_rad = tolerance
        
        return self.servo_driver.set_gimbal(
            x_angle_rad=x_angle_rad,
            y_angle_rad=y_angle_rad,
            wait_for_completion=wait_for_completion,
            timeout=timeout,
            tolerance=tolerance_rad
        )

    # ==================== 轨迹规划接口 ====================
    
    def plan_joint_trajectory(
        self,
        waypoints: Union[np.ndarray, Dict[str, np.ndarray]],
        planner_type: str = 'b_spline',
        duration: Optional[float] = None,
        num_points: int = 800,
        bspline_degree: int = 5,
        segment_method: str = 'quintic',
        duration_per_segment: Optional[float] = None,
        num_points_per_segment: int = 100,
        gripper_waypoints: Optional[Union[np.ndarray, Dict[str, np.ndarray]]] = None,
        arm: str = "both"
    ) -> Dict[str, Any]:
        """Plan joint space trajectory through waypoints (supports single-arm and dual-arm).

        :param waypoints: For single arm: Array [n_waypoints, n_dof] in radians.
                          For dual arm: Dict with 'left' and 'right' keys, each [n_waypoints, n_dof]
        :param planner_type: Planner type, 'b_spline' or 'multi_segment'
        :param duration: Total trajectory duration in seconds (for B-Spline)
        :param num_points: Number of points in trajectory (for B-Spline)
        :param bspline_degree: B-Spline degree, 3 (cubic) or 5 (quintic)
        :param segment_method: Multi-segment method, 'cubic' or 'quintic'
        :param duration_per_segment: Duration per segment in seconds (for Multi-Segment)
        :param num_points_per_segment: Number of points per segment (for Multi-Segment)
        :param gripper_waypoints: For single arm: Optional array [n_waypoints] (0-1000).
                                  For dual arm: Optional Dict with 'left' and 'right' keys, each [n_waypoints]
        :param arm: Arm mode, "left", "right", or "both" (default: "both")
        :return: Dictionary with trajectory data including 't', 'q', 'qd', 'qdd', and optionally 'gripper'
        """
        from robocore.planning import BSplinePlanner, MultiSegmentPlanner
        from robocore.utils.backend import to_numpy
                
        is_dual_arm = isinstance(waypoints, dict)
                
        if is_dual_arm:
            # Dual-arm mode
            waypoints_left = to_numpy(waypoints['left'])
            waypoints_right = to_numpy(waypoints['right'])
            
            if waypoints_left.ndim == 1:
                waypoints_left = waypoints_left.reshape(1, -1)
            if waypoints_right.ndim == 1:
                waypoints_right = waypoints_right.reshape(1, -1)
            
            if len(waypoints_left) < 2 or len(waypoints_right) < 2:
                raise ValueError("Need at least 2 waypoints for each arm")
            
            # Create planner
            if planner_type == 'b_spline':
                planner = BSplinePlanner(degree=bspline_degree)
            elif planner_type == 'multi_segment':
                planner = MultiSegmentPlanner(method=segment_method)
            else:
                raise ValueError(f"Unknown planner type: {planner_type}. Must be 'b_spline' or 'multi_segment'")
            
            # Plan trajectory for both arms
            if planner_type == 'b_spline':
                traj_left = planner.plan(waypoints=waypoints_left, duration=duration, num_points=num_points)
                traj_right = planner.plan(waypoints=waypoints_right, duration=duration, num_points=num_points)
            else:  # multi_segment
                if duration_per_segment is None:
                    duration_per_segment = 1.0
                traj_left = planner.plan(waypoints=waypoints_left, durations=duration_per_segment, num_points_per_segment=num_points_per_segment)
                traj_right = planner.plan(waypoints=waypoints_right, durations=duration_per_segment, num_points_per_segment=num_points_per_segment)
            
            # Combine trajectories
            trajectory = {
                't': to_numpy(traj_left['t']),
                'q_left': to_numpy(traj_left['q']),
                'q_right': to_numpy(traj_right['q']),
                'qd_left': to_numpy(traj_left['qd']),
                'qd_right': to_numpy(traj_right['qd']),
                'qdd_left': to_numpy(traj_left['qdd']),
                'qdd_right': to_numpy(traj_right['qdd']),
                'waypoints_left': waypoints_left,
                'waypoints_right': waypoints_right,
            }
            
            # Interpolate gripper values if provided
            if gripper_waypoints is not None:
                if isinstance(gripper_waypoints, dict):
                    g_left = to_numpy(gripper_waypoints.get('left'))
                    g_right = to_numpy(gripper_waypoints.get('right'))
                    
                    t_waypoints = np.linspace(0, trajectory['t'][-1], len(g_left))
                    t_traj = to_numpy(trajectory['t'])
                    gripper_left = np.interp(t_traj, t_waypoints, g_left)
                    gripper_right = np.interp(t_traj, t_waypoints, g_right)
                    gripper_left = np.clip(gripper_left, 0, 1000)
                    gripper_right = np.clip(gripper_right, 0, 1000)
                    trajectory['gripper_left'] = gripper_left
                    trajectory['gripper_right'] = gripper_right
                else:
                    raise ValueError("For dual-arm mode, gripper_waypoints must be a dict with 'left' and 'right' keys")
        else:
            # Single-arm mode
            waypoints = to_numpy(waypoints)
            if waypoints.ndim == 1:
                waypoints = waypoints.reshape(1, -1)
            
            if len(waypoints) < 2:
                raise ValueError("Need at least 2 waypoints")
            
            # Create planner
            if planner_type == 'b_spline':
                planner = BSplinePlanner(degree=bspline_degree)
            elif planner_type == 'multi_segment':
                planner = MultiSegmentPlanner(method=segment_method)
            else:
                raise ValueError(f"Unknown planner type: {planner_type}. Must be 'b_spline' or 'multi_segment'")
            
            # Plan trajectory
            if planner_type == 'b_spline':
                trajectory = planner.plan(waypoints=waypoints, duration=duration, num_points=num_points)
            else:  # multi_segment
                if duration_per_segment is None:
                    duration_per_segment = 1.0
                trajectory = planner.plan(waypoints=waypoints, durations=duration_per_segment, num_points_per_segment=num_points_per_segment)
            
            # Convert to numpy
            for key in ['t', 'q', 'qd', 'qdd']:
                if key in trajectory:
                    trajectory[key] = to_numpy(trajectory[key])
                
            # Interpolate gripper values if provided
            if gripper_waypoints is not None:
                gripper_waypoints = to_numpy(gripper_waypoints)
                t_waypoints = np.linspace(0, trajectory['t'][-1], len(gripper_waypoints))
                t_traj = to_numpy(trajectory['t'])
                gripper_trajectory = np.interp(t_traj, t_waypoints, gripper_waypoints)
                gripper_trajectory = np.clip(gripper_trajectory, 0, 1000)
                trajectory['gripper'] = gripper_trajectory
            
            # Add waypoints to trajectory for reference
            trajectory['waypoints'] = waypoints
        
        return trajectory

 


    def _wait_for_joint_target(
        self,
        target_joints: Optional[Union[List[float], List[List[float]]]],
        arm: str,
        tolerance: float,
        timeout: float,
        log_prefix: str = "等待关节接近目标"
    ) -> bool:
        """Wait until all joints reach target angles.

        :param target_joints: Target joint angles in radians. 
            - For single arm: List[float] (7 angles)
            - For dual arm: List[List[float]] (2x7 angles)
            - If None, returns True immediately
        :param arm: Arm to wait for, "left", "right", or "both"
        :param tolerance: Rad, acceptable abs distance to target for all joints
        :param timeout: Seconds, maximum wait time
        :param log_prefix: Log message prefix
        :return: True if target reached, False if timeout
        """
        # If no joint target is specified, there is nothing to wait for.
        if target_joints is None:
            logger.debug("No joint target specified, skip joint waiting.")
            return True

        start_time = time.time()
        check_interval = 0.05  # Check every 50ms
        last_check_time = 0

        while time.time() - start_time < timeout:
            current_time = time.time()
            # Only check state at intervals to avoid excessive polling
            if current_time - last_check_time < check_interval:
                time.sleep(0.01)  # Small sleep to avoid busy waiting
                continue

            last_check_time = current_time
            joints_dict = self.get_robot_state("joint")
            if joints_dict is not None:
                if arm == "both":
                    if not isinstance(target_joints, list) or len(target_joints) != 2:
                        logger.error("Invalid target_joints format for both arms")
                        return False
                    if not isinstance(joints_dict, dict):
                        continue
                    current_joints_left = joints_dict.get('left')
                    current_joints_right = joints_dict.get('right')
                    if current_joints_left is not None and current_joints_right is not None:
                        if isinstance(current_joints_left, list) and isinstance(current_joints_right, list):
                            if len(current_joints_left) == 7 and len(current_joints_right) == 7:
                                # Check both arms - all joints must be within tolerance
                                left_reached = all(abs(a - b) <= tolerance for a, b in zip(current_joints_left, target_joints[0]))
                                right_reached = all(abs(a - b) <= tolerance for a, b in zip(current_joints_right, target_joints[1]))
                                if left_reached and right_reached:
                                    return True
                else:
                    if not isinstance(target_joints, list) or len(target_joints) != 7:
                        logger.error(f"Invalid target_joints format for {arm} arm")
                        return False
                    if not isinstance(joints_dict, dict):
                        continue
                    current_joints = joints_dict.get(arm)
                    if current_joints is not None:
                        if isinstance(current_joints, list) and len(current_joints) == 7:
                            if all(abs(a - b) <= tolerance for a, b in zip(current_joints, target_joints)):
                                return True
            
        # Timeout - get final state for logging
        logger.warning("等待关节到目标附近超时")
        joints_dict = self.get_robot_state("joint")
        if arm == "both":
            if joints_dict is not None and isinstance(joints_dict, dict):
                current_joints = [joints_dict.get('left', []), joints_dict.get('right', [])]
                # Calculate and log errors for debugging
                if len(current_joints) == 2 and len(current_joints[0]) == 7 and len(current_joints[1]) == 7:
                    left_errors = [abs(a - b) for a, b in zip(current_joints[0], target_joints[0])]
                    right_errors = [abs(a - b) for a, b in zip(current_joints[1], target_joints[1])]
                    max_left_error = max(left_errors)
                    max_right_error = max(right_errors)
                    logger.warning(f"左臂最大误差: {max_left_error:.4f} rad (tolerance: {tolerance:.4f})")
                    logger.warning(f"右臂最大误差: {max_right_error:.4f} rad (tolerance: {tolerance:.4f})")
            else:
                current_joints = [[], []]
        else:
            if joints_dict is not None and isinstance(joints_dict, dict):
                current_joints = joints_dict.get(arm, [])
                # Calculate and log errors for debugging
                if isinstance(current_joints, list) and len(current_joints) == 7:
                    errors = [abs(a - b) for a, b in zip(current_joints, target_joints)]
                    max_error = max(errors)
                    logger.warning(f"{arm}臂最大误差: {max_error:.4f} rad (tolerance: {tolerance:.4f})")
            else:
                current_joints = []
        logger.warning(f"目标关节角度: {target_joints}")
        logger.warning(f"关节角度: {current_joints}")

        return False

    def _generate_random_q(self, scale: float = 0.5) -> List[float]:
        """Generate random joint configuration within limits.

        :param scale: Range scale factor within joint limits
        :return: Random joint angles in radians (7 DOF for single arm)
        """
        if not hasattr(self, 'robot_model') or self.robot_model is None:
            logger.warning("未提供 robot_model，使用默认关节范围生成随机值")
            rng = np.random.default_rng()
            return [float(rng.uniform(-1.0, 1.0)) for _ in range(7)]

        rng = np.random.default_rng()
        q = [0.0] * self.robot_model.num_dof()

        for js in self.robot_model._actuated:
            lo, hi = -1.0, 1.0
            if js.limit:
                if js.limit[0] is not None:
                    lo = js.limit[0]
                if js.limit[1] is not None:
                    hi = js.limit[1]
            mid = 0.5 * (lo + hi)
            span = 0.5 * (hi - lo) * scale
            q[js.index] = float(rng.uniform(mid - span, mid + span))

        return q

    def _get_gripper_type_with_cache(self, timeout: float = 1.0) -> Optional[str]:
        """Get gripper type with caching support.
        
        First tries to load from JSON cache, then queries hardware if needed.
        Returns None with warning if hardware query fails (non-critical).
        
        :param timeout: Maximum time to wait for hardware response in seconds
        :return: Gripper type name (e.g., "50mm" or "100mm"), or None if unavailable
        """
        # JSON file path in the same folder as this module
        current_dir = os.path.dirname(os.path.abspath(__file__))
        json_file_path = os.path.join(current_dir, "gripper_type.json")
        
        # 1) Try to load cached gripper type from JSON file (no serial communication)
        if os.path.exists(json_file_path):
            try:
                with open(json_file_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                cached_type = data.get("type_name")
                if isinstance(cached_type, str) and cached_type:
                    return cached_type
            except Exception as e:
                logger.warning(f"Failed to load cached gripper type from JSON, will try hardware query: {e}")
        
        # 2) If no valid cache, actively query hardware
        # Try with wait=True first to get response reliably
        if not self.servo_driver.acquire_info("gripper_type", wait=True, timeout=timeout):
            logger.warning("Failed to get gripper_type data within timeout period")
            return None
        
        result = self.data_parser.get_info("gripper_type")
        if result is None:
            logger.warning("Gripper type (50mm or 100mm) should be defined by parameters")
            return None
        
        # Save to JSON file for future use
        self._save_gripper_type_to_json(result)
        
        return result
    
    def _save_gripper_type_to_json(self, gripper_type: str):
        """Save gripper type to JSON file in the same folder as this module."""
        current_dir = os.path.dirname(os.path.abspath(__file__))
        json_file_path = os.path.join(current_dir, "gripper_type.json")
        
        try:
            with open(json_file_path, 'w', encoding='utf-8') as f:
                json.dump({"type_name": gripper_type}, f, indent=2, ensure_ascii=False)
            if hasattr(self.servo_driver, 'debug_mode') and self.servo_driver.debug_mode:
                logger.debug(f"Saved gripper type '{gripper_type}' to {json_file_path}")
        except Exception as e:
            logger.error(f"Failed to save gripper type to JSON file: {e}")

