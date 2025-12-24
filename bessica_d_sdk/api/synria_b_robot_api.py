"""
SynriaBessicaRobotAPI - User-level API (Bessica)

职责:
- 提供统一的上层用户接口
- 关节/夹爪/云台等高层控制封装
- 状态查询接口
- 系统控制函数
- 参数校验与最小插值运动

说明:
- 适配 Bessica 的 ServoDriver：单臂7关节、需显式传入 arm("left_arm"/"right_arm")；支持双臂 both
- 尽量对齐 Alicia 的 SynriaRobotAPI 方法命名，以便后续规范化统一
"""

import time
from typing import List, Optional, Dict, Union, Tuple
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
# from robocore.planning.trajectory import (
#     cubic_polynomial_trajectory,
#     quintic_polynomial_trajectory,
#     linear_joint_trajectory,
#     linear_cartesian_trajectory,
#     circular_cartesian_trajectory,
#     cartesian_waypoint_trajectory
# )
from ..utils.logger import logger
from ..hardware import ServoDriver
# from ..execution import HardwareExecutor, JointPlanner
# from ..utils.logger import logger
# logger = logging.getLogger("SynriaBessicaRobotAPI")
# from robocore.utils.control_utils import compute_steps_and_delay, validate_joint_list


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
        
        model_path = str(get_model_path("Bessica_D", version=robot_version, variant=variant))

        self.robot_model = BimanualRobotModel(model_path, left_end_link=left_end_link, right_end_link=right_end_link)
        
        # Access left and right arm models from bimanual model
        self.left_model = self.robot_model.left_model
        self.right_model = self.robot_model.right_model
        
        self.speed_deg_s = speed_deg_s
        # self.hardware_executor = HardwareExecutor(servo_driver)
        # self.joint_planner = JointPlanner()
        self.home_angles = [0.0] * 7
        self.default_arm = "both"

    # ==================== Connection Management ====================

    def connect(self) -> bool:
        """Connect to robot.

        :return: True if connection successful
        """
        result = self.servo_driver.connect()
        return result

    def disconnect(self):
        """Disconnect from robot and stop update threads."""
        self.servo_driver.stop_update_thread()
        self.servo_driver.disconnect()

    # ==================== 关节控制 ====================
    def set_home(self, arm: str = "both") -> bool:
        """Move robot to home position.

        :param arm: Arm to control, "left_arm", "right_arm", or "both"
        :return: True if successful
        """
        logger.info(f"set_home at head: arm={arm}")
        home_angles = [0.0] * 7

        if arm == "both":
            # Dual-arm: use unified set_joint_target, joint_format in radians
            success = self.set_joint_target(
                target_joints=[home_angles, home_angles],
                arm="both",
                joint_format="rad",
                wait=False,
                tolerance=0.05,
            )
            success &= self.set_gripper_target(command="open", arm="both")
            return success

        elif arm in ("left_arm", "right_arm"):
            # Single arm: move to home and open gripper
            ok = self.set_joint_target(
                target_joints=home_angles,
                arm=arm,
                joint_format="rad",
                wait=False,
                tolerance=0.05,
            )
            ok &= self.set_gripper_target(command="open", arm=arm)
            return ok

        else:
            logger.error(f"set_home: 非法 arm={arm}")
            return False

    def set_joint_target(
        self,
        target_joints: Union[List[float], List[List[float]]],
        arm: Optional[str] = None,
        joint_format: str = "rad",
        wait: bool = False,
        tolerance: float = 0.0524,
    ) -> bool:
        """Move robot to target joint angles.

        :param target_joints: Target joint angles. For single arm: List[float] (7 angles). For dual arm: List[List[float]] (2x7 angles)
        :param arm: Arm to control, "left_arm", "right_arm", or "both" (default: self.default_arm)
        :param joint_format: Unit format, "deg" or "rad" (default: "rad")
        :param wait: Wait for motion completion if True
        :param tolerance: Maximum allowed error per joint in same unit as joint_format (default: 0.0524 rad ≈ 3.0 deg)
        :return: True if command sent successfully
        """
        arm = arm or self.default_arm
        is_deg = joint_format.lower() in ("deg", "degree", "degrees")
        is_rad = joint_format.lower() in ("rad", "radian", "radians")
        
        if not (is_deg or is_rad):
            logger.error(f"set_joint_target: joint_format 必须是 'deg' 或 'rad'，当前: {joint_format}")
            return False
        
        # Convert to radians if needed
        convert = self.servo_driver.DEG_TO_RAD if is_deg else 1.0
        tolerance_rad = tolerance * convert if is_deg else tolerance
        
        if arm == "both":
            if not isinstance(target_joints, list) or len(target_joints) != 2:
                logger.error("set_joint_target: arm='both' 时，target_joints 必须是包含2个列表的列表")
                return False
            
            target_left, target_right = target_joints[0], target_joints[1]
            if not (isinstance(target_left, list) and len(target_left) == 7 and 
                    isinstance(target_right, list) and len(target_right) == 7):
                logger.error("set_joint_target: 左右臂都必须提供7个关节角度")
                return False
            
            target_left_rad = [a * convert for a in target_left]
            target_right_rad = [a * convert for a in target_right]
            
            success = self.servo_driver.set_joint_angles(
                joint_angles=target_left_rad, arm="left_arm", 
                wait_for_completion=wait, tolerance=tolerance_rad
            )
            success &= self.servo_driver.set_joint_angles(
                joint_angles=target_right_rad, arm="right_arm", 
                wait_for_completion=wait, tolerance=tolerance_rad
            )
            return success
            
        elif arm in ("left_arm", "right_arm"):
            if not isinstance(target_joints, list) or len(target_joints) != 7:
                logger.error(f"set_joint_target: 单臂模式必须提供7个关节角度，但得到 {len(target_joints) if isinstance(target_joints, list) else '非列表'}")
                return False
            
            target_rad = [a * convert for a in target_joints]
            return self.servo_driver.set_joint_angles(
                joint_angles=target_rad, arm=arm, 
                wait_for_completion=wait, tolerance=tolerance_rad
            )
        else:
            logger.error(f"set_joint_target: 非法 arm={arm}")
            return False

    def set_pose_target(self,
                        target_pose1: List[float],
                        target_pose2: Optional[List[float]] = None,
                        arm: Optional[str] = None,
                        method: str = 'dls',
                        tolerance: float = 1e-3,
                        max_iters: int = 100,
                        execute: bool = True) -> Dict:
        """Move end-effector to target pose using inverse kinematics.

        :param target_pose1: Target pose as [x, y, z, qx, qy, qz, qw]. For single arm: used for specified arm. For both: left arm.
        :param target_pose2: Target pose for right arm as [x, y, z, qx, qy, qz, qw] (required when arm="both")
        :param arm: Arm to control, "left_arm", "right_arm", or "both" (default: self.default_arm)
        :param method: IK solver method, 'dls', 'pinv', or 'transpose'
        :param tolerance: Position and orientation tolerance
        :param max_iters: Maximum number of iterations
        :param execute: Execute motion if True
        :return: Dictionary with success, q, iters, pos_err, ori_err, message
        """
        if not hasattr(self, 'robot_model') or self.robot_model is None:
            return {'success': False, 'message': 'robot_model not available', 'q': None}
        
        arm = arm or self.default_arm
        
        # Get current joint positions as initial guess
        current_joints = self.get_joints(arm=arm)
        if current_joints is None:
            current_joints = [0.0] * 7 if arm != "both" else [[0.0] * 7, [0.0] * 7]
        
        # Convert pose to transformation matrix
        def pose_to_matrix(pose: List[float]) -> np.ndarray:
            pos = np.array(pose[:3])
            quat = np.array(pose[3:])
            rot = quaternion_to_matrix(quat)
            return make_transform(rot, pos)
        
        # Handle single arm case
        if arm in ("left_arm", "right_arm"):
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
            
            result = ik_result['res_left'] if arm == "left_arm" else ik_result['res_right']
            
            # Execute motion if requested
            if execute and result.get('success', False):
                print(f"result: {result}")
                self.set_joint_target(result['q'], arm=arm, wait=False, joint_format="rad")
            
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
                self.set_joint_target([ik_result['q_left'], ik_result['q_right']], arm="both", wait=False)
            
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
    def set_gripper_target(
        self,
        arm: str,
        command: Optional[str] = None,
        value: Optional[float] = None,
        wait_for_completion: bool = False,
        timeout: float = 1.0,
        tolerance: float = 0.1,
    ) -> bool:
        """Control gripper position.

        :param arm: Arm to control, "left_arm", "right_arm", or "both"
        :param command: Command string, 'open' or 'close'
        :param value: Gripper value in degrees, 0 (closed) to 100 (open)
        :param wait_for_completion: Wait until gripper reaches target
        :param timeout: Maximum wait time in seconds
        :param tolerance: Acceptable difference to target value in degrees
        :return: True if successful
        """
        arm = arm or self.default_arm
        if (command is None) == (value is None):
            logger.error("必须二选一提供 command 或 value")
            return False
        if command is not None:
            if command == 'open':
                value = 0.1
            elif command == 'close':
                value = 99.9
            else:
                logger.error("command 仅支持 'open'/'close'")
                return False
        if value is None:
            logger.error("必须提供 command 或 value 之一")
            return False
        if arm in ("left_arm", "right_arm"):
            return self.servo_driver.set_gripper_deg(arm=arm, angle_deg=float(value), wait=wait_for_completion)
        elif arm == "both":
            return self.servo_driver.set_gripper_deg(angle_deg=float(value), arm="both", wait=wait_for_completion)
        else:
            logger.error(f"set_gripper_target: 非法 arm={arm}")
            return False

    # ==================== 位姿与状态 ====================
    def get_joints(self, arm: Optional[str] = None) -> Optional[Union[List[float], List[List[float]]]]:
        """Get current joint angles.

        :param arm: Arm to query, "left_arm", "right_arm", or "both" (default: self.default_arm)
        :return: Joint angles in radians. For single arm: List[float] (7 angles). For dual arm: List[List[float]] (2x7 angles). None if unavailable
        """
        self.default_arm = "both"
        arm = arm or self.default_arm
        joint_angles = self.servo_driver.read_joint_angles(arm)
        # logger.info(f"{arm}'s joint_angles: {joint_angles}")
        return joint_angles

    def get_gripper(self, arm: Optional[str] = None) -> Optional[Union[float, Tuple[float, float]]]:
        """Get current gripper position.

        :param arm: Arm to query, "left_arm", "right_arm", or "both" (default: self.default_arm)
        :return: Gripper position in degrees. For single arm: float. For dual arm: Tuple[float, float]. None if unavailable
        """
        arm = arm or self.default_arm
        try:
            return self.servo_driver.read_gripper_data(arm if arm in ['left_arm', 'right_arm'] else 'both')
        except Exception:
            return None

    def get_pose(self, arm: Optional[str] = None) -> Optional[Dict]:
        """Get current end-effector pose.

        :param arm: Arm to query, "left_arm", "right_arm", or "both" (default: self.default_arm)
        :return: Dictionary with transform, position, rotation, euler_xyz, quaternion_xyzw, output_to_ik. None if unavailable
        """
        if self.robot_model is None:
            logger.error("未安装 RoboCore 或未提供 robot_model，无法计算位姿")
            return None
        arm = arm or self.default_arm
        if arm == "both":
            joints_l = self.get_joints(arm="left_arm")
            joints_r = self.get_joints(arm="right_arm")
            if not joints_l or not joints_r or not isinstance(joints_l, list) or not isinstance(joints_r, list):
                logger.error("无法获取关节角度")
                return None
            # Joint angles are already in radians from get_joints()
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
            joints = self.get_joints(arm=arm)
            if not joints or not isinstance(joints, list):
                logger.error("无法获取关节角度")
                return None
            
            model = self.left_model if arm == "left_arm" else self.right_model
            # Joint angles are already in radians from get_joints()
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
        :param arm: Arm to control, "left_arm", "right_arm", or "both"
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

        :param arm: Arm to set zero, "left_arm", "right_arm", or "both"
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

    # ==================== 笛卡尔控制接口（可用 RoboCore 时增强） ====================
    def move_joint_trajectory(
        self,
        q_end: Union[List[float], List[List[float]]],
        arm: Optional[str] = None,
        duration: float = 2.0,
        method: str = 'cubic',
        num_points: int = 100,
        joint_format: str = 'rad',
        visualize: bool = False,
    ) -> bool:
        """Move robot along joint space trajectory to target.

        :param q_end: Target joint angles. For single arm: List[float] (7 angles). For dual arm: List[List[float]] (2x7 angles)
        :param arm: Arm to control, "left_arm", "right_arm", or "both" (default: self.default_arm)
        :param duration: Trajectory duration in seconds
        :param method: Interpolation method, 'linear', 'cubic', or 'quintic'
        :param num_points: Number of trajectory points
        :param joint_format: Unit format, 'rad' or 'deg'
        :param visualize: Enable trajectory visualization (not implemented)
        :return: True if successful
        """
        arm = arm or self.default_arm
        
        # 判断输入格式：一维数组还是二维数组
        is_dual_arm_input = False
        if isinstance(q_end, list) and len(q_end) > 0:
            if isinstance(q_end[0], list):
                is_dual_arm_input = True
            elif arm == "both":
                logger.error("arm='both' 时，q_end 必须是二维数组 [[left_7_joints], [right_7_joints]]")
                return False
        
        # 处理单臂模式
        if arm in ("left_arm", "right_arm"):
            if is_dual_arm_input:
                logger.warning(f"单臂模式但输入了二维数组，使用第一个元素")
                q_end = q_end[0]
            
            # 转换为弧度
            if joint_format == 'deg':
                q_end = [a * np.pi / 180.0 for a in q_end]
            
            if len(q_end) != 7:
                logger.error(f"单臂模式需要7个关节角度，但得到 {len(q_end)} 个")
                return False
            
            q_start = self.get_joints(arm=arm)
            if not q_start or not isinstance(q_start, list) or len(q_start) != 7:
                logger.error("无法获取当前关节角度")
                return False
            
            try:
                import numpy as _np
                from robocore.planning.trajectory import (
                    linear_joint_trajectory as _linear_joint_trajectory,
                    cubic_polynomial_trajectory as _cubic_polynomial_trajectory,
                    quintic_polynomial_trajectory as _quintic_polynomial_trajectory,
                )
                q_start_np = _np.array(q_start)
                q_end_np = _np.array(q_end)
                if method == 'linear':
                    _, q_traj, _, _ = _linear_joint_trajectory(q_start_np, q_end_np, duration, num_points)
                elif method == 'cubic':
                    _, q_traj, _, _ = _cubic_polynomial_trajectory(q_start_np, q_end_np, duration, num_points)
                elif method == 'quintic':
                    _, q_traj, _, _ = _quintic_polynomial_trajectory(q_start_np, q_end_np, duration, num_points)
                else:
                    logger.error(f"不支持的插值方法: {method}")
                    return False
                delay = duration / num_points
                for q in q_traj.tolist():
                    if not self.servo_driver.set_joint_angles(q, arm=arm, wait_for_completion=False):
                        return False
                    time.sleep(delay)
                return True
            except Exception as e:
                # 退化：简单线性插值
                logger.warning(f"使用简单线性插值（RoboCore 不可用: {e}）")
                steps = max(2, int(num_points))
                delay = duration / steps
                for s in range(1, steps + 1):
                    r = s / steps
                    q = [a + (b - a) * r for a, b in zip(q_start, q_end)]
                    if not self.servo_driver.set_joint_angles(q, arm=arm, wait_for_completion=False):
                        return False
                    time.sleep(delay)
                return True
        
        # 处理双臂模式
        elif arm == "both":
            if not is_dual_arm_input:
                logger.error("arm='both' 时，q_end 必须是二维数组 [[left_7_joints], [right_7_joints]]")
                return False
            
            if len(q_end) != 2 or len(q_end[0]) != 7 or len(q_end[1]) != 7:
                logger.error("双臂模式需要 [[left_7_joints], [right_7_joints]] 格式")
                return False
            
            q_end_left = q_end[0]
            q_end_right = q_end[1]
            
            # 转换为弧度
            if joint_format == 'deg':
                q_end_left = [a * np.pi / 180.0 for a in q_end_left]
                q_end_right = [a * np.pi / 180.0 for a in q_end_right]
            
            q_start = self.get_joints(arm="both")
            if not q_start or not isinstance(q_start, list) or len(q_start) != 2:
                logger.error("无法获取当前关节角度（双臂）")
                return False
            
            q_start_left = q_start[0]
            q_start_right = q_start[1]
            
            try:
                import numpy as _np
                from robocore.planning.trajectory import (
                    linear_joint_trajectory as _linear_joint_trajectory,
                    cubic_polynomial_trajectory as _cubic_polynomial_trajectory,
                    quintic_polynomial_trajectory as _quintic_polynomial_trajectory,
                )
                
                # 为左右臂分别生成轨迹
                q_start_left_np = _np.array(q_start_left)
                q_end_left_np = _np.array(q_end_left)
                q_start_right_np = _np.array(q_start_right)
                q_end_right_np = _np.array(q_end_right)
                
                if method == 'linear':
                    _, q_traj_left, _, _ = _linear_joint_trajectory(q_start_left_np, q_end_left_np, duration, num_points)
                    _, q_traj_right, _, _ = _linear_joint_trajectory(q_start_right_np, q_end_right_np, duration, num_points)
                elif method == 'cubic':
                    _, q_traj_left, _, _ = _cubic_polynomial_trajectory(q_start_left_np, q_end_left_np, duration, num_points)
                    _, q_traj_right, _, _ = _cubic_polynomial_trajectory(q_start_right_np, q_end_right_np, duration, num_points)
                elif method == 'quintic':
                    _, q_traj_left, _, _ = _quintic_polynomial_trajectory(q_start_left_np, q_end_left_np, duration, num_points)
                    _, q_traj_right, _, _ = _quintic_polynomial_trajectory(q_start_right_np, q_end_right_np, duration, num_points)
                else:
                    logger.error(f"不支持的插值方法: {method}")
                    return False
                
                delay = duration / num_points
                for q_left, q_right in zip(q_traj_left.tolist(), q_traj_right.tolist()):
                    # 同时发送左右臂指令
                    success_left = self.servo_driver.set_joint_angles(q_left, arm="left_arm", wait_for_completion=False)
                    success_right = self.servo_driver.set_joint_angles(q_right, arm="right_arm", wait_for_completion=False)
                    if not (success_left and success_right):
                        return False
                    time.sleep(delay)
                return True
            except Exception as e:
                # 退化：简单线性插值
                logger.warning(f"使用简单线性插值（RoboCore 不可用: {e}）")
                steps = max(2, int(num_points))
                delay = duration / steps
                for s in range(1, steps + 1):
                    r = s / steps
                    q_left = [a + (b - a) * r for a, b in zip(q_start_left, q_end_left)]
                    q_right = [a + (b - a) * r for a, b in zip(q_start_right, q_end_right)]
                    success_left = self.servo_driver.set_joint_angles(q_left, arm="left_arm", wait_for_completion=False)
                    success_right = self.servo_driver.set_joint_angles(q_right, arm="right_arm", wait_for_completion=False)
                    if not (success_left and success_right):
                        return False
                    time.sleep(delay)
                return True
        else:
            logger.error(f"非法的 arm 参数: {arm}")
            return False

    def move_cartesian_linear(
        self,
        target_pose: Union[List[float], List[List[float]]],
        arm: Optional[str] = None,
        target_pose_second_arm: Optional[List[float]] = None,
        duration: float = 2.0,
        num_points: int = 50,
        ik_method: str = 'dls',
        visualize: bool = False,
    ) -> bool:
        """Move end-effector along linear Cartesian trajectory to target pose.

        Requires robot_model and RoboCore; returns False if unavailable.

        :param target_pose: Target pose as [x, y, z, qx, qy, qz, qw]. For single arm: List[float] (7 elements). For dual arm: List[List[float]] (2x7 elements) or List[float] with target_pose_second_arm
        :param arm: Arm to control, "left_arm", "right_arm", or "both" (default: self.default_arm)
        :param target_pose_second_arm: Target pose for second arm (required when arm="both" and target_pose is 1D)
        :param duration: Trajectory duration in seconds
        :param num_points: Number of trajectory points
        :param ik_method: IK solver method, 'dls', 'pinv', or 'transpose'
        :param visualize: Enable trajectory visualization (not implemented)
        :return: True if successful
        """
        if self.robot_model is None:
            logger.error("未提供 robot_model，无法执行笛卡尔轨迹")
            return False
        
        arm = arm or self.default_arm
        
        # 判断输入格式
        is_dual_arm_input = False
        if isinstance(target_pose, list) and len(target_pose) > 0:
            if isinstance(target_pose[0], list):
                is_dual_arm_input = True
        
        try:
            import numpy as _np
            from robocore.transform import quaternion_to_matrix as _quat_to_mat, make_transform as _make_tf
            from robocore.planning.trajectory import linear_cartesian_trajectory as _linear_cartesian_trajectory
        except Exception as e:
            logger.error(f"未安装 RoboCore 或导入失败: {e}")
            return False
        
        # 处理单臂模式
        if arm in ("left_arm", "right_arm"):
            if is_dual_arm_input:
                logger.warning(f"单臂模式但输入了二维数组，使用第一个元素")
                target_pose = target_pose[0]
            
            if len(target_pose) != 7:
                logger.error(f"位姿必须是7个元素 [x, y, z, qx, qy, qz, qw]，但得到 {len(target_pose)} 个")
                return False
            
            current_pose = self.get_pose(arm=arm)
            if current_pose is None:
                logger.error("无法获取当前位姿")
                return False
            pose_start = current_pose['transform']
            
            position = _np.array(target_pose[:3])
            quaternion = _np.array(target_pose[3:])
            rotation = _quat_to_mat(quaternion)
            pose_end = _make_tf(rotation, position)
            
            q_init = self.get_joints(arm=arm)
            if not q_init or not isinstance(q_init, list) or len(q_init) != 7:
                logger.error("无法获取当前关节角度作为IK初值")
                return False
            
            try:
                _, _, q_traj = _linear_cartesian_trajectory(
                    self.robot_model,
                    pose_start,
                    pose_end,
                    duration,
                    num_points=num_points,
                    q_init=_np.array(q_init),
                    ik_backend='numpy',
                    ik_method=ik_method,
                    max_iters=200,
                    pos_tol=1e-3,
                    ori_tol=1e-3,
                )
            except Exception as e:
                logger.error(f"轨迹规划失败: {e}")
                return False
            
            delay = duration / num_points
            for q in q_traj.tolist():
                if not self.servo_driver.set_joint_angles(q, arm=arm, wait_for_completion=False):
                    return False
                time.sleep(delay)
            return True
        
        # 处理双臂模式
        elif arm == "both":
            # 确定左右臂的目标位姿
            if is_dual_arm_input:
                if len(target_pose) != 2 or len(target_pose[0]) != 7 or len(target_pose[1]) != 7:
                    logger.error("双臂模式需要 [[left_7_elements], [right_7_elements]] 格式")
                    return False
                target_pose_left = target_pose[0]
                target_pose_right = target_pose[1]
            elif target_pose_second_arm is not None:
                if len(target_pose) != 7 or len(target_pose_second_arm) != 7:
                    logger.error("双臂模式的位姿必须是7个元素 [x, y, z, qx, qy, qz, qw]")
                    return False
                target_pose_left = target_pose
                target_pose_right = target_pose_second_arm
            else:
                logger.error("双臂模式需要提供两个位姿：使用二维数组或提供 target_pose_second_arm 参数")
                return False
            
            # 获取当前位姿
            current_pose = self.get_pose(arm="both")
            if current_pose is None:
                logger.error("无法获取当前位姿（双臂）")
                return False
            
            pose_start_left = current_pose['transform'][0]
            pose_start_right = current_pose['transform'][1]
            
            # 构建目标位姿矩阵
            position_left = _np.array(target_pose_left[:3])
            quaternion_left = _np.array(target_pose_left[3:])
            rotation_left = _quat_to_mat(quaternion_left)
            pose_end_left = _make_tf(rotation_left, position_left)
            
            position_right = _np.array(target_pose_right[:3])
            quaternion_right = _np.array(target_pose_right[3:])
            rotation_right = _quat_to_mat(quaternion_right)
            pose_end_right = _make_tf(rotation_right, position_right)
            
            # 获取当前关节角度作为IK初值
            q_init = self.get_joints(arm="both")
            if not q_init or not isinstance(q_init, list) or len(q_init) != 2:
                logger.error("无法获取当前关节角度作为IK初值（双臂）")
                return False
            
            q_init_left = _np.array(q_init[0])
            q_init_right = _np.array(q_init[1])
            
            try:
                # 为左右臂分别生成轨迹
                _, _, q_traj_left = _linear_cartesian_trajectory(
                    self.robot_model,
                    pose_start_left,
                    pose_end_left,
                    duration,
                    num_points=num_points,
                    q_init=q_init_left,
                    ik_backend='numpy',
                    ik_method=ik_method,
                    max_iters=200,
                    pos_tol=1e-3,
                    ori_tol=1e-3,
                )
                _, _, q_traj_right = _linear_cartesian_trajectory(
                    self.robot_model,
                    pose_start_right,
                    pose_end_right,
                    duration,
                    num_points=num_points,
                    q_init=q_init_right,
                    ik_backend='numpy',
                    ik_method=ik_method,
                    max_iters=200,
                    pos_tol=1e-3,
                    ori_tol=1e-3,
                )
            except Exception as e:
                logger.error(f"轨迹规划失败: {e}")
                return False
            
            delay = duration / num_points
            for q_left, q_right in zip(q_traj_left.tolist(), q_traj_right.tolist()):
                # 同时发送左右臂指令
                success_left = self.servo_driver.set_joint_angles(q_left, arm="left_arm", wait_for_completion=False)
                success_right = self.servo_driver.set_joint_angles(q_right, arm="right_arm", wait_for_completion=False)
                if not (success_left and success_right):
                    return False
                time.sleep(delay)
            return True
        else:
            logger.error(f"非法的 arm 参数: {arm}")
            return False

    # ==================== 辅助方法 ====================

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

