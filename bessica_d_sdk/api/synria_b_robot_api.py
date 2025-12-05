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
from robocore.modeling import RobotModel
from robocore.transform import make_transform, quaternion_to_matrix
from robocore.kinematics import forward_kinematics
from robocore.transform import matrix_to_euler, matrix_to_quaternion
from robocore.planning.trajectory import (
    cubic_polynomial_trajectory,
    quintic_polynomial_trajectory,
    linear_joint_trajectory,
    linear_cartesian_trajectory,
    circular_cartesian_trajectory,
    cartesian_waypoint_trajectory
)
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
    ):
        """Initialize robot API.

        :param servo_driver: Servo driver instance
        :param speed_deg_s: Default speed in degrees per second
        :param robot_version: Robot version string, e.g., "v1_0"
        """
        self.servo_driver = servo_driver
        # 在 API 内部创建 RobotModel 实例
        try:
            from synriard import get_model_path
            urdf_path = get_model_path("Bessica_D", version=robot_version, variant="covered")
            self.robot_model = RobotModel(str(urdf_path))
        except Exception as e:
            # 本地兜底（按你项目里已有的 Alicia fallback 模式）
            from pathlib import Path
            # 自行根据本地路径来更改并找到bessicia的urdf文件
            default_urdf = Path(__file__).parent.parent / "assets" / "robot" / "urdf" / f"Alicia-D_{robot_version}" / "alicia_duo_with_gripper.urdf"
            if default_urdf.exists():
                self.robot_model = RobotModel(str(default_urdf), end_link='tool0')
            raise RuntimeError(f"无法创建 RobotModel，请检查 synriard 或本地 URDF。错误: {e}")
        # self.robot_model = robot_model
        # self.firmware_version = firmware_version
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
    def set_home(self, arm: str = "") -> bool:
        """Move robot to home position.

        :param arm: Arm to control, "left_arm", "right_arm", or "both" (default: self.default_arm)
        :return: True if successful
        """
        arm = arm or self.default_arm
        logger.info(f"set_home at head: arm={arm}")
        home_angles = [0.0] * 7
        if arm == "both":
            success = self.servo_driver.move_dual_joints_deg(self.home_angles, self.home_angles,wait_for_completion=False)
            success &= self.set_gripper_target(command="open", arm="both")
            # success = self.set_joint_target(target_joints=[home_angles,home_angles], arm="both", tolerance_deg=1.0)
            # success &= self.set_gripper_target(command="open", arm="both")
            return success
        elif arm in ("left_arm", "right_arm"):
            print(f"set_home: arm={arm}")
            ok = self.set_joint_target(target_joints=home_angles, arm=arm, tolerance_deg=1.0)
            ok &= self.set_gripper_target(command="open", arm=arm)
            return ok
        else:
            logger.error(f"set_home: 非法 arm={arm}")
            return False

    def set_joint_target(
        self,
        target_joints: Union[List[float], List[List[float]]],
        arm: Optional[str] = None,
        joint_format: str = "deg",
        wait: bool = False,
        tolerance_deg: float = 3.0,
    ) -> bool:
        """Move robot to target joint angles.

        :param target_joints: Target joint angles. For single arm: List[float] (7 angles). For dual arm: List[List[float]] (2x7 angles)
        :param arm: Arm to control, "left_arm", "right_arm", or "both" (default: self.default_arm)
        :param joint_format: Unit format, "deg" or "rad" (currently only "deg" supported)
        :param wait: Wait for motion completion if True
        :param tolerance_deg: Maximum allowed error per joint in degrees when waiting for completion (default: 3.0)
        :return: True if command sent successfully
        """
        arm = arm or self.default_arm
        if joint_format.lower() not in ("deg", "degree", "degrees"):
            logger.error("set_joint_target 目前仅支持 joint_format='deg'")
            return False
        if arm == "both":
            # shape the target_joints to a list of two lists
            target_joints_left = target_joints[0]
            target_joints_right = target_joints[1]
            return self.servo_driver.move_dual_joints_deg(
                left_angles_deg=target_joints_left, 
                right_angles_deg=target_joints_right,
                wait_for_completion=wait,
                tolerance_deg=tolerance_deg
            )
        elif arm in ("left_arm"):
            # logger.info(f"set_joint_target: arm={arm}, target_joints={target_joints}")
            return self.servo_driver.move_joints_deg(arm="right_arm", angles_deg=target_joints, wait_for_completion=wait, tolerance_deg=tolerance_deg)
        elif arm in ("right_arm"):
            # print(f"set_joint_target: arm={arm}, target_joints={target_joints}")
            return self.servo_driver.move_joints_deg(arm="left_arm", angles_deg=target_joints, wait_for_completion=wait, tolerance_deg=tolerance_deg)
        else:
            logger.error(f"set_joint_target: 非法 arm={arm}")
            return False

    def set_pose_target(self,
                        target_pose: List[float],
                        target_pose_second_arm: Optional[List[float]] = None,
                        backend: str = 'numpy',
                        method: str = 'dls',
                        display: bool = True,
                        tolerance: float = 1e-4,
                        max_iters: int = 100,
                        multi_start: int = 0,
                        use_random_init: bool = False,
                        arm: str = "both",
                        execute: bool = True) -> Dict:
        """Move end-effector to target pose using inverse kinematics.

        :param target_pose: Target pose as [x, y, z, qx, qy, qz, qw]
        :param target_pose_second_arm: Target pose for second arm (required when arm="both")
        :param backend: Computation backend, 'numpy' or 'torch'
        :param method: IK solver method, 'dls', 'pinv', or 'transpose'
        :param display: Display solution details
        :param tolerance: Position and orientation tolerance
        :param max_iters: Maximum number of iterations
        :param multi_start: Number of multi-start attempts, 0 to disable
        :param use_random_init: Use random initial guess instead of current pose
        :param arm: Arm to control, "left_arm", "right_arm", or "both"
        :param execute: Execute motion if True
        :return: Dictionary with success, q, iters, pos_err, ori_err, message, motion_executed
        """
        if not hasattr(self, 'robot_model') or self.robot_model is None:
            return {
                'success': False,
                'message': '未提供 robot_model，无法求解 IK',
                'q': None
            }
        if arm == "left_arm" or arm == "right_arm":
            # 构建位姿矩阵
            position = np.array(target_pose[:3])
            quaternion = np.array(target_pose[3:])
            rotation_matrix = quaternion_to_matrix(quaternion)
            pose_matrix = make_transform(rotation_matrix, position)
            if use_random_init:
                q_init = self._generate_random_q(scale=0.5)
                if display:
                    logger.info("使用随机初始值")
            else:
                q_init = self.get_joints(arm=arm)
                if q_init is None:
                    return {
                        'success': False,
                        'message': '无法获取当前关节角度',
                        'q': None
                    }

            ik_result = inverse_kinematics(
                self.robot_model,
                pose_matrix,
                q_init,
                backend=backend,
                method=method,
                max_iters=max_iters,
                pos_tol=tolerance,
                ori_tol=tolerance,
                multi_start=multi_start,
                multi_noise=0.3,
                use_analytic_jacobian=True
            )
            if ik_result['success']:
                if display:
                    logger.info("✓ IK 求解成功!")
                    logger.info(f"  迭代次数: {ik_result['iters']}")
                    logger.info(f"  位置误差: {ik_result['pos_err']:.6e} m")
                    logger.info(f"  姿态误差: {ik_result['ori_err']:.6e} rad")
                    logger.info(f"  关节角度 (rad): {[f'{q:+.4f}' for q in ik_result['q']]}")
                    logger.info(f"  关节角度 (deg): {[f'{np.rad2deg(q):+.2f}' for q in ik_result['q']]}")
                # if execute:
                    q = np.rad2deg(ik_result['q'])
                    print(f"q: {q}")
                    success = self.set_joint_target(q.tolist(), arm=arm)
                    ik_result['motion_executed'] = bool(success)
                # else:
                #     ik_result['motion_executed'] = False
                #     if display:
                #         logger.info("  (未执行运动，execute=False)")
                return ik_result
            else:
                if display:
                    logger.error(f"✗ IK 求解失败: {ik_result.get('message', '未知错误')}")
                    logger.error(f"  迭代次数: {ik_result.get('iters', 'N/A')}")
                    logger.error(f"  位置误差: {ik_result.get('pos_err', float('inf')):.6e} m")
                    logger.error(f"  姿态误差: {ik_result.get('ori_err', float('inf')):.6e} rad")
                return ik_result
        if arm == "both":
            position_l = np.array(target_pose[:3])
            quaternion_l = np.array(target_pose[3:])
            rotation_matrix_l = quaternion_to_matrix(quaternion_l)
            pose_matrix_l = make_transform(rotation_matrix_l, position_l)
            position_r = np.array(target_pose_second_arm[:3])
            quaternion_r = np.array(target_pose_second_arm[3:])
            rotation_matrix_r = quaternion_to_matrix(quaternion_r)
            pose_matrix_r = make_transform(rotation_matrix_r, position_r)
            # Get initial guess
            if use_random_init:
                # Generate random initial guess within joint limits
                q_init_l = self._generate_random_q(scale=0.5)
                q_init_r = self._generate_random_q(scale=0.5)
                if display:
                    logger.info("使用随机初始值")
            else:
                q_init_l = self.get_joints(arm="left_arm")
                q_init_r = self.get_joints(arm="right_arm")
                if q_init_l is None or q_init_r is None:
                    return {
                        'success': False,
                        'message': '无法获取当前关节角度',
                        'ql': None,
                        'qr': None
                    }

            if display:
                logger.info(f"左臂初始关节角度 (rad): {[f'{q:+.4f}' for q in q_init_l]}")
                logger.info(f"左臂初始关节角度 (deg): {[f'{np.rad2deg(q):+.2f}' for q in q_init_l]}")
                logger.info(f"右臂初始关节角度 (rad): {[f'{q:+.4f}' for q in q_init_r]}")
                logger.info(f"右臂初始关节角度 (deg): {[f'{np.rad2deg(q):+.2f}' for q in q_init_r]}")
                logger.info(f"正在求解IK (方法: {method}, 最大迭代: {max_iters})...")
            ik_result_l = inverse_kinematics(
                self.robot_model,
                pose_matrix_l,
                q_init_l,
                backend=backend,
                method=method,
                max_iters=max_iters,
                pos_tol=tolerance,
                ori_tol=tolerance,
                multi_start=multi_start,
                multi_noise=0.3,
                use_analytic_jacobian=True
            )
            ik_result_r = inverse_kinematics(
                self.robot_model,
                pose_matrix_r,
                q_init_r,
                backend=backend,
                method=method,
                max_iters=max_iters,
                pos_tol=tolerance,
                ori_tol=tolerance,
                multi_start=multi_start,
                multi_noise=0.3,
                use_analytic_jacobian=True
            )
            if ik_result_l.get('success') and ik_result_r.get('success'):
                if display:
                    logger.info("✓ IK 求解成功!")
                    logger.info(f"  左臂迭代次数: {ik_result_l.get('iters')}")
                    logger.info(f"  左臂位置误差: {ik_result_l.get('pos_err', 0.0):.6e} m")
                    logger.info(f"  左臂姿态误差: {ik_result_l.get('ori_err', 0.0):.6e} rad")
                    logger.info(f"  左关节角度 (rad): {[f'{q:+.4f}' for q in ik_result_l['q']]}")
                    logger.info(f"  左关节角度 (deg): {[f'{np.rad2deg(q):+.2f}' for q in ik_result_l['q']]}")
                    logger.info(f"  右臂迭代次数: {ik_result_r.get('iters')}")
                    logger.info(f"  右臂位置误差: {ik_result_r.get('pos_err', 0.0):.6e} m")
                    logger.info(f"  右臂姿态误差: {ik_result_r.get('ori_err', 0.0):.6e} rad")
                    logger.info(f"  右关节角度 (rad): {[f'{q:+.4f}' for q in ik_result_r['q']]}")
                    logger.info(f"  右关节角度 (deg): {[f'{np.rad2deg(q):+.2f}' for q in ik_result_r['q']]}")
                # if execute:
                    q_l = np.rad2deg(ik_result_l['q'])
                    q_r = np.rad2deg(ik_result_r['q'])
                    q = [q_l.tolist(), q_r.tolist()]
                    ok = self.set_joint_target(target_joints=q, arm="both")
                    ik_result_l['motion_executed'] = bool(ok)
                    ik_result_r['motion_executed'] = bool(ok)
                # else:
                #     ik_result_l['motion_executed'] = False
                #     ik_result_r['motion_executed'] = False
                #     if display:
                #         logger.info("  (未执行运动，execute=False)")
                return ik_result_l, ik_result_r
            else:
                if display:
                    logger.error(f"✗ IK 求解失败: {ik_result.get('message', '未知错误')}")
                    logger.error(f"  迭代次数: {ik_result.get('iters', 'N/A')}")
                    logger.error(f"  位置误差: {ik_result.get('pos_err', float('inf')):.6e} m")
                    logger.error(f"  姿态误差: {ik_result.get('ori_err', float('inf')):.6e} rad")
                return ik_result

    # 插值接口（显式）
    # def set_joint_target_interpolation(
    #     self,
    #     target_joints: List[float],
    #     arm: Optional[str] = None,
    #     joint_format: str = "rad",
    #     speed_factor: float = 1.0,
    #     T_default: float = 4.0,
    #     n_steps_ref: int = 200,
    # ) -> bool:
    #     arm = arm or self.default_arm
    #     _validate_joint_list(target_joints, expected_len=7)
    #     if joint_format.lower() == 'deg':
    #         target_joints = [a * np.pi / 180.0 for a in target_joints]
    #     target_joints, _ = _check_and_clip_joint_limits(target_joints, self.robot_model)
    #     steps, delay = _compute_steps_and_delay(speed_factor=speed_factor, T_default=T_default, n_steps_ref=n_steps_ref)
    #     cur = self.get_joints(arm=arm)
    #     if not isinstance(cur, list) or len(cur) != 7:
    #         return self.servo_driver.set_joint_angles(target_joints, arm=arm, wait_for_completion=True)
    #     for s in range(1, steps + 1):
    #         r = s / steps
    #         interp = [c + (t - c) * r for c, t in zip(cur, target_joints)]
    #         if not self.servo_driver.set_joint_angles(interp, arm=arm, wait_for_completion=False):
    #             return False
    #         time.sleep(delay)
    #     return True

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
        :return: Joint angles in degrees. For single arm: List[float] (7 angles). For dual arm: List[List[float]] (2x7 angles). None if unavailable
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
            joints = [joints_l, joints_r]
            T_fk_l = forward_kinematics(self.robot_model, joints_l, backend='numpy', return_end=True)
            position_l = T_fk_l[:3, 3]
            rotation_l = T_fk_l[:3, :3]
            euler_l = matrix_to_euler(rotation_l, seq='xyz')
            quat_l = matrix_to_quaternion(rotation_l)
            output_to_ik_l = [position_l[0], position_l[1], position_l[2], quat_l[0], quat_l[1], quat_l[2], quat_l[3]]
            T_fk_r = forward_kinematics(self.robot_model, joints_r, backend='numpy', return_end=True)
            position_r = T_fk_r[:3, 3]
            rotation_r = T_fk_r[:3, :3]
            euler_r = matrix_to_euler(rotation_r, seq='xyz')
            quat_r = matrix_to_quaternion(rotation_r)
            output_to_ik_r = [position_r[0], position_r[1], position_r[2], quat_r[0], quat_r[1], quat_r[2], quat_r[3]]
            return {
                'transform': [T_fk_l, T_fk_r],
                'position': [position_l, position_r],
                'rotation': [rotation_l, rotation_r],
                'euler_xyz': [euler_l, euler_r],
                'quaternion_xyzw': [quat_l, quat_r],
                'output_to_ik': [output_to_ik_l, output_to_ik_r],
            }
        elif arm == "left_arm" or arm == "right_arm":
            joints = self.get_joints(arm=arm)
            if not joints or not isinstance(joints, list):
                logger.error("无法获取关节角度")
                return None
            T_fk = forward_kinematics(self.robot_model, joints, backend='numpy', return_end=True)
            position = T_fk[:3, 3]
            rotation = T_fk[:3, :3]
            euler = matrix_to_euler(rotation, seq='xyz')
            quat = matrix_to_quaternion(rotation)
            output_to_ik = [position[0], position[1], position[2], quat[0], quat[1], quat[2], quat[3]]
            return {
                'transform': T_fk,
                'position': position,
                'rotation': rotation,
                'euler_xyz': euler,
                'quaternion_xyzw': quat,
                'output_to_ik': output_to_ik,
            }

    # def get_firmware_version(self, timeout: float = 5.0, send_interval: float = 0.2) -> Optional[str]:
    #     # 优先读缓存
    #     cache_path = os.path.join(os.path.dirname(__file__), "firmware_version.json")
    #     if os.path.exists(cache_path):
    #         try:
    #             with open(cache_path, "r") as f:
    #                 ver = json.load(f).get("firmware_version")
    #                 if ver:
    #                     self.firmware_version = ver
    #                     return ver
    #         except Exception:
    #             pass
    #     # 主动查询（依赖 ServoDriver 的解析）
    #     start = time.time()
    #     cmd = [0xAA, 0x0A, 0x01, 0x00, 0x00, 0xFF]
    #     while time.time() - start < timeout:
    #         try:
    #             self.servo_driver.serial_comm.send_data(cmd)
    #             time.sleep(0.1)
    #             ver = self.servo_driver.get_firmware_version()
    #             if ver and ver != "未知版本":
    #                 try:
    #                     with open(cache_path, "w") as f:
    #                         json.dump({"firmware_version": ver}, f)
    #                 except Exception:
    #                     pass
    #                 return ver
    #         except Exception as e:
    #             logger.error(f"读取固件版本异常: {e}")
    #         time.sleep(send_interval)
    #     return None

    # ==================== 系统控制 ====================
    def set_speed(self, speed_deg_s: float) -> bool:
        """Set motion speed.

        :param speed_deg_s: Speed in degrees per second
        :return: True if successful
        """
        return self.servo_driver.set_speed_deg_s(speed_deg_s)

    # def set_speed_factor(self, speed_factor: float) -> bool:
    #     """按系数设置速度：factor=1.0 → 原始值约 1000。"""
    #     return self.servo_driver.set_speed_factor(speed_factor)
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

    # # ==================== 打印/辅助 ====================
    # def print_state(self, arm: str = 'both', output_format: str = 'deg'):
    #     def _print_once():
    #         joints = self.get_joints(arm=arm)
    #         grip = self.get_gripper(arm=arm)
    #         if joints is None:
    #             logger.warning("无法获取关节状态")
    #             return
    #         if arm == 'both':
    #             if output_format == 'deg':
    #                 left = [a * 180.0 / np.pi for a in joints[0]]
    #                 right = [a * 180.0 / np.pi for a in joints[1]]
    #             else:
    #                 left, right = joints
    #             logger.info(f"左臂: {[round(a, 2) for a in left]} | 右臂: {[round(a, 2) for a in right]} | 夹爪: {grip}")
    #         else:
    #             arr = [a * 180.0 / np.pi for a in joints] if output_format == 'deg' else joints
    #             logger.info(f"{arm}: {[round(a, 2) for a in arr]} | 夹爪: {grip}")
    #     _print_once()
