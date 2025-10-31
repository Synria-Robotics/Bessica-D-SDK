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
import pkg_resources
# Import from robocore for kinematics and planning
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
from ..hardware import ServoDriver
from ..utils.control import move_joints_dual_arm, set_dual_gripper, control_move, move_joints,set_gripper_angle,open_gripper,close_gripper
# from ..execution import HardwareExecutor, JointPlanner
# from ..utils.logger import logger
logger = logging.getLogger("SynriaBessicaRobotAPI")
# from robocore.utils.control_utils import compute_steps_and_delay, validate_joint_list


class SynriaBessicaRobotAPI:
    """Bessica 机器人 API - 与 ServoDriver 协作，面向双臂/单臂控制。"""

    def __init__(
        self,
                 servo_driver: ServoDriver,
                 #robot_model: None,
                 #robot_model: RobotModel,
                 #firmware_version: None,
                 speed_deg_s: float = 20.0,
                 robot_version: str = "v1_0",
                 ):
        """Initialize robot API.

        :param servo_driver: Servo driver instance
        :param robot_model: Robot model from RoboCore
        :param firmware_version: Firmware version string
        :param speed_deg_s: Default speed in degrees per second
        """
        self.servo_driver = servo_driver
        # 在 API 内部创建 RobotModel 实例
        try:
                from robot_descriptions.urdf.Bessica_D_v1_0 import Bessica_D_Covered
                # 你当前的资源结构是 Bessica_D_v1_0/Bessica_D_Covered.urdf
                # 这类包通常导出一个 .urdf 路径属性，如：
                urdf_path = Bessica_D_Covered.urdf
                self.robot_model = RobotModel(str(urdf_path))
        except Exception as e:
                # 本地兜底（按你项目里已有的 Alicia fallback 模式）
                from pathlib import Path
                default_urdf = Path(__file__).parent.parent / "assets" / "robot" / "urdf" / f"Alicia-D_{robot_version}" / "alicia_duo_with_gripper.urdf"
                if default_urdf.exists():
                    self.robot_model = RobotModel(str(default_urdf), end_link='tool0')
                raise RuntimeError(f"无法创建 RobotModel，请检查 robot_descriptions 或本地 URDF。错误: {e}")
        #self.robot_model = robot_model
        #self.firmware_version = firmware_version
        self.firmware_new = False
        self.speed_deg_s = speed_deg_s
        # self.hardware_executor = HardwareExecutor(servo_driver)
        # self.joint_planner = JointPlanner()
        self.home_angles = [0.0] * 7
        self.default_arm = "both"

    # ==================== Connection Management ====================
    
    def connect(self) -> bool:
        """Connect to robot and detect firmware version."""
        result = self.servo_driver.connect()
        return result
    def disconnect(self):
        """Disconnect from robot."""
        self.servo_driver.stop_update_thread()
        self.servo_driver.disconnect()

    # ==================== 关节控制 ====================
    def set_home(self, arm: str = "both", speed_factor: float = 1.0) -> bool:
        """回到零位。单臂传 7 关节 0；双臂依次/插值到 0。"""
        if arm == "both":
            move_joints_dual_arm(self.servo_driver, self.home_angles, self.home_angles)
            open_gripper(self.servo_driver, 0.0, arm="both")


    def set_joint_target(
        self,
        target_joints: List[float],
        arm: Optional[str] = None,
        joint_format: str = "rad",
        wait: bool = True,
        speed_factor: float = 0.5,
    ) -> bool:
        """将单臂移动到目标关节角。"""
        arm = arm or self.default_arm
        if arm == "both":
            move_joints_dual_arm(self.servo_driver, target_joints, target_joints)
        elif arm == "left_arm":
            move_joints(self.servo_driver, target_joints, arm="right_arm")
        elif arm == "right_arm":
            move_joints(self.servo_driver, target_joints, arm="left_arm")
        else:
            logger.error(f"请输入指定要控制的机械臂，当前指定机械臂为{arm}")
            return False
        #control_move(self.servo_driver, target_joints, arm=arm)
        #return True

    def set_pose_target(self,
                         target_pose: List[float],
                         backend: str = 'numpy',
                         method: str = 'dls', 
                         display: bool = True, 
                         tolerance: float = 1e-4, 
                         max_iters: int = 100, 
                         multi_start: int = 0, 
                         use_random_init: bool = False, 
                         speed_factor: float = 1.0, 
                         arm: str = "both",
                         execute: bool = True) -> Dict:
        """基于逆解将末端移动到目标位姿。

        :param target_pose: 目标位姿 [x, y, z, qx, qy, qz, qw]
        :param backend: 'numpy' 或 'torch'
        :param method: 'dls'/'pinv'/'transpose'
        :param display: 是否打印求解细节
        :param tolerance: 位置与姿态容差
        :param max_iters: 最大迭代次数
        :param multi_start: 多起点尝试次数
        :param use_random_init: 是否使用随机初值
        :param speed_factor: 运动速度因子（用于插值）
        :param execute: 是否执行得到的关节解
        :return: 包含 success/q/iters/pos_err/ori_err/message/motion_executed 等字段
        """
        if not hasattr(self, 'robot_model') or self.robot_model is None:
            return {
                'success': False,
                'message': '未提供 robot_model，无法求解 IK',
                'q': None
            }

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
        if display:
            logger.info(f"初始关节角度 (rad): {[f'{q:+.4f}' for q in q_init]}")
            logger.info(f"初始关节角度 (deg): {[f'{np.rad2deg(q):+.2f}' for q in q_init]}")
            logger.info(f"正在求解IK (方法: {method},求解臂{arm} 最大迭代: {max_iters})...")
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

        if ik_result.get('success'):
            if display:
                logger.info("✓ IK 求解成功!")
                logger.info(f"  迭代次数: {ik_result.get('iters')}")
                logger.info(f"  位置误差: {ik_result.get('pos_err', 0.0):.6e} m")
                logger.info(f"  姿态误差: {ik_result.get('ori_err', 0.0):.6e} rad")
                logger.info(f"  关节角度 (rad): {[f'{q:+.4f}' for q in ik_result['q']]}")
                logger.info(f"  关节角度 (deg): {[f'{np.rad2deg(q):+.2f}' for q in ik_result['q']]}")

            if execute:
                ok = move_joints(self.servo_driver, ik_result['q'], arm=arm, interpolate=True)
                ik_result['motion_executed'] = bool(ok)
            else:
                ik_result['motion_executed'] = False
                if display:
                    logger.info("  (未执行运动，execute=False)")
            return ik_result
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
        wait_for_completion: bool = True,
        timeout: float = 1.0,
        tolerance: float = 0.1,
    ) -> bool:
        """
        控制夹爪：
        - command: 'open' or 'close'
        - value: 角度值（单位：度，0~100），内部转换为弧度传给 ServoDriver
        """
        if (command is None) == (value is None):
            logger.error("必须二选一提供 command 或 value")
            return False
        if command is not None:
            if command == 'open':
                value = 0.0
            elif command == 'close':
                value = 100.0
            else:
                logger.error("command 仅支持 'open'/'close'")
                return False
        # value 单位: 度 -> 弧度
        #angle_rad = float(value) * np.pi / 180.0
        if arm == "left_arm" or arm == "right_arm":
            set_gripper_angle(self.servo_driver, value, arm=arm, wait=wait_for_completion)
        elif arm == "both":
            set_dual_gripper(self.servo_driver, value, value, wait=wait_for_completion) #wait=wait_for_completion)
        else:
            logger.error(f"请输入指定要控制的机械臂，当前指定机械臂为{arm}")
            return False
        return True

    # ==================== 位姿与状态 ====================
    def get_joints(self, arm: Optional[str] = None) -> Optional[Union[List[float], List[List[float]]]]:
        self.default_arm = "both"
        arm = arm or self.default_arm
        joint_angles = self.servo_driver.read_joint_angles(arm)
        logger.info(f"{arm}'s joint_angles: {joint_angles}")
        
        return joint_angles

    def get_gripper(self, arm: Optional[str] = None) -> Optional[Union[float, Tuple[float, float]]]:
        arm = arm or self.default_arm
        try:
            logger.info(f"{arm}'s gripper: {self.servo_driver.read_gripper_data(arm if arm in ['left_arm', 'right_arm'] else 'both')}")
            return self.servo_driver.read_gripper_data(arm if arm in ['left_arm', 'right_arm'] else 'both')
        except Exception:
            return None

    def get_pose(self, arm: Optional[str] = None) -> Optional[Dict]:
        if self.robot_model is None:
            logger.error("未安装 RoboCore 或未提供 robot_model，无法计算位姿")
            return None
        arm = arm or self.default_arm
        joints = self.get_joints(arm=arm)
        if not joints or not isinstance(joints, list):
            logger.error("无法获取关节角度")
            return None
        T_fk = forward_kinematics(self.robot_model, joints, backend='numpy', return_end=True)
        position = T_fk[:3, 3]
        rotation = T_fk[:3, :3]
        euler = matrix_to_euler(rotation, seq='xyz')
        quat = matrix_to_quaternion(rotation)
        return {
            'transform': T_fk,
            'position': position,
            'rotation': rotation,
            'euler_xyz': euler,
            'quaternion_xyzw': quat,
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
    def torque_control(self, command: str, arm: str = 'both') -> bool:
        if command == 'on':
            return self.servo_driver.enable_torque(arm)
        elif command == 'off':
            return self.servo_driver.disable_torque(arm)
        else:
            logger.error("command 必须为 'on' 或 'off'")
            return False

    def set_zero(self, arm: str = 'both') -> bool:
        logger.info(f"即将将{arm}关闭扭矩，请确定环境正常,输入enter继续...")
        input()
        self.torque_control(command="off", arm=arm)
        logger.info(f"即将将{arm}设置为零点，请确定环境正常，输入enter继续...")
        input()
        result = self.servo_driver.set_zero_position(arm)
        self.torque_control(command="on", arm=arm)
        logger.info(f"{arm}归零成功: {result}")


    # ==================== 轨迹接口（可用 RoboCore 时增强） ====================
    # def move_joint_trajectory(
    #     self,
    #     q_end: List[float],
    #     arm: Optional[str] = None,
    #     duration: float = 2.0,
    #     method: str = 'linear',
    #     num_points: int = 100,
    #     joint_format: str = 'rad',
    #     visualize: bool = False,
    # ) -> bool:
    #     arm = arm or self.default_arm
    #     _validate_joint_list(q_end, expected_len=7)
    #     if joint_format == 'deg':
    #         q_end = [a * np.pi / 180.0 for a in q_end]
    #     q_start = self.get_joints(arm=arm)
    #     if not q_start:
    #         logger.error("无法获取当前关节角度")
    #         return False
    #     if HAVE_ROBOCORE and method in {'linear', 'cubic', 'quintic'}:
    #         if method == 'linear':
    #             _, q_traj, _, _ = linear_joint_trajectory(np.array(q_start), np.array(q_end), duration, num_points)
    #         elif method == 'cubic':
    #             _, q_traj, _, _ = cubic_polynomial_trajectory(np.array(q_start), np.array(q_end), duration, num_points)
    #         else:
    #             _, q_traj, _, _ = quintic_polynomial_trajectory(np.array(q_start), np.array(q_end), duration, num_points)
    #         delay = duration / num_points
    #         for q in q_traj.tolist():
    #             if not self.servo_driver.set_joint_angles(q, arm=arm, wait_for_completion=False):
    #                 return False
    #             time.sleep(delay)
    #         return True
    #     # 退化：简单线性插值
    #     steps, delay = _compute_steps_and_delay(speed_factor=duration / 2.0, T_default=duration, n_steps_ref=num_points)
    #     for s in range(1, steps + 1):
    #         r = s / steps
    #         q = [a + (b - a) * r for a, b in zip(q_start, q_end)]
    #         if not self.servo_driver.set_joint_angles(q, arm=arm, wait_for_completion=False):
    #             return False
    #         time.sleep(delay)
    #     return True

    # def move_cartesian_linear(
    #     self,
    #     target_pose: List[float],
    #     arm: Optional[str] = None,
    #     duration: float = 2.0,
    #     num_points: int = 50,
    #     ik_method: str = 'dls',
    #     visualize: bool = False,
    # ) -> bool:
    #     if not HAVE_ROBOCORE or self.robot_model is None:
    #         logger.error("未安装 RoboCore 或未提供 robot_model，无法执行笛卡尔轨迹")
    #         return False
    #     arm = arm or self.default_arm
    #     current_pose = self.get_pose(arm=arm)
    #     if current_pose is None:
    #         logger.error("无法获取当前位姿")
    #         return False
    #     pose_start = current_pose['transform']
    #     position = np.array(target_pose[:3])
    #     quaternion = np.array(target_pose[3:])
    #     rotation = quaternion_to_matrix(quaternion)
    #     pose_end = make_transform(rotation, position)
    #     q_init = self.get_joints(arm=arm)
    #     try:
    #         _, _, q_traj = linear_cartesian_trajectory(
    #             self.robot_model,
    #             pose_start,
    #             pose_end,
    #             duration,
    #             num_points=num_points,
    #             q_init=np.array(q_init),
    #             ik_backend='numpy',
    #             ik_method=ik_method,
    #             max_iters=100,
    #             pos_tol=1e-3,
    #             ori_tol=1e-3,
    #         )
    #     except Exception as e:
    #         logger.error(f"轨迹规划失败: {e}")
    #         return False
    #     delay = duration / num_points
    #     for q in q_traj.tolist():
    #         if not self.servo_driver.set_joint_angles(q, arm=arm, wait_for_completion=False):
    #             return False
    #         time.sleep(delay)
    #     return True

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