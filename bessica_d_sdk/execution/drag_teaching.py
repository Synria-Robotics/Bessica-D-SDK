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

import os
import json
import time
import threading
from typing import List, Dict, Any, Optional, Union
from datetime import datetime
import numpy as np

from bessica_d_sdk.utils.trajectory_utils import record_waypoints_manual


class SimpleDragTeaching:
    """拖动示教类 - 支持单臂和双臂 (Bessica)"""

    def __init__(self, args, controller, arm: str = "both"):
        """
        :param args: Command line arguments
        :param controller: Robot controller instance
        :param arm: Arm mode, "left", "right", or "both" (default: "both")
        """
        self.args = args
        self.controller = controller
        self.arm = arm

    def setup(self):
        """初始化设置"""
        if self.args.save_motion:
            print(f"动作名: {self.args.save_motion}")
        if self.args.mode == 'auto':
            sample_hz = getattr(self.args, "sample_hz", 50.0)
            print(f"采样频率: {sample_hz} Hz")
        # 显示回放/运动速度
        if hasattr(self.args, "speed_deg_s"):
            print(f"关节速度: {self.args.speed_deg_s} deg/s")
        print(f"控制模式: {self.arm} arm(s)")
        print("=" * 30)

    def manual_mode(self) -> List[Dict[str, Any]]:
        """手动模式 - 记录关键点"""
        return record_waypoints_manual(self.controller, arm=self.arm)

    def auto_mode(self) -> List[Dict[str, Any]]:
        """自动模式 - 连续记录 (支持单臂和双臂)"""
        print("\n=== 自动模式 ===")
        print("关闭扭矩后拖动机械臂，系统自动记录轨迹")
        if self.arm == "both":
            print("记录双臂轨迹")
        else:
            print(f"记录{self.arm}臂轨迹")

        input("按回车开始...")
        self.controller.torque_control(command='off', arm=self.arm)
        print("[安全] 扭矩已关闭，可以拖动机械臂")

        # 记录变量
        trajectory = []
        recording = threading.Event()

        def record_loop():
            """后台记录线程"""
            sample_hz = getattr(self.args, "sample_hz", 50.0)
            dt = 1.0 / sample_hz
            start_time = time.time()

            while recording.is_set():
                try:
                    current_time = time.time() - start_time
                    # Get joint and gripper together in a single call (more efficient)
                    state = self.controller.get_robot_state("joint_gripper")
                    
                    if self.arm == "both":
                        # Dual-arm mode
                        if state is not None and isinstance(state, dict):
                            left_js = state.get('left')
                            right_js = state.get('right')
                            if left_js is not None and right_js is not None:
                                point = {
                                    "t": current_time,
                                    "q_left": left_js.angles,
                                    "q_right": right_js.angles,
                                    "grip_left": left_js.gripper,
                                    "grip_right": right_js.gripper
                                }
                                trajectory.append(point)
                    else:
                        # Single-arm mode
                        if state is not None and isinstance(state, dict):
                            js = state.get(self.arm)
                            if js is not None:
                                point = {
                                    "t": current_time,
                                    "q": js.angles,
                                    "grip": js.gripper
                                }
                                trajectory.append(point)

                except Exception as e:
                    print(f"[警告] 记录失败: {e}")

                time.sleep(dt)

        try:
            input("开始拖动，按回车开始记录...")
            recording.set()
            thread = threading.Thread(target=record_loop, daemon=True)
            thread.start()
            sample_hz = getattr(self.args, "sample_hz", 50.0)
            print(f"[记录中] 频率 {sample_hz} Hz...")

            input("按回车停止记录...")
            recording.clear()
            thread.join(timeout=1.0)

        finally:
            self.controller.torque_control(command='on', arm=self.arm)
            print("[安全] 扭矩已重新开启")

        print(f"[完成] 记录了 {len(trajectory)} 个点")
        return trajectory

    def replay_only_mode(self) -> Optional[List[Dict[str, Any]]]:
        """仅回放模式 - 加载已有数据"""
        print("\n=== 仅回放模式 ===")

        # 检查动作名是否提供
        if not self.args.save_motion:
            print("[错误] 回放模式需要指定动作名")
            print("请使用 --save-motion <motion_name> 参数")
            print("或使用 --list-motions 查看可用动作")
            return None

        # 加载数据
        save_dir = os.path.join("example_motions", self.args.save_motion)
        traj_path = os.path.join(save_dir, "joint_traj.json")
        meta_path = os.path.join(save_dir, "meta.json")

        # 检查目录和文件是否存在
        if not os.path.exists(save_dir):
            print(f"[错误] 未找到动作目录: {save_dir}")
            print("\n可用的动作:")
            available = list_available_motions()
            if available:
                for motion in available[:5]:  # 只显示前5个
                    print(f"  - {motion}")
                if len(available) > 5:
                    print(f"  ... 还有 {len(available)-5} 个动作")
                print(f"\n使用 --list-motions 查看完整列表")
            else:
                print("  未找到任何已录制的动作")
                print("  请先使用 auto 或 manual 模式录制动作")
            return None

        if not os.path.exists(traj_path):
            print(f"[错误] 未找到轨迹文件: {traj_path}")
            print("该动作可能损坏或不完整")
            return None

        if not os.path.exists(meta_path):
            print(f"[警告] 未找到元数据文件: {meta_path}")
            print("将使用默认设置继续加载")

        # 读取数据和元信息
        try:
            with open(traj_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            print(f"[错误] 读取轨迹文件失败: {e}")
            return None

        # 读取元信息
        meta = {}
        if os.path.exists(meta_path):
            try:
                with open(meta_path, 'r', encoding='utf-8') as f:
                    meta = json.load(f)
            except Exception as e:
                print(f"[警告] 读取元数据失败: {e}")

        # 更新模式为原始记录模式
        original_mode = meta.get('mode', 'auto')
        print(f"[加载] 轨迹: {len(data)}个点")
        print(f"[加载] 原始模式: {original_mode}")
        print(f"[加载] 创建时间: {meta.get('created_at', 'Unknown')}")
        print(f"[加载] 采样频率: {meta.get('sample_hz', 'Unknown')} Hz")
        print(f"[加载] 控制模式: {meta.get('arm', 'both')}")

        # 临时保存原始参数并设置为原始模式
        self._original_mode = self.args.mode
        self._original_arm = self.arm
        self.args.mode = original_mode
        self.arm = meta.get('arm', 'both')

        return data

    def save_data(self, data: List[Dict[str, Any]]) -> Optional[str]:
        """保存数据"""
        if not data:
            print("[保存] 没有数据")
            return None

        # 创建保存目录
        save_dir = os.path.join("example_motions", self.args.save_motion)
        os.makedirs(save_dir, exist_ok=True)

        # 保存关节轨迹
        traj_path = os.path.join(save_dir, "joint_traj.json")
        with open(traj_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

        # 保存元信息
        sample_hz = getattr(self.args, "sample_hz", 50.0)
        meta = {
            "motion": self.args.save_motion,
            "mode": self.args.mode,  # 保存记录时的模式
            "arm": self.arm,  # 保存控制模式
            "created_at": datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            "sample_hz": sample_hz,
            "count": len(data),
            "description": "拖动示教轨迹"
        }

        meta_path = os.path.join(save_dir, "meta.json")
        with open(meta_path, 'w', encoding='utf-8') as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)

        print(f"[保存] 轨迹: {traj_path}")
        print(f"[保存] 元数据: {meta_path}")

        return save_dir

    def replay(self, data: List[Dict[str, Any]]):
        """根据模式回放轨迹 (支持单臂和双臂)"""
        if not data:
            return

        # 获取目标速度和加速度参数
        speed_deg_s = getattr(self.args, "speed_deg_s", 10)  # 目标速度，度/秒
        acceleration_deg_s2 = 22.0  # 加速度，度/秒²

        def _sleep_based_on_velocity(point: Dict[str, Any], prev_joints: Optional[Union[np.ndarray, Dict[str, np.ndarray]]]):
            """
            Calculate wait time based on joint angle difference, target velocity and acceleration.
            
            :param point: Current waypoint with joint angles
            :param prev_joints: Previous waypoint joint angles, None for first point
            :return: Wait time in seconds
            """
            if prev_joints is None:
                return 0.0

            # Handle dual-arm vs single-arm
            if self.arm == "both":
                if "q_left" in point and "q_right" in point:
                    current_left = np.array(point["q_left"])
                    current_right = np.array(point["q_right"])
                    if isinstance(prev_joints, dict):
                        prev_left = np.array(prev_joints.get("q_left", []))
                        prev_right = np.array(prev_joints.get("q_right", []))
                    else:
                        prev_left = np.array(prev_joints[0]) if len(prev_joints) > 0 else np.array([])
                        prev_right = np.array(prev_joints[1]) if len(prev_joints) > 1 else np.array([])
                    
                    if len(current_left) == 7 and len(prev_left) == 7 and len(current_right) == 7 and len(prev_right) == 7:
                        angle_diff_left = np.abs(np.degrees(current_left - prev_left))
                        angle_diff_right = np.abs(np.degrees(current_right - prev_right))
                        angle_diff_deg = np.concatenate([angle_diff_left, angle_diff_right])
                    else:
                        return 0.0
                else:
                    return 0.0
            else:
                # Single-arm mode
                if "q" in point:
                    current_joints = np.array(point["q"])
                    if isinstance(prev_joints, np.ndarray):
                        prev_joints_array = prev_joints
                    elif isinstance(prev_joints, list):
                        prev_joints_array = np.array(prev_joints)
                    else:
                        return 0.0
                    
                    if len(current_joints) == 7 and len(prev_joints_array) == 7:
                        angle_diff_deg = np.abs(np.degrees(current_joints - prev_joints_array))
                    else:
                        return 0.0
                else:
                    return 0.0

            # Use a robust diff to avoid being dominated by a single joint
            # 75th percentile lets most joints move smoothly while ignoring extreme outliers
            effective_angle_diff = float(np.percentile(angle_diff_deg, 75))

            if effective_angle_diff < 1e-6:  # Very small movement, no wait needed
                return 0.0

            # Calculate motion time based on acceleration and velocity
            # If distance is large enough to reach target velocity
            min_distance_to_reach_velocity = (speed_deg_s ** 2) / acceleration_deg_s2

            if effective_angle_diff >= min_distance_to_reach_velocity:
                # Trapezoidal profile: accelerate, constant velocity, decelerate
                motion_time = speed_deg_s / acceleration_deg_s2 + effective_angle_diff / speed_deg_s
            else:
                # Triangular profile: accelerate then decelerate (never reach max velocity)
                motion_time = 2.0 * np.sqrt(effective_angle_diff / acceleration_deg_s2)

            return motion_time

        print(f"\n=== 轨迹回放 ===")
        print(f"回放模式: {self.args.mode}")
        print(f"控制模式: {self.arm} arm(s)")
        replay = input(f"是否回放轨迹（{len(data)}个点）？(y/n): ").strip().lower()
        if replay != 'y':
            return

        print("[回放] 开始...")

        # 先以速度30移动到第一个点
        first_point = data[0]
        print("[回放] 移动到起始点（速度30）...")
        try:
            if self.arm == "both":
                if "q_left" in first_point and "q_right" in first_point:
                    first_gripper = [
                        first_point.get("grip_left", 0.0),
                        first_point.get("grip_right", 0.0)
                    ]
                    self.controller.set_robot_state(
                        target_joints=[first_point["q_left"], first_point["q_right"]],
                        gripper_value=first_gripper,
                        arm="both",
                        joint_format='rad',
                        speed_deg_s=30,
                        wait_for_completion=True,
                    )
                else:
                    print("[错误] 轨迹数据格式不匹配（需要双臂数据）")
                    return
            else:
                first_gripper = first_point.get("grip", 0.0)
                self.controller.set_robot_state(
                    target_joints=first_point["q"],
                    gripper_value=first_gripper,
                    arm=self.arm,
                    joint_format='rad',
                    speed_deg_s=30,
                    wait_for_completion=True,
                )
        except Exception as e:
            print(f"[警告] 移动到起始点失败: {e}")

        # 等待0.001秒
        time.sleep(0.001)
        print("[回放] 开始播放轨迹...")

        # 初始化前一个点的关节角度
        if self.arm == "both":
            prev_joints = {
                "q_left": data[0].get("q_left", []),
                "q_right": data[0].get("q_right", [])
            }
        else:
            prev_joints = data[0].get("q", [])

        if self.args.mode == 'auto':
            # 自动模式：使用直接设置，快速回放
            print("[回放] 使用直接设置模式（快速）")
            for i, point in enumerate(data):
                try:
                    # 根据速度和角度差计算运动时间
                    motion_time = _sleep_based_on_velocity(point, prev_joints if i > 0 else None)

                    if self.arm == "both":
                        gripper_val = [
                            point.get("grip_left", 0.0),
                            point.get("grip_right", 0.0)
                        ]
                        self.controller.set_robot_state(
                            target_joints=[point["q_left"], point["q_right"]],
                            gripper_value=gripper_val,
                            arm="both",
                            joint_format='rad',
                            speed_deg_s=speed_deg_s,
                            tolerance=0.3,
                            wait_for_completion=True,
                        )
                        prev_joints = {
                            "q_left": point["q_left"],
                            "q_right": point["q_right"]
                        }
                    else:
                        gripper_value = point.get("grip", 0.0)
                        self.controller.set_robot_state(
                            target_joints=point["q"],
                            gripper_value=gripper_value,
                            arm=self.arm,
                            joint_format='rad',
                            speed_deg_s=speed_deg_s,
                            tolerance=0.3,
                            wait_for_completion=True,
                        )
                        prev_joints = point["q"]

                    print(f"[回放] {i+1}/{len(data)}")
                except Exception as e:
                    print(f"[错误] 回放第{i+1}点失败: {e}")

        elif self.args.mode == 'manual':
            # 手动模式：使用插值运动，平滑回放
            print("[回放] 使用插值运动模式（平滑）")
            for i, point in enumerate(data):
                try:
                    # 根据速度和角度差计算运动时间
                    motion_time = _sleep_based_on_velocity(point, prev_joints if i > 0 else None)

                    start_time = time.time()
                    
                    if self.arm == "both":
                        gripper_val = [
                            int(point.get("grip_left", 0.0)) if point.get("grip_left") is not None else None,
                            int(point.get("grip_right", 0.0)) if point.get("grip_right") is not None else None
                        ]
                        try:
                            self.controller.set_robot_state(
                                target_joints=[point["q_left"], point["q_right"]],
                                gripper_value=gripper_val,
                                arm="both",
                                joint_format='rad',
                                speed_deg_s=speed_deg_s,
                                wait_for_completion=False
                            )
                        except Exception as e:
                            print(f"[警告] 设置关节/夹爪失败: {e}")
                        prev_joints = {
                            "q_left": point["q_left"],
                            "q_right": point["q_right"]
                        }
                    else:
                        gripper_value = int(point.get("grip", 0.0)) if point.get("grip") is not None else None
                        try:
                            self.controller.set_robot_state(
                                target_joints=point["q"],
                                gripper_value=gripper_value,
                                arm=self.arm,
                                joint_format='rad',
                                speed_deg_s=speed_deg_s,
                                wait_for_completion=False
                            )
                        except Exception as e:
                            print(f"[警告] 设置关节/夹爪失败: {e}")
                        prev_joints = point["q"]

                    # 根据计算出的运动时间等待
                    if motion_time > 0:
                        elapsed = time.time() - start_time
                        remaining_time = motion_time - elapsed
                        if remaining_time > 0:
                            # 缩放系数 = speed_deg_s / 5，反映实际运动时间与理论时间的比例关系
                            speed_factor = max(1.0, speed_deg_s / 5.0)
                            time.sleep(remaining_time / speed_factor)

                    print(f"[回放] {i+1}/{len(data)}")

                except Exception as e:
                    print(f"[错误] 回放第{i+1}点失败: {e}")

        print("[回放] 完成")

    def run(self):
        """运行主程序"""
        try:
            # 对于replay_only模式，在setup之前先验证动作是否存在
            if self.args.mode == 'replay_only':
                if not self.args.save_motion:
                    print("[错误] 回放模式必须指定 --save-motion 参数")
                    print("使用 --list-motions 查看可用动作")
                    return

                # 检查动作是否存在
                save_dir = os.path.join("example_motions", self.args.save_motion)
                if not os.path.exists(save_dir):
                    print(f"[错误] 动作 '{self.args.save_motion}' 不存在")
                    print("\n提示:")
                    available = list_available_motions()
                    if available:
                        print("可用的动作:")
                        for motion in available[:3]:
                            print(f"  {motion}")
                        if len(available) > 3:
                            print(f"  ... 还有 {len(available)-3} 个")
                    else:
                        print("未找到任何已录制的动作，请先录制")
                    return

            self.setup()

            # 根据模式执行不同操作
            if self.args.mode == 'manual':
                data = self.manual_mode()
            elif self.args.mode == 'auto':
                data = self.auto_mode()
            elif self.args.mode == 'replay_only':
                data = self.replay_only_mode()
            else:
                raise ValueError(f"不支持的模式: {self.args.mode}")

            if data:
                # 只有非replay_only模式才保存数据
                if self.args.mode != 'replay_only':
                    save_dir = self.save_data(data)

                    if save_dir:
                        print(f"\n[完成] 拖动示教完成！")
                        print(f"数据已保存到: {save_dir}")
                else:
                    print(f"\n[完成] 加载轨迹完成！")

                # 回放
                self.replay(data)

                # 恢复原始模式（对于replay_only）
                if hasattr(self, '_original_mode'):
                    self.args.mode = self._original_mode
                if hasattr(self, '_original_arm'):
                    self.arm = self._original_arm

            else:
                print("[完成] 未记录数据或加载失败")

        except KeyboardInterrupt:
            print("\n[中断] 用户中断")
        except Exception as e:
            print(f"[错误] 运行失败: {e}")


def list_available_motions() -> List[str]:
    """列出所有可用的动作"""
    motions_dir = "example_motions"
    if not os.path.exists(motions_dir):
        return []

    motions = []
    for item in os.listdir(motions_dir):
        motion_path = os.path.join(motions_dir, item)
        if os.path.isdir(motion_path):
            # 检查是否有轨迹数据
            traj_file = os.path.join(motion_path, "joint_traj.json")
            if os.path.exists(traj_file):
                motions.append(item)

    return sorted(motions)


def print_available_motions():
    """打印所有可用的动作"""
    motions = list_available_motions()

    print("=== 可用的动作列表 ===")
    if not motions:
        print("未找到任何已录制的动作")
        print("请先使用 auto 或 manual 模式录制动作")
        return

    print(f"在 example_motions/ 目录下找到 {len(motions)} 个动作:")

    for i, motion in enumerate(motions, 1):
        motion_dir = os.path.join("example_motions", motion)
        meta_path = os.path.join(motion_dir, "meta.json")

        # 读取动作信息
        info = f"{i:2d}. {motion}"
        if os.path.exists(meta_path):
            try:
                with open(meta_path, 'r', encoding='utf-8') as f:
                    meta = json.load(f)
                mode = meta.get('mode', 'unknown')
                created = meta.get('created_at', 'unknown')
                count = meta.get('count', 0)
                arm = meta.get('arm', 'both')
                info += f" (模式: {mode}, 控制: {arm}, 点数: {count}, 创建: {created})"
            except:
                pass

        print(info)

    print(f"\n使用示例:")
    if motions:
        print(f"python 09_demo_drag_teaching.py --port /dev/ttyUSB0 --mode replay_only --save-motion {motions[0]}")
