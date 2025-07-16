from typing import List, Union
import time
from bessica_d_sdk.controller import ArmController


def control_move(controller: ArmController,
                 current_angles: Union[List[float], List[List[float]]],
                 target_angles: Union[List[float], List[List[float]]],
                 arm: str = 'left_arm',
                 steps: int = 120,
                 delay: float = 0.03) -> bool:
    """
    缓慢插值移动机械臂到目标角度（支持单臂或双臂）

    Args:
        controller: ArmController 实例
        current_angles: 单臂：长度为 7 的角度列表；双臂：[[left], [right]]
        target_angles: 同上，目标角度
        arm: "left_arm", "right_arm", 或 "both"
        steps: 插值步数
        delay: 每步延迟时间（秒）

    Returns:
        bool: 控制是否成功
    """
    if arm == "both":
        for step in range(1, steps + 1):
            interp_angles = [
                [c + (t - c) * step / steps for c, t in zip(current_angles[0], target_angles[0])],
                [c + (t - c) * step / steps for c, t in zip(current_angles[1], target_angles[1])]
            ]
            if not controller.set_joint_angles(interp_angles, arm="both", wait_for_completion=False):
                return False
            time.sleep(delay)

    elif arm in ["left_arm", "right_arm"]:
        for step in range(1, steps + 1):
            interp_angles = [c + (t - c) * step / steps for c, t in zip(current_angles, target_angles)]
            if not controller.set_joint_angles(interp_angles, arm=arm, wait_for_completion=False):
                return False
            time.sleep(delay)
    return True

def move_to_zero(controller: ArmController, arm: str = "both", interpolate: bool = True) -> bool:
    """
    将指定机械臂或双臂移动到零位。
    """
    target = [[0.0] * 7, [0.0] * 7] if arm == "both" else [0.0] * 7
    current = controller.read_joint_angles(arm)
    if interpolate:
        return control_move(controller, current, target, arm=arm)
    else:
        return controller.set_joint_angles(joint_angles=target, arm=arm, wait_for_completion=False)


def move_joint(controller: ArmController, joint_id: int, angle_deg: float, arm: str = "left_arm", interpolate: bool = True) -> bool:
    """
    控制单个机械臂的某个关节角度（单位：角度）

    Args:
        joint_id: 要控制的关节编号（0~6），对应1-7关节
        angle_deg: 目标角度（单位：度）
        arm: "left_arm" 或 "right_arm"
        interpolate: 是否插值移动（默认插值，防止机械臂大幅度移动）

    Returns:
        bool: 控制是否成功
    """
    current = controller.read_joint_angles(arm)
    target = list(current)
    target[joint_id] = angle_deg * controller.DEG_TO_RAD

    if interpolate:
        return control_move(controller, current, target, arm=arm)
    else:
        return controller.set_joint_angles(joint_angles=target, arm=arm, wait_for_completion=False)

def move_all_joints(controller: ArmController, angles_deg: List[float], arm: str = "left_arm", interpolate: bool = True) -> bool:
    """
    控制单臂所有关节角度（单位：度）

    Args:
        angles_deg: 长度为 7 的关节角度列表
        arm: 控制哪一个臂
        interpolate: 是否插值移动

    Returns:
        bool: 控制是否成功
    """
    if len(angles_deg) != 7:
        print("必须提供7个关节角度")
        return False

    current = controller.read_joint_angles(arm)
    target = [a * controller.DEG_TO_RAD for a in angles_deg]

    if interpolate:
        return control_move(controller, current, target, arm=arm)
    else:
        return controller.set_joint_angles(joint_angles=target, arm=arm, wait_for_completion=True)

def move_dual_joint(controller: ArmController, angles_left_deg: List[float], angles_right_deg: List[float], interpolate: bool = True) -> bool:
    """
    控制双臂全部 14 个关节（单位：度）

    Args:
        angles_left_deg: 左臂 7 个关节角度
        angles_right_deg: 右臂 7 个关节角度
        interpolate: 是否插值移动

    Returns:
        bool: 控制是否成功
    """
    if len(angles_left_deg) != 7 or len(angles_right_deg) != 7:
        print("每个机械臂必须提供 7 个关节角度")
        return False

    current = controller.read_joint_angles("both")
    target = [
        [a * controller.DEG_TO_RAD for a in angles_left_deg],
        [a * controller.DEG_TO_RAD for a in angles_right_deg]
    ]

    if interpolate:
        return control_move(controller, current, target, arm="both")
    else:
        return controller.set_joint_angles(joint_angles=target, arm="both", wait_for_completion=True)

def set_gripper_angle(controller: ArmController, angle_deg: float, arm: str = "left_arm", wait: bool = True) -> bool:
    """
    控制单个机械臂夹爪的角度（单位：度）

    Args:
        angle_deg: 目标角度（0~100 度）
        arm: "left_arm" 或 "right_arm"
        wait: 是否等待完成

    Returns:
        bool: 控制是否成功
    """
    angle_rad = angle_deg * controller.DEG_TO_RAD
    return controller.set_gripper(angle_rad, arm=arm, wait_for_completion=wait)

def set_dual_gripper(controller: ArmController, left_deg: float, right_deg: float, wait: bool = True) -> bool:
    """
    分别控制左右臂夹爪角度（单位：度）

    Args:
        left_deg: 左臂夹爪角度（度）
        right_deg: 右臂夹爪角度（度）
        wait: 是否等待完成

    Returns:
        bool: 控制是否成功
    """
    angles_rad = (left_deg * controller.DEG_TO_RAD, right_deg * controller.DEG_TO_RAD)
    return controller.set_gripper(angles_rad, arm="both", wait_for_completion=wait)


def open_gripper(controller: ArmController, angle_deg: float = 0.0, arm: str = "both", wait: bool = True) -> bool:
    """
    打开夹爪（默认 0°）

    Args:
        angle_deg: 打开角度（单臂）或最大开度（双臂）
        arm: 控制哪个臂或 "both"
        wait: 是否等待完成
    """
    if arm == "both":
        return set_dual_gripper(controller, angle_deg, angle_deg, wait=wait)
    else:
        return set_gripper_angle(controller, angle_deg, arm=arm, wait=wait)


def close_gripper(controller: ArmController, arm: str = "both", wait: bool = True) -> bool:
    """
    关闭夹爪（设置为 0°）

    Args:
        arm: 控制哪个臂或 "both"
        wait: 是否等待完成
    """
    return open_gripper(controller, angle_deg=100.0, arm=arm, wait=wait)


def print_joint_angles(controller: ArmController, arm: str = "both"):
    """
    打印当前关节角度（度）。
    """
    joint_angles = controller.read_joint_angles(arm)
    if arm == "both":
        left_deg = [round(a * controller.RAD_TO_DEG, 2) for a in joint_angles[0]]
        right_deg = [round(a * controller.RAD_TO_DEG, 2) for a in joint_angles[1]]
        print(f"左臂关节角度: {left_deg}")
        print(f"右臂关节角度: {right_deg}")
    else:
        angles_deg = [round(a * controller.RAD_TO_DEG, 2) for a in joint_angles]
        print(f"{arm} 关节角度: {angles_deg}")
