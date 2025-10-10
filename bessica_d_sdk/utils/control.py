from typing import List, Union
import time
from bessica_d_sdk.controller import ArmController
import logging
import copy

# 配置日志
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("Control_utils")
import logging

# 配置日志
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("Control_utils")


def ease_in_out_cubic(x: float) -> float:
    return 4 * x**3 if x < 0.5 else 1 - pow(-2 * x + 2, 3) / 2

def control_move(controller: ArmController,
                 current_angles: Union[List[float], List[List[float]]],
                 target_angles: Union[List[float], List[List[float]]],
                 arm: str,
                 steps: int = 300,
                 delay: float = 0.004) -> bool:
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
            ratio = ease_in_out_cubic(step / steps)
            interp_angles_left = [
                c + (t - c) * ratio for c, t in zip(current_angles[0], target_angles[0])
            ]
            interp_angles_right = [
                c + (t - c) * ratio for c, t in zip(current_angles[1], target_angles[1])
            ]

            if not controller.set_joint_angles(interp_angles_left, arm="left_arm", wait_for_completion=False):
                return False
            time.sleep(0.005)
            if not controller.set_joint_angles(interp_angles_right, arm="right_arm", wait_for_completion=False):
                return False
            time.sleep(0.005)
            time.sleep(delay)

    elif arm in ["left_arm", "right_arm"]:
        for step in range(1, steps + 1):
            ratio = ease_in_out_cubic(step / steps)
            interp_angles = [c + (t - c) * ratio for c, t in zip(current_angles, target_angles)]
            if not controller.set_joint_angles(interp_angles, arm=arm, wait_for_completion=False):
                return False
            time.sleep(delay)
    else:
        logger.error(f"请检查arm输入指令是否正确，当前arm值为{arm}")
        return False

    return True

def wait_for_valid_state(controller: ArmController, arm: str, timeout: float=5.0):
    start_time = time.time()
    while time.time() - start_time < timeout:
        js = controller.read_joint_state(arm)
        if js and max(abs(a) for a in js.angles) > 1e-3:
            return True
        time.sleep(0.05)
    print(f"超时：未收到 {arm} 状态数据")
    return False

def move_to_zero(controller: ArmController, arm: str = None, interpolate: bool = True) -> bool:
    """
    将指定机械臂移动到零位 (新协议仅支持单臂控制帧)。
    若 arm == 'both' 则依次对 left_arm 与 right_arm 执行。
    interpolate=True 时使用插值逐步发送；否则直接一次到位。
    """
    if not arm:
        logger.error(f"请输入指定要控制的机械臂，当前指定机械臂为{arm}")
        return False

    if arm == "both":
        left_ok = move_to_zero(controller, 'left_arm', interpolate=interpolate)
        right_ok = move_to_zero(controller, 'right_arm', interpolate=interpolate)
        return left_ok and right_ok

    if arm not in ["left_arm", "right_arm"]:
        logger.error(f"arm 参数无效: {arm}")
        return False

    target = [0.0] * 7
    current = controller.read_joint_angles(arm)
    # 可能状态线程尚未获得数据
    if not current or len(current) != 7:
        logger.warning(f"当前未获取到 {arm} 有效角度，直接发送零位")
        return controller.set_joint_angles(joint_angles=target, arm=arm, wait_for_completion=True)

    if interpolate:
        return control_move(controller, current, target, arm=arm)
    else:
        return controller.set_joint_angles(joint_angles=target, arm=arm, wait_for_completion=True)

def move_joint(controller: ArmController, joint_id: int, angle_deg: float, arm: str = None, interpolate: bool = True) -> bool:
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
    if not arm:
        logger.error(f"请输入指定要控制的机械臂，当前指定机械臂为{arm}")
        return False
    state = controller.read_joint_state(arm)
    current = state.angles
    target = copy.deepcopy(current)
    target[joint_id] = angle_deg * controller.DEG_TO_RAD

    if interpolate:
        return control_move(controller, current, target, arm=arm)
    else:
        return controller.set_joint_angles(joint_angles=target, arm=arm, wait_for_completion=True)

def move_joints(controller: ArmController, angles_deg: List[float], arm: str = None, interpolate: bool = True) -> bool:
    """
    控制单臂所有关节角度（单位：度）

    Args:
        angles_deg: 长度为 7 的关节角度列表
        arm: 控制哪一个臂
        interpolate: 是否插值移动

    Returns:
        bool: 控制是否成功
    """
    if not arm:
        logger.error(f"请输入指定要控制的机械臂，当前指定机械臂为{arm}")
        return False 
        
    if len(angles_deg) != 7:
        logger.error("必须提供7个关节角度")
        return False

    current = controller.read_joint_angles(arm)
    target = [a * controller.DEG_TO_RAD for a in angles_deg]

    if interpolate:
        return control_move(controller, current, target, arm=arm)
    else:
        return controller.set_joint_angles(joint_angles=target, arm=arm, wait_for_completion=True)

def move_joint_dual_arm(controller: ArmController, left_joint_id: int, right_joint_id: int, angles_left_deg: float, angles_right_deg: float, interpolate: bool = True) -> bool:
    """
    控制双臂全部 14 个关节（单位：度）

    Args:
        angles_left_deg: 左臂 7 个关节角度
        angles_right_deg: 右臂 7 个关节角度
        interpolate: 是否插值移动

    Returns:
        bool: 控制是否成功
    """
    current = controller.read_joint_angles(arm='both')
    target = copy.deepcopy(current)

    target[0][left_joint_id] = angles_left_deg * controller.DEG_TO_RAD
    target[1][right_joint_id] = angles_right_deg * controller.DEG_TO_RAD

    if interpolate:
        return control_move(controller, current, target, arm="both")
    else:
        return controller.set_joint_angles(joint_angles=target, arm="both", wait_for_completion=True)

def move_joints_dual_arm(controller: ArmController, 
                         left_angles_deg: List[float], 
                         right_angles_deg: List[float], 
                         interpolate: bool = True) -> bool:
    """
    控制双臂全部 14 个关节（单位：度）

    Args:
        left_angles_deg: 左臂 7 个关节角度
        right_angles_deg: 右臂 7 个关节角度
        interpolate: 是否插值移动

    Returns:
        bool: 控制是否成功
    """
    if len(left_angles_deg) != 7 or len(right_angles_deg) != 7:
        logger.error("每个机械臂必须提供 7 个关节角度")
        return False

    current = controller.read_joint_angles(arm="both")
    target = [
        [a * controller.DEG_TO_RAD for a in left_angles_deg],
        [a * controller.DEG_TO_RAD for a in right_angles_deg],
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
    controller.set_gripper(angles_rad[0], arm="left_arm", wait_for_completion=wait)
    time.sleep(0.1)
    controller.set_gripper(angles_rad[1], arm="right_arm", wait_for_completion=wait)
    time.sleep(0.1)
    

def open_gripper(controller: ArmController, angle_deg: float = 100.0, arm: str = "both", wait: bool = True) -> bool:
    """
    打开夹爪（默认 100°）

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
    return open_gripper(controller, angle_deg=0.0, arm=arm, wait=wait)

def print_joint_angles(controller: ArmController, arm: str = "both", read_gripper: bool=True):
    """
    打印当前关节角度（度）。
    """
    joint_angles = controller.read_joint_angles(arm)
    if arm == "both":
        left_deg = [round(a * controller.RAD_TO_DEG, 2) for a in joint_angles[0]]
        right_deg = [round(a * controller.RAD_TO_DEG, 2) for a in joint_angles[1]]
        print(f"左臂关节角度: {left_deg}")
        print(f"右臂关节角度: {right_deg}")
        if read_gripper:
            print_gripper_angles(controller, arm)
    else:
        angles_deg = [round(a * controller.RAD_TO_DEG, 2) for a in joint_angles]
        print(f"{arm} 关节角度: {angles_deg}")
        if read_gripper:
            print_gripper_angles(controller, arm)

def print_gripper_angles(controller: ArmController, arm: str = "both"):
    """
    打印当前关节夹爪角度（度）。（0~100度）
    """
    gripper_angles = controller.read_gripper_data(arm)
    if arm == "both":
        left_deg = round(gripper_angles[0] * controller.RAD_TO_DEG, 2)
        right_deg = round(gripper_angles[1] * controller.RAD_TO_DEG, 2)
        print(f"左夹爪角度: {left_deg}")
        print(f"右夹爪角度: {right_deg}")
        
    else:
        angles_deg = round(gripper_angles * controller.RAD_TO_DEG, 2)
        if arm == 'left_arm':  
            print(f"左夹爪角度: {angles_deg}")
        else:
            print(f"右夹爪角度: {angles_deg}")