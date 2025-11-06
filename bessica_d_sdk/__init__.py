"""
Bessica-D SDK v3.0.0 - 与 RoboCore 桥接

架构层次：
- 用户层: SynriaBessicaRobotAPI (统一用户接口)
- 规划层: 使用 RoboCore 轨迹规划功能
- 控制层: MotionController (运动控制)
- 执行层: HardwareExecutor (硬件执行)
- 硬件层: ServoDriver (底层驱动)
- 运动学层: 使用 RoboCore 运动学功能 (FK/IK/Jacobian)

Bridge with RoboCore:
- robocore.kinematics: 提供 FK/IK/Jacobian 计算
- robocore.planning: 提供轨迹规划功能
- robocore.modeling: 提供 RobotModel
"""

from .api import SynriaBessicaRobotAPI
from .hardware import ServoDriver

# Import from RoboCore for kinematics and modeling
from robocore.modeling import RobotModel
from robocore.kinematics import forward_kinematics, inverse_kinematics, jacobian
from robocore.planning import (
    cubic_polynomial_trajectory,
    quintic_polynomial_trajectory,
    linear_joint_trajectory,
    linear_cartesian_trajectory,
    trapezoidal_velocity_profile
)

__version__ = "3.0.0"
__author__ = "Synria Robotics"
__description__ = "Bessica-D机械臂SDK v3.0.0 - Bridged with RoboCore"

# Re-export RoboCore components for convenience
__all__ = [
    # Core API
    "SynriaBessicaRobotAPI",
    "create_robot",
    
    # Hardware Layer
    "ServoDriver",
    
    # RoboCore - Modeling
    "RobotModel",
    
    # RoboCore - Kinematics
    "forward_kinematics",
    "inverse_kinematics",
    "jacobian",
    
    # RoboCore - Planning
    "cubic_polynomial_trajectory",
    "quintic_polynomial_trajectory",
    "linear_joint_trajectory",
    "linear_cartesian_trajectory",
    "trapezoidal_velocity_profile",
]


def create_robot(
        port: str = "", 
        baudrate: int = 1000000, 
        robot_version: str = "v1_0",
        debug_mode: bool = False,
        speed_deg_s: float = 20.0
    ) -> SynriaBessicaRobotAPI:
    """
    创建并初始化 Bessica-D 机器人实例的便捷函数。
    
    :param port: 串口端口，留空则自动搜索
    :param baudrate: 波特率，默认 1000000
    :param robot_version: 机器人版本，默认 "v1_0"
    :param debug_mode: 是否启用调试模式
    :param speed_deg_s: 默认运动速度（度/秒），默认 20.0
    :return: SynriaBessicaRobotAPI 实例
    """
    # 创建硬件层
    servo_driver = ServoDriver(port=port, baudrate=baudrate, debug_mode=debug_mode)
    
    # 创建用户层（RobotModel 在 API 内部创建）
    robot = SynriaBessicaRobotAPI(
        servo_driver=servo_driver,
        robot_version=robot_version,
        speed_deg_s=speed_deg_s
    )
    
    return robot
