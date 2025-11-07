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


try:
	import sys as _sys
	from . import robocore_main as _rcf_pkg  # 本地同仓库实现
	_sys.modules.setdefault('robocore_main', _rcf_pkg)
except Exception:
	pass

from robocore.modeling import RobotModel
from robocore.kinematics import forward_kinematics, inverse_kinematics, jacobian
from robocore.planning import (
	cubic_polynomial_trajectory,
	quintic_polynomial_trajectory,
	linear_joint_trajectory,
	linear_cartesian_trajectory,
	trapezoidal_velocity_profile
)
try:
	from robocore_main.bridge.sim.mujoco.interactive_dual_arm import InteractiveDualArmIK  # type: ignore
	from robocore_main.utils.path import get_robocore_path  # type: ignore
except Exception:
	InteractiveDualArmIK = None  # type: ignore
	def get_robocore_path(_path: str) -> str:  # type: ignore
		return _path

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

    # RoboCoreMain - MuJoCo
    "InteractiveDualArmIK",
    "get_robocore_path",

]


def create_robot(
        port: str = "", 
        baudrate: int = 1000000, 
        robot_version: str = "v1_0",
        debug_mode: bool = False,
        speed_deg_s: float = 20.0,
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


# ===== MuJoCo helpers (for demos) =====
def get_mjcf_path(robot_version: str = "v1_0") -> str:
	"""返回 Bessica MuJoCo 模型路径（若 robocore_main 提供路径工具）。"""
	import os
	# 1) 优先使用本地随包的 robocore_main 资源，路径稳定且包含 meshes
	_local_mjcf = os.path.abspath(os.path.join(
		os.path.dirname(__file__),
		"robocore_main", "assets", "robot", "mjcf",
		f"Bessica-D_{robot_version}", "Bessica-D_Interactive.xml"
	))
	if os.path.isfile(_local_mjcf):
		return _local_mjcf
	# 2) 回退到 robocore_main 的资源查找
	try:
		return get_robocore_path(f"assets/robot/mjcf/Bessica-D_{robot_version}/Bessica-D_Interactive.xml")  # type: ignore
	except Exception:
		# 3) 最后手段：提示明确错误，避免误指向 utils 下不完整的 MJCF
		raise FileNotFoundError(
			f"未找到 MJCF：{_local_mjcf}，且 robocore_main 资源查找失败。请确认已包含 robocore_main 资源或安装 RoboCore_full。"
		)


def create_mujoco_controller(robot_version: str = "v1_0"):
	"""创建双臂交互式 MuJoCo 控制器（InteractiveDualArmIK）。

	返回:
		InteractiveDualArmIK 实例（若依赖不可用，将抛出异常）
	"""
	if InteractiveDualArmIK is None:
		raise RuntimeError("robocore_main 未就绪，无法创建 InteractiveDualArmIK")
	mjcf_path = get_mjcf_path(robot_version)
	left_end = "left_arm_link7"
	right_end = "right_arm_link7"
	return InteractiveDualArmIK(mjcf_path, left_end, right_end)  # type: ignore


# 可用时导出 MuJoCo 辅助符号
try:
	if InteractiveDualArmIK is not None:  # type: ignore
		__all__ += [
			"InteractiveDualArmIK",
			"get_robocore_path",
			"get_mjcf_path",
			"create_mujoco_controller",
		]
except Exception:
	pass
