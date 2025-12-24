"""
Bessica-D SDK v3.1.0 - Bridged with RoboCore

Architecture layers:
- User layer: SynriaBessicaRobotAPI (unified user interface)
- Planning layer: RoboCore trajectory planning
- Control layer: Motion control
- Execution layer: Hardware execution
- Hardware layer: ServoDriver (low-level driver)
- Kinematics layer: RoboCore kinematics (FK/IK/Jacobian)

Bridge with RoboCore:
- robocore.kinematics: FK/IK/Jacobian computation
- robocore.planning: Trajectory planning
- robocore.modeling: RobotModel
"""

from .api import SynriaBessicaRobotAPI
from .hardware import ServoDriver


from robocore.modeling import RobotModel
from robocore.kinematics import forward_kinematics, inverse_kinematics, jacobian
# from robocore.planning import (
# 	cubic_polynomial_trajectory,
# 	quintic_polynomial_trajectory,
# 	linear_joint_trajectory,
# 	linear_cartesian_trajectory,
# 	trapezoidal_velocity_profile
# )

from robocore.bridge.sim.mujoco.interactive_dual_arm import InteractiveDualArmIK  # type: ignore
from robocore.utils.path import get_robocore_path  # type: ignore


__version__ = "3.1.0"
__author__ = "Synria Robotics"
__description__ = "Bessica-D Robot SDK v3.1.0 - Bridged with RoboCore"

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
        variant: str = "skeleton",
        left_base_link: str = 'base_link',
        left_end_link: str = 'left_arm_link7',
        right_base_link: str = 'base_link',
        right_end_link: str = 'right_arm_link7',
    ) -> SynriaBessicaRobotAPI:
    """Create and initialize Bessica-D robot instance.
    
    :param port: Serial port, empty string for auto-search
    :param baudrate: Baud rate, default 1000000
    :param robot_version: Robot version, default "v1_0"
    :param debug_mode: Enable debug mode if True
    :param speed_deg_s: Default motion speed (deg/s), default 20.0
    :param variant: Model variant, default "skeleton"
    :param left_base_link: Left arm base link name
    :param left_end_link: Left arm end-effector link name
    :param right_base_link: Right arm base link name
    :param right_end_link: Right arm end-effector link name
    :return: SynriaBessicaRobotAPI instance
    """
    servo_driver = ServoDriver(port=port, baudrate=baudrate, debug_mode=debug_mode)
    
    robot = SynriaBessicaRobotAPI(
        servo_driver=servo_driver,
        robot_version=robot_version,
        variant=variant,
        speed_deg_s=speed_deg_s,
        left_base_link=left_base_link,
        left_end_link=left_end_link,
        right_base_link=right_base_link,
        right_end_link=right_end_link,
    )
    
    return robot


# ===== MuJoCo helpers (for demos) =====
def get_mjcf_path(robot_version: str = "v1_0") -> str:
	"""Return Bessica MuJoCo model path (if robocore_main provides path tools)."""
	import os
	# 1) Prefer local robocore_main resources bundled with package (stable path, includes meshes)
	_local_mjcf = os.path.abspath(os.path.join(
		os.path.dirname(__file__),
		"robocore_main", "assets", "robot", "mjcf",
		f"Bessica-D_{robot_version}", "Bessica-D_Interactive.xml"
	))
	if os.path.isfile(_local_mjcf):
		return _local_mjcf
	# 2) Fallback to robocore_main resource lookup
	try:
		return get_robocore_path(f"assets/robot/mjcf/Bessica-D_{robot_version}/Bessica-D_Interactive.xml")  # type: ignore
	except Exception:
		# 3) Last resort: clear error message, avoid pointing to incomplete MJCF in utils
		raise FileNotFoundError(
			f"MJCF not found: {_local_mjcf}, and robocore_main resource lookup failed. "
			f"Please ensure robocore_main resources are included or install RoboCore_full."
		)


def create_mujoco_controller(robot_version: str = "v1_0"):
	"""Create dual-arm interactive MuJoCo controller (InteractiveDualArmIK).

	:return: InteractiveDualArmIK instance (raises exception if dependencies unavailable)
	"""
	if InteractiveDualArmIK is None:
		raise RuntimeError("robocore_main not ready, cannot create InteractiveDualArmIK")
	mjcf_path = get_mjcf_path(robot_version)
	left_end = "left_arm_link7"
	right_end = "right_arm_link7"
	return InteractiveDualArmIK(mjcf_path, left_end, right_end)  # type: ignore


# Export MuJoCo helper symbols when available
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
