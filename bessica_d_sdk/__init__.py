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
Bessica-D SDK v3.1.1 - Bridged with RoboCore

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
from synriard import get_model_path


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




def create_mujoco_controller(robot_version: str = "v1_0"):
	"""Create dual-arm interactive MuJoCo controller (InteractiveDualArmIK).

	:return: InteractiveDualArmIK instance (raises exception if dependencies unavailable)
	"""
	if InteractiveDualArmIK is None:
		raise RuntimeError("robocore_main not ready, cannot create InteractiveDualArmIK")
	model_path = get_model_path("Bessica_D", version="v1_1", variant="skeleton", model_format="mjcf")

	left_end = "left_arm_link7"
	right_end = "right_arm_link7"
	return InteractiveDualArmIK(model_path, left_end, right_end)  # type: ignore


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
