"""
Bessica-D SDK
==============

此包提供了与 Bessica-D 灵越双臂交互的工具。
主要通过 `ArmController` 类进行控制。
"""

__version__ = "0.1.0"
__author__ = "Xuanya Robotics" 

from .controller import ArmController
from .data_parser import JointState

__all__ = [
    "ArmController",
    "JointState"
]