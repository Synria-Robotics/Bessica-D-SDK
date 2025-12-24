# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
Bessica_D robot DDS communication class.

This node publishes a compact state message containing:
- joint positions (14)
- joint velocities (14)
- pose data (21) = head / right_gripper_center / left_gripper_center poses

and receives command messages (format defined by the remote consumer) and
forwards them into shared memory for Isaac Lab to consume.
"""

import json
import numpy as np
from typing import Any, Dict, Optional

from dds.dds_base import DDSObject
from synria_common_sdk.remote_communication.channel import ChannelPublisher, ChannelSubscriber
from synria_common_sdk.idl.std_msgs.msg.dds_._String_ import String_


class BessicaRobotDDS(DDSObject):
    """Bessica_D robot DDS communication class."""
    
    def __init__(self, node_name: str = "bessica_robot"):
        """Initialize the Bessica robot DDS node"""
        # avoid duplicate initialization
        if hasattr(self, '_initialized'):
            return
            
        super().__init__()
        self.node_name = node_name
        self._initialized = True
        
        # setup the shared memory
        # Bessica_D has 14 DOF (7 left arm + 7 right arm) and pose data for head / grippers
        self.setup_shared_memory(
            input_shm_name="isaac_bessica_d_state",  # read joint + pose state from Isaac Lab
            output_shm_name="dds_bessica_d_cmd",     # output the command to Isaac Lab
            input_size=4096,                         # joint_pos(14) + joint_vel(14) + pose_data(21) + margin
            output_size=2048                         # joint command, modes, etc.
        )
        
        print(f"[{self.node_name}] Bessica robot DDS node initialized")
    
    def setup_publisher(self) -> bool:
        """Setup the publisher of the Bessica_D robot."""
        try:
            # Publish compact JSON state over a String_ topic.
            self.publisher = ChannelPublisher("rt/bessica_d/state", String_)
            self.publisher.Init()
            print(f"[{self.node_name}] State publisher initialized (rt/bessica_d/state)")
            return True
        except Exception as e:
            print(f"bessica_robot_dds [{self.node_name}] State publisher initialization failed: {e}")    
            return False
    
    def setup_subscriber(self) -> bool:
        """Setup the subscriber of the Bessica_D robot."""
        try:
            print(f"[{self.node_name}] Create ChannelSubscriber...")
            # Subscribe to command messages as JSON String_.
            self.subscriber = ChannelSubscriber("rt/bessica_d/cmd", String_)
            self.subscriber.Init(lambda msg: self.dds_subscriber(msg, ""), 32)
            return True
        except Exception as e:
            print(f"bessica_robot_dds [{self.node_name}] Command subscriber initialization failed: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def dds_publisher(self) -> Any:
        """Read state from shared memory and publish as a JSON String_ message."""
        try:
            data = self.input_shm.read_data()
            if data is None:
                return

            positions = data.get("joint_positions")
            velocities = data.get("joint_velocities")
            pose_data = data.get("imu_data")  # [21] = head / right / left gripper poses

            # Build a compact JSON message
            msg_dict: Dict[str, Any] = {}
            if positions is not None:
                msg_dict["joint_positions"] = positions
            if velocities is not None:
                msg_dict["joint_velocities"] = velocities
            if pose_data is not None:
                msg_dict["pose_data"] = pose_data

            if not msg_dict:
                return

            string_msg = String_(data=json.dumps(msg_dict))
            self.publisher.Write(string_msg)

        except Exception as e:
            print(f"bessica_robot_dds [{self.node_name}] Error processing publish data: {e}")

    
    def dds_subscriber(self, msg: Any, datatype: str = None) -> Dict[str, Any]:
        """Process incoming command messages and write them into shared memory.

        The expected message format is a JSON string with arbitrary keys, e.g.:
            {
                "joint_positions_cmd": [...],
                "joint_velocities_cmd": [...],
                "extra": {...}
            }
        """
        try:
            if not hasattr(msg, "data"):
                return {}

            try:
                cmd_data = json.loads(msg.data)
            except Exception:
                print(f"bessica_robot_dds [{self.node_name}] Failed to parse command JSON")
                return {}

            # Forward raw command data into shared memory for Isaac Lab side.
            self.output_shm.write_data(cmd_data)
            return cmd_data
        except Exception as e:
            print(f"bessica_robot_dds [{self.node_name}] Error processing subscribe data: {e}")
            return {}
    
    def get_robot_command(self) -> Optional[Dict[str, Any]]:
        """Get the robot control command
        
        Returns:
            Dict: the robot control command, return None if there is no new command
        """
        if self.output_shm:
            return self.output_shm.read_data()
        return None
    
    def write_robot_state(self, joint_positions, joint_velocities, pose_data):
        """Write the robot state to the shared memory
        
        Args:
            joint_positions: the joint position list or torch.Tensor (14 joints)
            joint_velocities: the joint velocity list or torch.Tensor (14 joints)
            pose_data: the pose data list or torch.Tensor (21 values)
                      Format: [head_pos(3), head_quat(4), right_gripper_pos(3), right_gripper_quat(4), left_gripper_pos(3), left_gripper_quat(4)]
                      Order: head, right, left
                      Each quaternion is (w,x,y,z) format
        """
        if self.input_shm is None:
            return
        try:
            state_data = {
                "joint_positions": joint_positions.tolist() if hasattr(joint_positions, 'tolist') else joint_positions,
                "joint_velocities": joint_velocities.tolist() if hasattr(joint_velocities, 'tolist') else joint_velocities,
                "imu_data": pose_data.tolist() if hasattr(pose_data, 'tolist') else pose_data,
            }
            self.input_shm.write_data(state_data)
        except Exception as e:
            print(f"bessica_robot_dds [{self.node_name}] Error writing robot state: {e}")

