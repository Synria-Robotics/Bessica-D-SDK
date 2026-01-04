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
Bessica_D robot DDS communication class (real robot / 14‑DOF joint bridge).

This node now uses a **joint‑only** DDS interface that is suitable for
real‑robot control as well as simulation bridges.  It:

- Publishes a compact state message containing:
  - ``joint_positions``: list[float] length 14 (7 left + 7 right)
- Subscribes to a command message containing:
  - ``joint_positions_cmd``: list[float] length 14 (7 left + 7 right)

The exact semantics of the command are left to the consumer (e.g. direct
position command, desired target for a controller, etc.).

Internally this class still uses shared memory to communicate with the rest
of the system (e.g. an Isaac or hardware controller process).  The shared
memory payload is expected to be a dict with at least:

    {
        "joint_positions": [float] * 14
    }

Any extra keys are ignored for DDS publishing.
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
        # Bessica_D has 14 DOF (7 left arm + 7 right arm)
        self.setup_shared_memory(
            input_shm_name="sdk_bessica_d_state",  # read joint state from Isaac / controller
            output_shm_name="dds_bessica_d_cmd",     # output the command to Isaac / controller
            # Plenty of margin for small JSON payloads; we only need 14 joint values.
            input_size=1024,
            output_size=1024
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
        """Read state from shared memory and publish as a JSON String_ message.

        Real‑robot / 14‑DOF format:

            {
                "joint_positions": [float] * 14
            }
        """
        try:
            data = self.input_shm.read_data()
            if data is None:
                return

            # We are only interested in joint positions for the real‑robot interface.
            positions = data.get("joint_positions")
            if positions is None:
                return

            # Ensure it is JSON‑serializable
            if hasattr(positions, "tolist"):
                positions = positions.tolist()

            msg_dict: Dict[str, Any] = {
                "joint_positions": positions,
            }

            string_msg = String_(data=json.dumps(msg_dict))
            self.publisher.Write(string_msg)

        except Exception as e:
            print(f"bessica_robot_dds [{self.node_name}] Error processing publish data: {e}")

    
    def dds_subscriber(self, msg: Any, datatype: str = None) -> Dict[str, Any]:
        """Process incoming command messages and write them into shared memory.

        Expected JSON command format (real‑robot / 14‑DOF):

            {
                "joint_positions_cmd": [float] * 14
            }

        Any additional keys are forwarded as‑is into shared memory.
        """
        try:
            if not hasattr(msg, "data"):
                return {}

            try:
                cmd_data = json.loads(msg.data)
            except Exception:
                print(f"bessica_robot_dds [{self.node_name}] Failed to parse command JSON")
                return {}

            # Forward raw command data into shared memory for controller side.
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
    
    def write_robot_state(self, joint_positions):
        """Write the robot state (joint positions only) to shared memory.

        Args:
            joint_positions: list / numpy.ndarray / torch.Tensor of length 14
                             (7 left arm + 7 right arm), in radians.
        """
        if self.input_shm is None:
            return
        try:
            state_data = {
                "joint_positions": joint_positions.tolist()
                if hasattr(joint_positions, 'tolist')
                else joint_positions,
            }
            self.input_shm.write_data(state_data)
        except Exception as e:
            print(f"bessica_robot_dds [{self.node_name}] Error writing robot state: {e}")

