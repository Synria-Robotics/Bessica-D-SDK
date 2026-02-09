"""
Demo: VR Teleoperation for Real Robot

This script receives DDS commands from VR teleoperation system and applies them to the real robot.
It also publishes robot state back via DDS for the teleoperation system to use.

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Usage:
    python 12_demo_vr_teleoperation.py --port /dev/ttyUSB0 --robot_version v1_0

Features:
- Subscribes to DDS command topic: rt/bessica_d/cmd
- Publishes robot state to DDS topic: rt/bessica_d/state
- Applies joint position commands to real robot
- Reads robot state and publishes it back
"""

import os
import sys
import time
import json
import threading
import numpy as np
from typing import Optional, Dict, Any

# Ensure project root (containing `synria_common_sdk`) is on sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import bessica_d_sdk
from bessica_d_sdk.utils.logger import logger

from synria_common_sdk.remote_communication.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from synria_common_sdk.idl.std_msgs.msg.dds_._String_ import String_


class RealRobotDDSBridge:
    """Bridge between DDS commands and real robot hardware."""
    
    def __init__(self, robot: bessica_d_sdk.SynriaBessicaRobotAPI, publish_rate: float = 100.0, channel_id: int = 0, speed_deg_s: float = 20.0):
        """
        Initialize DDS bridge for real robot.
        
        Args:
            robot: SynriaBessicaRobotAPI instance (connected robot)
            publish_rate: State publishing rate in Hz (default: 100.0)
            channel_id: DDS channel ID (0 for real robot only, 1 for simulation/both mode, default: 0)
            speed_deg_s: Motion speed in degrees per second (default: 20.0)
        """
        self.robot = robot
        self.publish_rate = publish_rate
        self.publish_interval = 1.0 / publish_rate
        self.channel_id = channel_id
        self.speed_deg_s = speed_deg_s
        
        # Command subscriber
        self.cmd_subscriber = None
        self.cmd_topic = "rt/bessica_d/cmd"
        
        # State publisher
        self.state_publisher = None
        self.state_topic = "rt/bessica_d/state"
        
        # Threading
        self.running = False
        self.publish_thread = None
        self._lock = threading.Lock()
        
        # Latest command (for thread safety)
        self.latest_cmd = None
        self.cmd_received = False
        
        logger.info("[RealRobotDDSBridge] Initialized")
    
    def _command_callback(self, msg: String_):
        """Callback for receiving DDS commands from VR teleoperation."""
        try:
            if not hasattr(msg, "data") or not msg.data:
                return
            
            cmd_data = json.loads(msg.data)
            
            with self._lock:
                self.latest_cmd = cmd_data
                self.cmd_received = True
            
            # Log command reception (debug level to avoid spam)
            if "joint_positions_cmd" in cmd_data:
                logger.debug(
                    f"[RealRobotDDSBridge] Received joint command: "
                    f"{len(cmd_data['joint_positions_cmd'])} joints"
                )
            else:
                logger.debug(f"[RealRobotDDSBridge] Received command: {list(cmd_data.keys())}")
            
        except json.JSONDecodeError as e:
            logger.error(f"[RealRobotDDSBridge] Failed to parse command JSON: {e}")
            logger.error(f"[RealRobotDDSBridge] Raw message: {msg.data if hasattr(msg, 'data') else 'N/A'}")
        except Exception as e:
            logger.error(f"[RealRobotDDSBridge] Error processing command: {e}")
            import traceback
            traceback.print_exc()
    
    def setup_dds(self, channel_id: int = 0):
        """Setup DDS publisher and subscriber.
        
        Args:
            channel_id: DDS channel ID (0 for real robot only, 1 for simulation/both mode)
        """
        # Initialize DDS channel (default 0 for real robot, 1 for simulation/both mode)
        ChannelFactoryInitialize(channel_id)
        logger.info(f"[RealRobotDDSBridge] DDS initialized (channel {channel_id})")
        
        # Setup command subscriber
        self.cmd_subscriber = ChannelSubscriber(self.cmd_topic, String_)
        self.cmd_subscriber.Init(self._command_callback, 32)
        logger.info(f"[RealRobotDDSBridge] Command subscriber initialized ({self.cmd_topic})")
        
        # Setup state publisher
        self.state_publisher = ChannelPublisher(self.state_topic, String_)
        self.state_publisher.Init()
        logger.info(f"[RealRobotDDSBridge] State publisher initialized ({self.state_topic})")
        
        # Give DDS time to establish connections
        time.sleep(0.5)
    
    def _publish_state_loop(self):
        """Publish robot state at fixed rate."""
        logger.info("[RealRobotDDSBridge] State publishing thread started")
        
        while self.running:
            start_time = time.time()
            
            try:
                # Read robot state
                state_data = self._read_robot_state()
                
                if state_data:
                    # Publish state via DDS
                    payload = json.dumps(state_data)
                    msg = String_(data=payload)
                    self.state_publisher.Write(msg)
                    logger.debug("[RealRobotDDSBridge] Published state")
                
            except Exception as e:
                logger.error(f"[RealRobotDDSBridge] Error publishing state: {e}")
            
            # Maintain publishing rate
            elapsed = time.time() - start_time
            sleep_time = max(0, self.publish_interval - elapsed)
            time.sleep(sleep_time)
        
        logger.info("[RealRobotDDSBridge] State publishing thread stopped")
    
    def _read_robot_state(self) -> Optional[Dict[str, Any]]:
        """Read current robot state from hardware."""
        try:
            # Read joint positions (in radians) using unified state API
            joints_dict = self.robot.get_robot_state("joint")
            if joints_dict is None or not isinstance(joints_dict, dict):
                logger.warning("[RealRobotDDSBridge] Failed to get joint state dict")
                return None
            
            left_angles = joints_dict.get("left")
            right_angles = joints_dict.get("right")
            if (
                not isinstance(left_angles, list)
                or not isinstance(right_angles, list)
                or len(left_angles) != 7
                or len(right_angles) != 7
            ):
                logger.warning(
                    "[RealRobotDDSBridge] Unexpected joint format: left=%s, right=%s",
                    left_angles,
                    right_angles,
                )
                return None
            
            # Flatten to 14-DOF list [left_7, right_7]
            joint_positions_flat = list(left_angles) + list(right_angles)
            
            # Read joint velocities (not available from hardware, use zeros)
            joint_velocities = [0.0] * 14
            
            # Read pose data (head/gripper poses) - simplified for now
            # Format: [head_pos(3), head_quat(4), right_gripper_pos(3), right_gripper_quat(4), left_gripper_pos(3), left_gripper_quat(4)]
            # Total: 21 values
            pose_data = [0.0] * 21  # Placeholder - can be extended with FK if needed
            
            # Try to get gripper poses using forward kinematics
            try:
                left_pose = self.robot.get_pose(arm="left")
                right_pose = self.robot.get_pose(arm="right")
                
                if left_pose and right_pose:
                    # Extract position and quaternion from pose dict
                    # New pose format (Bessica): 
                    #   {'position': np.array(3), 'quaternion_xyzw': np.array(4)}
                    left_pos = left_pose.get('position', [0.0, 0.0, 0.0])
                    left_quat_xyzw = left_pose.get('quaternion_xyzw', [0.0, 0.0, 0.0, 1.0])
                    # Convert [x, y, z, w] -> [w, x, y, z] for VR interface
                    left_quat_list = [
                        float(left_quat_xyzw[3]),
                        float(left_quat_xyzw[0]),
                        float(left_quat_xyzw[1]),
                        float(left_quat_xyzw[2]),
                    ]
                    
                    right_pos = right_pose.get('position', [0.0, 0.0, 0.0])
                    right_quat_xyzw = right_pose.get('quaternion_xyzw', [0.0, 0.0, 0.0, 1.0])
                    right_quat_list = [
                        float(right_quat_xyzw[3]),
                        float(right_quat_xyzw[0]),
                        float(right_quat_xyzw[1]),
                        float(right_quat_xyzw[2]),
                    ]
                    
                    # Build pose_data: [head_pos(3), head_quat(4), right_gripper_pos(3), right_gripper_quat(4), left_gripper_pos(3), left_gripper_quat(4)]
                    pose_data = (
                        [0.0, 0.0, 0.0] +  # head position (placeholder)
                        [1.0, 0.0, 0.0, 0.0] +  # head quaternion (placeholder)
                        list(right_pos) + list(right_quat_list) +  # right gripper
                        list(left_pos) + list(left_quat_list)  # left gripper
                    )
            except Exception as e:
                logger.debug(f"[RealRobotDDSBridge] Could not compute poses: {e}")
                # Use placeholder pose_data
            
            state_dict = {
                "joint_positions": joint_positions_flat,
                "joint_velocities": joint_velocities,
                "pose_data": pose_data
            }
            
            return state_dict
            
        except Exception as e:
            logger.error(f"[RealRobotDDSBridge] Error reading robot state: {e}")
            return None
    
    def _apply_command(self, cmd_data: Dict[str, Any]):
        """Apply command to real robot."""
        try:
            # Extract joint position command
            # Command format supports:
            #   - {"joint_positions_cmd": [14 floats], "arm": "both"}  # Both arms
            #   - {"joint_positions_cmd": [7 floats], "arm": "left"}   # Left arm only
            #   - {"joint_positions_cmd": [7 floats], "arm": "right"}  # Right arm only
            #   - {"joint_positions_cmd": [14 floats]}  # Legacy: both arms (default)
            
            if "joint_positions_cmd" not in cmd_data:
                logger.debug(f"[RealRobotDDSBridge] Command keys: {list(cmd_data.keys())}")
                return
            
            joint_positions_cmd = cmd_data["joint_positions_cmd"]
            arm = cmd_data.get("arm", "both")  # Default to "both" for backward compatibility
            
            if not isinstance(joint_positions_cmd, list):
                logger.warning(
                    f"[RealRobotDDSBridge] Invalid joint command type: expected list, "
                    f"got {type(joint_positions_cmd)}"
                )
                return
            
            # Extract gripper command if present
            gripper_value = cmd_data.get("gripper_value", None)
            
            # Validate and apply based on arm selection
            if arm == "both":
                # Both arms: expect 14 joints [left_7, right_7]
                if len(joint_positions_cmd) == 14:
                    left_angles_rad = joint_positions_cmd[:7]
                    right_angles_rad = joint_positions_cmd[7:14]
                    
                    # Handle gripper values: can be [left, right] list or single value for both
                    gripper_left = None
                    gripper_right = None
                    if gripper_value is not None:
                        if isinstance(gripper_value, list) and len(gripper_value) == 2:
                            gripper_left = gripper_value[0]
                            gripper_right = gripper_value[1]
                        elif isinstance(gripper_value, (int, float)):
                            # Single value applies to both grippers
                            gripper_left = gripper_value
                            gripper_right = gripper_value
                    
                    success = self.robot.set_robot_state(
                        target_joints=[left_angles_rad, right_angles_rad],
                        gripper_value=[gripper_left, gripper_right] if gripper_left is not None else None,
                        arm="both",
                        joint_format="rad",
                        speed_deg_s=self.speed_deg_s,
                        wait_for_completion=False,
                        tolerance=0.1,
                    )
                    
                    if success:
                        logger.debug("[RealRobotDDSBridge] Applied joint command (both arms)")
                    else:
                        logger.warning("[RealRobotDDSBridge] Failed to apply joint command (both arms)")
                else:
                    logger.warning(
                        f"[RealRobotDDSBridge] Invalid joint command length for 'both': "
                        f"expected 14, got {len(joint_positions_cmd)}"
                    )
                    
            elif arm == "left":
                # Left arm only: expect 7 joints
                if len(joint_positions_cmd) == 7:
                    # Handle gripper value: can be single value or [left, right] list
                    gripper_left = None
                    if gripper_value is not None:
                        if isinstance(gripper_value, list) and len(gripper_value) >= 1:
                            gripper_left = gripper_value[0]
                        elif isinstance(gripper_value, (int, float)):
                            gripper_left = gripper_value
                    
                    success = self.robot.set_robot_state(
                        target_joints=joint_positions_cmd,
                        gripper_value=gripper_left,
                        arm="left",
                        joint_format="rad",
                        speed_deg_s=self.speed_deg_s,
                        wait_for_completion=False,
                        tolerance=0.1,
                    )
                    
                    if success:
                        logger.debug("[RealRobotDDSBridge] Applied joint command (left arm)")
                    else:
                        logger.warning("[RealRobotDDSBridge] Failed to apply joint command (left arm)")
                else:
                    logger.warning(
                        f"[RealRobotDDSBridge] Invalid joint command length for 'left': "
                        f"expected 7, got {len(joint_positions_cmd)}"
                    )
                    
            elif arm == "right":
                # Right arm only: expect 7 joints
                if len(joint_positions_cmd) == 7:
                    # Handle gripper value: can be single value or [left, right] list
                    gripper_right = None
                    if gripper_value is not None:
                        if isinstance(gripper_value, list) and len(gripper_value) >= 2:
                            gripper_right = gripper_value[1]
                        elif isinstance(gripper_value, list) and len(gripper_value) == 1:
                            gripper_right = gripper_value[0]
                        elif isinstance(gripper_value, (int, float)):
                            gripper_right = gripper_value
                    # print("cmd: ", joint_positions_cmd)
                    success = self.robot.set_robot_state(
                        target_joints=joint_positions_cmd,
                        gripper_value=gripper_right,
                        arm="right",
                        joint_format="rad",
                        speed_deg_s=self.speed_deg_s,
                        wait_for_completion=False,
                        tolerance=0.1,
                    )
                    
                    if success:
                        logger.debug("[RealRobotDDSBridge] Applied joint command (right arm)")
                    else:
                        logger.warning("[RealRobotDDSBridge] Failed to apply joint command (right arm)")
                else:
                    logger.warning(
                        f"[RealRobotDDSBridge] Invalid joint command length for 'right': "
                        f"expected 7, got {len(joint_positions_cmd)}"
                    )
            else:
                logger.warning(
                    f"[RealRobotDDSBridge] Invalid arm selection: '{arm}'. "
                    f"Expected 'left', 'right', or 'both'"
                )
            
            # Handle other command types if needed (torques, etc.)
            if "joint_torques_cmd" in cmd_data:
                logger.debug("[RealRobotDDSBridge] Torque commands not yet implemented")
                
        except Exception as e:
            logger.error(f"[RealRobotDDSBridge] Error applying command: {e}")
            import traceback
            traceback.print_exc()
    
    def start(self):
        """Start the DDS bridge."""
        if self.running:
            logger.warning("[RealRobotDDSBridge] Already running")
            return
        
        self.setup_dds(self.channel_id)
        
        self.running = True
        
        # Start state publishing thread
        self.publish_thread = threading.Thread(target=self._publish_state_loop, daemon=True)
        self.publish_thread.start()
        
        logger.info("[RealRobotDDSBridge] Started")
    
    def run(self):
        """Main control loop - processes commands and publishes state."""
        logger.info("[RealRobotDDSBridge] Entering main control loop...")
        logger.info("Waiting for VR teleoperation commands...")
        logger.info("Press Ctrl+C to stop")
        
        cmd_count = 0
        last_log_time = time.time()
        
        try:
            while self.running:
                # Process latest command
                with self._lock:
                    if self.cmd_received and self.latest_cmd is not None:
                        cmd_to_apply = self.latest_cmd.copy()
                        self.cmd_received = False
                        cmd_count += 1
                    else:
                        cmd_to_apply = None
                
                if cmd_to_apply:
                    self._apply_command(cmd_to_apply)
                
                # Log command rate periodically
                current_time = time.time()
                if current_time - last_log_time >= 5.0:  # Every 5 seconds
                    if cmd_count > 0:
                        rate = cmd_count / (current_time - last_log_time + 5.0)
                        logger.info(f"[RealRobotDDSBridge] Command rate: {rate:.1f} Hz ({cmd_count} commands)")
                    else:
                        logger.info("[RealRobotDDSBridge] Waiting for commands...")
                    cmd_count = 0
                    last_log_time = current_time
                
                # Small sleep to prevent busy waiting
                time.sleep(0.001)
                
        except KeyboardInterrupt:
            logger.info("[RealRobotDDSBridge] Interrupted by user")
        finally:
            self.stop()
    
    def stop(self):
        """Stop the DDS bridge."""
        if not self.running:
            return
        
        logger.info("[RealRobotDDSBridge] Stopping...")
        self.running = False
        
        if self.publish_thread:
            self.publish_thread.join(timeout=2.0)
        
        logger.info("[RealRobotDDSBridge] Stopped")


def main(args):
    """Main function."""
    logger.info("=" * 70)
    logger.info("VR Teleoperation - Real Robot Bridge")
    logger.info("=" * 70)
    
    # Initialize robot
    robot = bessica_d_sdk.create_robot(
        port=args.port,
        robot_version=args.robot_version
    )
    
    try:
        # Move to home position
        logger.info("Moving to home position...")
        robot.set_home(arm="both")
        time.sleep(2.0)
        logger.info("✓ Robot at home position")
        
        # Create and start DDS bridge
        bridge = RealRobotDDSBridge(robot, publish_rate=args.publish_rate, channel_id=args.channel_id, speed_deg_s=args.speed_deg_s)
        bridge.start()
        
        # Run main loop
        bridge.run()
        
    except Exception as e:
        logger.error(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        logger.info("Disconnecting robot...")
        robot.disconnect()
        logger.info("✓ Robot disconnected")


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description="VR Teleoperation - Real Robot Bridge")
    
    # Robot configuration
    parser.add_argument('--port', type=str, default="", help="Serial port (e.g., /dev/ttyUSB0 or COM3)")
    parser.add_argument('--robot_version', type=str, default="v1_1", help="Robot version (default: v1_1)")
    parser.add_argument('--speed_deg_s', type=float, default=5.0, help="Motion speed (deg/s, default: 40.0)")    
    # DDS configuration
    parser.add_argument('--publish_rate', type=float, default=30.0, help="State publishing rate in Hz (default: 100.0)")
    parser.add_argument('--channel_id', type=int, default=0, choices=[0, 1], 
                        help="DDS channel ID: 0 for real robot only, 1 for simulation/both mode (default: 0)")
    
    args = parser.parse_args()
    
    main(args)

