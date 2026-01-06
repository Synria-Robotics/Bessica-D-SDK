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

import bessica_d_sdk
from bessica_d_sdk.utils.logger import logger
import time
import json
import threading
import numpy as np
from typing import Optional, Dict, Any

from synria_common_sdk.remote_communication.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from synria_common_sdk.idl.std_msgs.msg.dds_._String_ import String_


class RealRobotDDSBridge:
    """Bridge between DDS commands and real robot hardware."""
    
    def __init__(self, robot: bessica_d_sdk.SynriaBessicaRobotAPI, publish_rate: float = 100.0):
        """
        Initialize DDS bridge for real robot.
        
        Args:
            robot: SynriaBessicaRobotAPI instance (connected robot)
            publish_rate: State publishing rate in Hz (default: 100.0)
        """
        self.robot = robot
        self.publish_rate = publish_rate
        self.publish_interval = 1.0 / publish_rate
        
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
        """Callback for receiving DDS commands."""
        try:
            if not hasattr(msg, "data"):
                return
            
            cmd_data = json.loads(msg.data)
            
            with self._lock:
                self.latest_cmd = cmd_data
                self.cmd_received = True
            
            logger.debug(f"[RealRobotDDSBridge] Received command: {list(cmd_data.keys())}")
            
        except Exception as e:
            logger.error(f"[RealRobotDDSBridge] Error processing command: {e}")
            import traceback
            traceback.print_exc()
    
    def setup_dds(self):
        """Setup DDS publisher and subscriber."""
        # Initialize DDS channel 0 for real robot
        ChannelFactoryInitialize(0)
        logger.info("[RealRobotDDSBridge] DDS initialized (channel 0 for real robot)")
        
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
            if "joint_positions_cmd" in cmd_data:
                joint_positions_cmd = cmd_data["joint_positions_cmd"]
                
                if len(joint_positions_cmd) == 14:
                    # Split into left and right arms
                    left_angles_rad = joint_positions_cmd[:7]
                    right_angles_rad = joint_positions_cmd[7:14]
                    
                    # Apply command to robot (non-blocking, no wait)
                    success = self.robot.set_robot_state(
                        target_joints=[left_angles_rad, right_angles_rad],
                        arm="both",
                        joint_format="rad",
                        wait_for_completion=False,  # Don't wait for completion to maintain control frequency
                        tolerance=0.1,
                    )
                    
                    if success:
                        logger.debug("[RealRobotDDSBridge] Applied joint command")
                    else:
                        logger.warning("[RealRobotDDSBridge] Failed to apply joint command")
                else:
                    logger.warning(f"[RealRobotDDSBridge] Invalid joint command length: {len(joint_positions_cmd)}")
            
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
        
        self.setup_dds()
        
        self.running = True
        
        # Start state publishing thread
        self.publish_thread = threading.Thread(target=self._publish_state_loop, daemon=True)
        self.publish_thread.start()
        
        logger.info("[RealRobotDDSBridge] Started")
    
    def run(self):
        """Main control loop - processes commands and publishes state."""
        logger.info("[RealRobotDDSBridge] Entering main control loop...")
        logger.info("Press Ctrl+C to stop")
        
        try:
            while self.running:
                # Process latest command
                with self._lock:
                    if self.cmd_received and self.latest_cmd is not None:
                        cmd_to_apply = self.latest_cmd.copy()
                        self.cmd_received = False
                    else:
                        cmd_to_apply = None
                
                if cmd_to_apply:
                    self._apply_command(cmd_to_apply)
                
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
        baudrate=args.baudrate,
        robot_version=args.robot_version,
        debug_mode=args.debug,
        speed_deg_s=args.speed_deg_s
    )
    
    try:
        # Connect to robot
        logger.info("Connecting to robot...")
        if not robot.connect():
            logger.error("✗ Connection failed, please check serial port settings")
            return
        
        logger.info("✓ Robot connected")
        
        # Move to home position
        logger.info("Moving to home position...")
        robot.set_home(arm="both")
        time.sleep(2.0)
        logger.info("✓ Robot at home position")
        
        # Create and start DDS bridge
        bridge = RealRobotDDSBridge(robot, publish_rate=args.publish_rate)
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
    parser.add_argument('--baudrate', type=int, default=1000000, help="Baud rate (default: 1000000)")
    parser.add_argument('--robot_version', type=str, default="v1_0", help="Robot version (default: v1_0)")
    parser.add_argument('--speed_deg_s', type=float, default=40.0, help="Motion speed (deg/s, default: 40.0)")
    parser.add_argument('--debug', action='store_true', help="Enable debug mode")
    
    # DDS configuration
    parser.add_argument('--publish_rate', type=float, default=100.0, help="State publishing rate in Hz (default: 100.0)")
    
    args = parser.parse_args()
    
    main(args)

