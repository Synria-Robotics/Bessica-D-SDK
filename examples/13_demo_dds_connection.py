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
Demo 13: Real‑robot DDS state publisher (14‑DOF joint positions only)

This demo connects to a real Bessica_D robot via `bessica_d_sdk`, reads the
current joint angles from both arms in real time, and publishes them to a DDS
topic:

    Topic:  rt/bessica_d/state
    Type:   String_  (JSON payload)
    Format: {"joint_positions": [float] * 14}

The joint ordering is:
- First 7 values:  left joints  (as returned by `get_joints("left")`)
- Next 7 values:   right joints (as returned by `get_joints("right")`)

This matches the real‑robot 14‑DOF interface used by `BessicaRobotDDS`.
"""

import argparse
import json
import os
import sys
import time
import select

# Ensure project root (containing `synria_common_sdk`) is on sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import bessica_d_sdk
from bessica_d_sdk.utils.logger import logger

from synria_common_sdk.remote_communication.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from synria_common_sdk.idl.std_msgs.msg.dds_._String_ import String_


def create_robot(args):
    """Create a Bessica robot instance using standard SDK factory."""
    robot = bessica_d_sdk.create_robot(
        port=args.port,
        baudrate=args.baudrate,
        robot_version=args.robot_version,
        debug_mode=False,
        speed_deg_s=args.speed_deg_s,
    )
    return robot


def main(args):
    """
    Connect to the real robot and continuously publish its joint state
    (14‑DOF) to `rt/bessica_d/state` as JSON.
    """
    # Initialize DDS factory (domain id 0 for real robot, 1 for simulation)
    # For real robot, use channel 0 to match teleop_hand_and_arm_sdk.py
    logger.info(f"[DDS Demo 13] Initializing DDS factory (domain_id={args.domain_id})...")
    ChannelFactoryInitialize(args.domain_id)

    # Create DDS publisher (state) and prepare subscriber (commands)
    logger.info("[DDS Demo 13] Creating state publisher on rt/bessica_d/state ...")
    state_pub = ChannelPublisher("rt/bessica_d/state", String_)
    state_pub.Init()
    logger.info("[DDS Demo 13] State publisher initialized.")

    # Create and connect robot
    robot = create_robot(args)
    try:
        logger.info("[DDS Demo 13] Connecting to robot...")
        if not robot.connect():
            print("✗ Connection failed, please check serial port and power")
            return

        # Optional: set speed
        robot.set_speed(speed_deg_s=args.speed_deg_s)

        # Set up DDS command subscriber to control the real robot
        logger.info("[DDS Demo 13] Setting up command subscriber on rt/bessica_d/cmd ...")

        def cmd_callback(msg: String_):
            """Apply incoming joint position commands (14‑DOF) to the real robot."""
            try:
                data = json.loads(msg.data)
            except Exception as e:
                logger.error(f"[DDS Demo 13] Failed to parse command JSON: {e}")
                return

            joints_cmd = data.get("joint_positions_cmd")
            if not isinstance(joints_cmd, list) or len(joints_cmd) != 14:
                logger.warning(
                    "[DDS Demo 13] Invalid joint_positions_cmd, expected 14 values, got: %s",
                    joints_cmd,
                )
                return

            left_cmd = joints_cmd[:7]
            right_cmd = joints_cmd[7:]
            success = robot.set_joint_target(
                target_joints=[left_cmd, right_cmd],
                arm="both",
                joint_format="rad",  # commands are expected in radians
                wait=False,
                tolerance=0.1,
            )
            if not success:
                logger.warning("[DDS Demo 13] Failed to apply joint_positions_cmd to robot")

        cmd_sub = ChannelSubscriber("rt/bessica_d/cmd", String_)
        cmd_sub.Init(cmd_callback, queueLen=32)
        logger.info("[DDS Demo 13] Command subscriber initialized.")

        logger.info("[DDS Demo 13] Successfully connected. Start publishing joint state and listening for commands...")

        publish_hz = args.publish_hz
        dt = 1.0 / publish_hz if publish_hz > 0 else 0.02

        logger.info("Press 'q' then Enter to exit, or Ctrl+C.")

        while True:
            # Read both arms' joints; API returns radians
            joints_left = robot.get_joints(arm="left")
            joints_right = robot.get_joints(arm="right")

            if (
                not isinstance(joints_left, list)
                or not isinstance(joints_right, list)
                or len(joints_left) != 7
                or len(joints_right) != 7
            ):
                logger.warning(
                    "[DDS Demo 13] Failed to read joint angles, left=%s, right=%s",
                    joints_left,
                    joints_right,
                )
                time.sleep(dt)
                continue

            # 14‑DOF: 7 left + 7 right
            joint_positions = joints_left + joints_right

            msg_dict = {"joint_positions": joint_positions}
            msg = String_(data=json.dumps(msg_dict))
            state_pub.Write(msg)

            if args.verbose:
                logger.info("[DDS Demo 13] Published joints: %s", joint_positions)

            time.sleep(dt)

            # Non-blocking keyboard detection to exit gracefully
            try:
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    ch = sys.stdin.readline().strip()
                    if ch.lower() == "q":
                        logger.info("[DDS Demo 13] 'q' pressed, exiting loop...")
                        break
            except Exception:
                # If stdin is not a TTY or select fails, just ignore and continue.
                pass

    except KeyboardInterrupt:
        logger.info("[DDS Demo 13] Interrupted by user, stopping...")
    except Exception as e:
        print(f"✗ Error in DDS state publisher: {e}")
        import traceback

        traceback.print_exc()
    finally:
        try:
            robot.disconnect()
        except Exception:
            pass
        logger.info("[DDS Demo 13] Robot disconnected.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Demo 13: Publish real Bessica_D joint state to DDS (rt/bessica_d/state)"
    )

    # Robot configuration (aligned with other demos, compact style)
    parser.add_argument("--port", type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument("--baudrate", type=int, default=1000000, help="波特率 (默认: 1000000)")
    parser.add_argument("--robot_version", type=str, default="v1_0", help="机械臂版本 (默认: v1_0)")
    parser.add_argument("--speed_deg_s", type=float, default=40.0, help="关节运动速度 (度/秒, 默认: 40.0)")
    parser.add_argument("--publish_hz", type=float, default=50.0, help="DDS 发布频率 (Hz, 默认: 50.0)")
    parser.add_argument("--domain_id", type=int, default=0, help="DDS domain ID (0 for real robot, 1 for simulation, 默认: 0)")
    parser.add_argument("--verbose", action="store_true", help="打印已发布的关节状态")

    cmd_args = parser.parse_args()
    main(cmd_args)

