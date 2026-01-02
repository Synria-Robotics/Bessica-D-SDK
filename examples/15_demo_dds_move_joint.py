"""
Demo 15: Send joint commands via DDS (14‑DOF) to control the robot

This demo mirrors the behavior of `05_demo_move_joint.py`, but instead of
controlling the robot directly via the SDK, it publishes joint commands over
DDS. A separate bridge (e.g. `13_demo_dds_connection.py`) listens to the
command topic and applies the motion on the real robot.

DDS command interface (real‑robot / 14‑DOF):

    Topic:  rt/bessica_d/cmd
    Type:   String_ (JSON payload)
    Format: {
        "joint_positions_cmd": [float] * 14   # radians
    }

Joint ordering:
- First 7 values:  left joints
- Next 7 values:   right joints
"""

import argparse
import json
import math
import os
import sys
import time

# Ensure project root (containing `synria_common_sdk`) is on sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from synria_common_sdk.remote_communication.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
)
from synria_common_sdk.idl.std_msgs.msg.dds_._String_ import String_


def build_joint_command(deg: float) -> list:
    """Build a simple symmetric 14‑DOF joint command from a single degree value."""
    rad = deg * math.pi / 180.0
    left = [rad] * 7
    right = [rad] * 7
    return left + right


def main(args):
    """
    Publish one or more joint position commands over DDS to move the robot.
    """
    print(f"[DDS Demo 15] Initializing DDS factory (domain_id={args.domain_id})...")
    ChannelFactoryInitialize(args.domain_id)

    print("[DDS Demo 15] Creating command publisher on rt/bessica_d/cmd ...")
    pub = ChannelPublisher("rt/bessica_d/cmd", String_)
    pub.Init()
    print("[DDS Demo 15] Command publisher initialized.")

    # Build joint command (radians) based on requested degree value
    joint_positions_cmd = build_joint_command(args.angle_deg)
    msg_dict = {"joint_positions_cmd": joint_positions_cmd}
    msg = String_(data=json.dumps(msg_dict))

    print(
        f"[DDS Demo 15] Sending joint command: angle_deg={args.angle_deg} "
        f"({len(joint_positions_cmd)} joints, radians)"
    )
    pub.Write(msg)

    # Optionally send the command multiple times to ensure it is received
    for i in range(1, args.repeat):
        time.sleep(args.interval)
        pub.Write(msg)
        print(f"[DDS Demo 15] Resent joint command ({i + 1}/{args.repeat})")

    print("[DDS Demo 15] Done.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Demo 15: Publish joint position commands via DDS (rt/bessica_d/cmd)"
    )

    parser.add_argument(
        "--domain_id",
        type=int,
        default=1,
        help="DDS domain ID (must match bridge, default: 1)",
    )
    parser.add_argument(
        "--angle_deg",
        type=float,
        default=20.0,
        help="Target joint angle in degrees for all joints (default: 20.0)",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=1,
        help="Number of times to resend the same command (default: 1)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.5,
        help="Interval in seconds between repeated commands (default: 0.5)",
    )

    cmd_args = parser.parse_args()
    main(cmd_args)


