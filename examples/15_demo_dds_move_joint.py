"""
Demo 15: Test DDS joint commands for real robot

Simple test script to send joint position commands via DDS to test the real robot bridge
(e.g. `12_demo_vr_teleoperation.py`).

DDS command interface:

    Topic:  rt/bessica_d/cmd
    Type:   String_ (JSON payload)
    Format: {
        "joint_positions_cmd": [float] * 7 or [float] * 14,  # radians
        "arm": "left" | "right" | "both"  # which arm(s) to control
    }

Usage:
    # Test both arms (14 joints, all to 20 degrees)
    python 15_demo_dds_move_joint.py --arm both --angle_deg 20.0

    # Test left arm only (7 joints)
    python 15_demo_dds_move_joint.py --arm left --angle_deg 20.0

    # Test right arm only (7 joints)
    python 15_demo_dds_move_joint.py --arm right --angle_deg 20.0

    # Test with real robot bridge (channel 0)
    python 15_demo_dds_move_joint.py --domain_id 0 --arm both --angle_deg 15.0
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


def main(args):
    """
    Publish joint position commands over DDS to test the real robot bridge.
    """
    print("=" * 70)
    print("Demo 15: DDS Joint Command Test")
    print("=" * 70)
    
    print(f"\n[DDS Demo 15] Initializing DDS factory (domain_id={args.domain_id})...")
    ChannelFactoryInitialize(args.domain_id)

    print("[DDS Demo 15] Creating command publisher on rt/bessica_d/cmd ...")
    pub = ChannelPublisher("rt/bessica_d/cmd", String_)
    pub.Init()
    print("[DDS Demo 15] Command publisher initialized.")

    # Convert angle from degrees to radians
    angle_rad = args.angle_deg * math.pi / 180.0
    
    # Build joint command based on arm selection
    if args.arm == "both":
        # Both arms: 14 joints (7 left + 7 right)
        joint_positions_cmd = [angle_rad] * 14
        print(f"[DDS Demo 15] Command: both arms, 14 joints, {args.angle_deg}° ({angle_rad:.4f} rad)")
    elif args.arm == "left":
        # Left arm only: 7 joints
        joint_positions_cmd = [angle_rad] * 7
        print(f"[DDS Demo 15] Command: left arm, 7 joints, {args.angle_deg}° ({angle_rad:.4f} rad)")
    elif args.arm == "right":
        # Right arm only: 7 joints
        joint_positions_cmd = [angle_rad] * 7
        print(f"[DDS Demo 15] Command: right arm, 7 joints, {args.angle_deg}° ({angle_rad:.4f} rad)")
    else:
        print(f"[DDS Demo 15] Error: Invalid arm selection: {args.arm}")
        return

    # Build DDS message
    msg_dict = {
        "joint_positions_cmd": joint_positions_cmd,
        "arm": args.arm
    }
    msg = String_(data=json.dumps(msg_dict))

    print(f"\n[DDS Demo 15] Sending command...")
    print(f"  Arm: {args.arm}")
    print(f"  Joints: {len(joint_positions_cmd)}")
    print(f"  Angle: {args.angle_deg}° ({angle_rad:.4f} rad)")
    
    # Send command
    pub.Write(msg)
    print(f"[DDS Demo 15] ✓ Command sent")

    # Optionally send the command multiple times to ensure it is received
    if args.repeat > 1:
        print(f"\n[DDS Demo 15] Resending command {args.repeat - 1} more time(s)...")
        for i in range(1, args.repeat):
            time.sleep(args.interval)
            pub.Write(msg)
            print(f"[DDS Demo 15] Resent command ({i + 1}/{args.repeat})")

    print("\n" + "=" * 70)
    print("[DDS Demo 15] Done.")
    print("=" * 70)
    print("\n💡 Make sure 12_demo_vr_teleoperation.py is running to receive commands!")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Demo 15: Test DDS joint position commands (rt/bessica_d/cmd)"
    )

    parser.add_argument(
        "--domain_id",
        type=int,
        default=0,
        help="DDS domain ID (0 for real robot, 1 for simulation, default: 0)",
    )
    parser.add_argument(
        "--arm",
        type=str,
        choices=["left", "right", "both"],
        default="both",
        help="Arm to control: left, right, or both (default: both)",
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
