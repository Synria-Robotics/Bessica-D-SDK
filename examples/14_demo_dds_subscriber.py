"""
Demo 14: DDS state subscriber (14‑DOF joint positions)

This demo subscribes to the DDS topic `rt/bessica_d/state` and prints the
received joint positions in real time. It pairs with `13_demo_dds_connection.py`
which publishes the robot state.

    Topic:  rt/bessica_d/state
    Type:   String_  (JSON payload)
    Format: {"joint_positions": [float] * 14}

The joint ordering is:
- First 7 values:  left joints
- Next 7 values:   right joints
"""

import argparse
import json
import os
import sys
import time

# Ensure project root (containing `synria_common_sdk`) is on sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from synria_common_sdk.remote_communication.channel import (
    ChannelFactoryInitialize,
    ChannelSubscriber,
)
from synria_common_sdk.idl.std_msgs.msg.dds_._String_ import String_


def state_callback(msg: String_):
    """Callback to process received joint state from DDS."""
    try:
        data = json.loads(msg.data)
    except Exception as e:
        print(f"[DDS Demo 14] Failed to parse JSON: {e}")
        print(f"[DDS Demo 14] Raw message: {msg.data}")
        return

    joints = data.get("joint_positions")
    if isinstance(joints, list) and len(joints) == 14:
        left = joints[:7]
        right = joints[7:14]
        print(f"[DDS Demo 14] Received 14 joints:")
        print(f"  Left arm (7):  {left}")
        print(f"  Right arm (7):  {right}")
    else:
        print(f"[DDS Demo 14] Received data: {data}")


def main(args):
    """
    Subscribe to `rt/bessica_d/state` and print received joint positions.
    """
    # Initialize DDS factory (domain id must match publisher, default is 1)
    print(f"[DDS Demo 14] Initializing DDS factory (domain_id={args.domain_id})...")
    ChannelFactoryInitialize(args.domain_id)

    # Create DDS subscriber
    print(f"[DDS Demo 14] Creating state subscriber on rt/bessica_d/state ...")
    state_sub = ChannelSubscriber("rt/bessica_d/state", String_)
    state_sub.Init(state_callback, queueLen=32)
    print("[DDS Demo 14] State subscriber initialized. Waiting for messages...")
    print("[DDS Demo 14] Press Ctrl+C to exit.")

    try:
        # Keep process alive to receive messages
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[DDS Demo 14] Interrupted by user, stopping...")
    except Exception as e:
        print(f"✗ Error in DDS subscriber: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            state_sub.Close()
        except Exception:
            pass
        print("[DDS Demo 14] Subscriber closed.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Demo 14: Subscribe to Bessica_D joint state from DDS (rt/bessica_d/state)"
    )

    parser.add_argument(
        "--domain_id",
        type=int,
        default=1,
        help="DDS domain ID (must match publisher, default: 1)",
    )

    cmd_args = parser.parse_args()
    main(cmd_args)

