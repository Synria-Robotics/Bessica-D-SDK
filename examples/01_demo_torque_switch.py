"""
Demo: Robot torque control

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Warning: 
- Ensure no obstacles around the robot arm before disabling torque
- When torque is disabled, manually support the robot arm
"""

import bessica_d_sdk
from bessica_d_sdk.utils.logger import logger

def main(args):
    """Execute robot torque control.

    """
    # Initialize robot instance
    robot = bessica_d_sdk.create_robot(
        port=args.port,
    )
    
    try:
        logger.info(f"Controlling torque for: {args.arm}")
        logger.info("Please manually hold the robot arm(s).")
        logger.info("请托住机械臂以免其突然掉落。")
        input("Press Enter to disable torque...")
        robot.torque_control('off', arm=args.arm)
        logger.info(f"Torque disabled for {args.arm}.")
        
        input("Press Enter to re-enable torque...")
        robot.torque_control('on', arm=args.arm)
        logger.info(f"Torque re-enabled for {args.arm}.")
        
    
    except Exception as e:
        print(f"✗ Error: {e}")
        
        import traceback
        traceback.print_exc()
    
    finally:
        robot.disconnect()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(
        description="Robot torque control program",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Control both arms (default)
  python 01_demo_torque_switch.py --port /dev/ttyACM0
  
  # Control left arm only
  python 01_demo_torque_switch.py --port /dev/ttyACM0 --arm left
  
  # Control right arm only
  python 01_demo_torque_switch.py --port /dev/ttyACM0 --arm right
        """
    )
    
    # Robot configuration
    parser.add_argument('--port', type=str, default="", 
                       help="Serial port (e.g., /dev/ttyACM0 or COM3)")
    parser.add_argument('--arm', type=str, default='both',
                       choices=['both', 'left', 'right'],
                       help="Arm to control: 'both', 'left', or 'right' (default: both)")
    args = parser.parse_args()

    main(args)
