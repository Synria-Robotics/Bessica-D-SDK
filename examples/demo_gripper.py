from bessica_d_sdk import ArmController
from bessica_d_sdk.utils import control
import time

def main():
    controller = ArmController()
    if not controller.connect():
        print("未能连接机械臂")
        return False

    print("=== 单臂夹爪控制 ===")
    control.open_gripper(controller, arm='right_arm', wait=False)
    time.sleep(1)
    control.set_gripper_angle(controller, angle_deg=50, arm='right_arm', wait=False)
    time.sleep(1)
    control.close_gripper(controller, arm="right_arm", wait=False)
    time.sleep(1)
    
    

    print("=== 双臂夹爪控制 ===")
    control.open_gripper(controller, arm="both", wait=False)
    time.sleep(1)
    control.set_dual_gripper(controller, left_deg=50, right_deg=30,wait=False)
    time.sleep(1)
    control.close_gripper(controller, arm="both",wait=False)
    time.sleep(1)
    control.open_gripper(controller, arm="both", wait=False)

    controller.disconnect()

if __name__ == "__main__":
    main()
