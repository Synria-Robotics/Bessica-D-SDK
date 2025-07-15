from bessica_d_sdk import ArmController
from bessica_d_sdk.utils import control

def main():
    controller = ArmController()
    controller.connect()

    print("=== 单臂夹爪控制 ===")
    control.open_gripper(controller, arm='right_arm')
    control.set_gripper_angle(controller, angle_deg=50, arm='right_arm')
    control.close_gripper(controller, arm="right_arm")
    
    

    print("=== 双臂夹爪控制 ===")
    control.open_gripper(controller, arm="both")
    control.set_dual_gripper(controller, left_deg=50, right_deg=30)
    control.close_gripper(controller, arm="both")

    controller.disconnect()

if __name__ == "__main__":
    main()
