from bessica_d_sdk import ArmController
from bessica_d_sdk.utils import control
import time

def main():
    controller = ArmController()
    controller.connect()

    print("=== 双臂控制示例 ===")
    # control.open_gripper(controller, arm="both")
    

    # print("打开夹爪：左 80°, 右 60°")
    # control.set_dual_gripper(controller, left_deg=80, right_deg=60)
    control.move_to_zero(controller, arm="both")
    print("移动关节1到 20° (左) 和 10° (右)...")
    control.move_dual_joint(controller, angles_left_deg=[-20, 0, 40, 0, 0, 0, 0],
                                 angles_right_deg=[20, 0, 40, 0, 0, 0, 0])
    time.sleep(1)
    # control.move_dual_joint(controller, angles_left_deg=[20, 0, -40, 0, 0, 0, 0],
    #                              angles_right_deg=[-20, 0, -40, 0, 0, 0, 0])
    control.move_to_zero(controller, arm="both")

    control.print_joint_angles(controller, arm="both")

    print("关闭夹爪...")
    # control.close_gripper(controller, arm="both")

    controller.disconnect()

if __name__ == "__main__":
    main()
