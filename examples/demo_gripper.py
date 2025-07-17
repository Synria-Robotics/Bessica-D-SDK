from bessica_d_sdk import ArmController
from bessica_d_sdk.utils import control
import time
def main():
    controller = ArmController(debug_mode=False)
    try:
        if not controller.connect():
            print("机械臂没有连接")
            return

        print("=== 单臂夹爪控制 ===")
        i = 0
        # for i in range(10):
        #     control.open_gripper(controller, arm='right_arm')
        #     time.sleep(1)
        #     control.set_gripper_angle(controller, angle_deg=50, arm='right_arm')
        #     time.sleep(1)
        #     control.close_gripper(controller, arm="right_arm")
        #     time.sleep(1)
        #     i +=1
            
        print("=== 双臂夹爪控制 ===")
        for i in range(10):
            control.open_gripper(controller, arm="both")
            time.sleep(1)
            control.set_dual_gripper(controller, left_deg=50, right_deg=30)
            time.sleep(1)
            control.close_gripper(controller, arm="both")
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n\n程序已停止")
    finally:
        controller.disconnect()
        print("已断开连接")

if __name__ == "__main__":
    main()
