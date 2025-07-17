from bessica_d_sdk import ArmController
from bessica_d_sdk.utils import control
import time

def main():
    controller = ArmController()
    try:
        if not controller.connect():
            print("未连接")
            return False
        
        print("=== 双臂控制示例 ===")
        # control.move_to_zero(controller, arm="both")

    # print("打开夹爪：左 80°, 右 60°")
    # control.set_dual_gripper(controller, left_deg=80, right_deg=60)
        control.print_joint_angles(controller, arm="both")
        i = 0
        while i <10:
            print(f"第{i}次移动关节1到 20° (左) 和 20° (右)...")
            control.print_joint_angles(controller, arm="both")
            control.move_dual_joint(controller, angles_left_deg=[20, 0, 10, 0, 0, 0, 0],
                                        angles_right_deg=[20, 0, 10, 0, 0, 0, 0])
            time.sleep(0.5)
            control.move_dual_joint(controller, angles_left_deg=[-20, 0, 10, 0, 0, 0, 0],
                                        angles_right_deg=[-20, 0, 10, 0, 0, 0, 0])
            time.sleep(0.5)
            i += 1

        control.print_joint_angles(controller, arm="both")

        # print("关闭夹爪...")
        # control.close_gripper(controller, arm="both")

    except KeyboardInterrupt:
        print("\n\n程序已停止")

    finally:
        # 断开连接
        controller.disconnect()
        print("已断开连接")

if __name__ == "__main__":
    main()
