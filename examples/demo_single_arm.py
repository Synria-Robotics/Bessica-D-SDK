from bessica_d_sdk import ArmController
from bessica_d_sdk.utils import control
import time

from bessica_d_sdk.utils import control
import time

def main():
    controller = ArmController(debug_mode=False)
    try:
        if not controller.connect():
            print("机械臂没有连接")
            return
    controller = ArmController(debug_mode=False)
    try:
        if not controller.connect():
            print("机械臂没有连接")
            return

        print("=== 左臂控制示例 ===")
        control.print_joint_angles(controller, arm="left_arm")
        # control.wait_for_valid_state(controller, arm='left_arm')
        time.sleep(2)
        # control.move_to_zero(controller, arm="left_arm")
        control.move_all_joints(controller, angles_deg=[0,0,0,0,0,0,0], arm="left_arm", interpolate=True)
        
        i = 0
        max_steps = 10  # 限制循环次数，便于测试
        # while i < max_steps:
        #     print(f"\n第 {i} 次运行")
        #     control.print_joint_angles(controller, arm="right_arm")
        #     control.move_to_zero(controller, arm="right_arm")
        #     # control.move_all_joints(controller, angles_deg=[0,0,0,0,0,0,0], arm="left_arm", interpolate=True)
        #     # move 到目标角度
        #     control.move_all_joints(controller, angles_deg=[10,10,10,10,0,0,20], arm="right_arm", interpolate=True)
    
        #     control.print_joint_angles(controller, arm="right_arm")

         
        #     i += 1

        print("\n控制结束，断开连接")
    except KeyboardInterrupt:
        print("\n\n程序已停止")
    finally:
        controller.disconnect()
        print("已断开连接")

if __name__ == "__main__":
    main()
