import time
from bessica_d_sdk.controller import ArmController
from bessica_d_sdk.utils import print_joint_angles

def main():
    print("=== Bessica-D 关节状态读取 Demo ===")

    controller = ArmController(debug_mode=False)
    if not controller.connect():
        print("连接失败")
        return

    arm = "right_arm"  # 可改为 "left_arm" 或 "both"

    try:
        print("读取当前关节状态：")
        print_joint_angles(controller, arm)
        state = controller.read_joint_state(arm)
        print(state.gripper)
        # 如需持续读取状态（可选）
        print("持续读取状态（按 Ctrl+C 停止）")
        while True:
            print_joint_angles(controller, arm)
            state = controller.read_joint_state(arm)
            print(state.gripper*controller.RAD_TO_DEG) 
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("读取中断。")

    finally:
        controller.disconnect()
        print("已断开连接")

if __name__ == "__main__":
    main()
