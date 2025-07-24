from bessica_d_sdk.controller import ArmController
import time

def main():
    controller = ArmController()
    if not controller.connect():
        print("无法连接到机械臂")
        return

    arm = input("选择需要调零的臂（left_arm / right_arm / both）：").strip()
    print("将关闭扭矩，请手扶机械臂以防下垂")
    input("按 Enter 继续...")
    controller.disable_torque(arm)
    input("按 Enter 确认零点位置...")

    result = controller.set_zero_position(arm)
    print(f"归零结果: {result}")
    controller.enable_torque(arm)
    controller.disconnect()

if __name__ == "__main__":
    main()
