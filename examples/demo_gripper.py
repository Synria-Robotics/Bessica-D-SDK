import time
from bessica_d_sdk.controller import ArmController
from bessica_d_sdk.utils import open_gripper, close_gripper, set_gripper_angle, set_dual_gripper

def main():
    print("=== Bessica-D 夹爪控制 Demo ===")

    controller = ArmController(debug_mode=False)
    if not controller.connect():
        print("连接失败")
        return

    try:
        controller.enable_torque("both")
        time.sleep(1)

        # 1. 打开夹爪
        print("打开左臂夹爪")
        open_gripper(controller, arm="left_arm")
        print(f"当前夹爪角度：{controller.read_gripper_data('left_arm')}")
        time.sleep(1)

        print("打开右臂夹爪")
        open_gripper(controller, arm="right_arm")
        print(f"当前夹爪角度：{controller.read_gripper_data('right_arm')}")
        time.sleep(1)

        # 2. 关闭夹爪
        print("关闭左臂夹爪")
        close_gripper(controller, arm="left_arm")
        print(f"当前夹爪角度：{controller.read_gripper_data('left_arm')}")
        time.sleep(1)

        print("关闭右臂夹爪")
        close_gripper(controller, arm="right_arm")
        print(f"当前夹爪角度：{controller.read_gripper_data('right_arm')}")
        time.sleep(1)

        # 3. 设置夹爪角度（单臂）
        print("设置右臂夹爪角度为 60")
        set_gripper_angle(controller, 60, arm="right_arm")
        print(f"当前夹爪角度：{controller.read_gripper_data('right_arm')}")
        time.sleep(1)

        # 4. 设置双臂夹爪角度
        print("设置双臂夹爪角度，左=30，右=70")
        set_dual_gripper(controller, left_deg=30, right_deg=70)
        print(f"当前夹爪角度：{controller.read_gripper_data('both')}")
        time.sleep(1)

    finally:
        controller.disconnect()
        print("已断开连接")

if __name__ == "__main__":
    main()
