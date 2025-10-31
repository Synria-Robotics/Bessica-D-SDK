import time
from bessica_d_sdk.controller import ArmController
from bessica_d_sdk.hardware import ServoDriver
from bessica_d_sdk.utils import open_gripper, close_gripper, set_gripper_angle, set_dual_gripper, print_gripper_angles

def main():
    print("=== Bessica-D 夹爪控制 Demo ===")

    controller = ServoDriver(debug_mode=False)
    if not controller.connect():
        print("连接失败")
        return

    try:
        print("当前夹爪状态")
        print_gripper_angles(controller, arm="both")

        # 1. 打开夹爪
        print("打开左臂夹爪")
        open_gripper(controller, arm="left_arm")
        print_gripper_angles(controller, arm="left_arm")

        print("打开右臂夹爪")
        open_gripper(controller, arm="right_arm")
        print_gripper_angles(controller, arm="right_arm")

        # 2. 关闭夹爪
        print("关闭左臂夹爪")
        close_gripper(controller, arm="left_arm")
        print_gripper_angles(controller, arm="left_arm")

        print("关闭右臂夹爪")
        close_gripper(controller, arm="right_arm")
        print_gripper_angles(controller, arm="right_arm")

        # 3. 设置夹爪角度（单臂）
        print("设置右臂夹爪角度为 60")
        set_gripper_angle(controller, 100, arm="right_arm")
        print_gripper_angles(controller, arm="right_arm")

        # 4. 设置双臂夹爪角度
        print("设置双臂夹爪角度，左=30，右=70")
        set_dual_gripper(controller, left_deg=30, right_deg=30)
        print_gripper_angles(controller, arm="both")

         # 5. 交互式控制
        print("\n=== 进入交互式控制模式 ===")
        print("输入夹爪角度(0-100度)，或输入'q'退出")
        
        while True:
            user_input = input("\n请输入角度值: ")
            
            if user_input.lower() == 'q':
                break
            
            try:
                angle_deg = float(user_input)
                if angle_deg < 0 or angle_deg > 100:
                    print("角度超出范围，有效范围: 0-100度")
                    continue
                    
                print(f"设置夹爪角度: {angle_deg}度")
                angle_rad = angle_deg * controller.DEG_TO_RAD
                controller.set_gripper(angle_rad, arm= 'left_arm', wait_for_completion=True)
                
                print_gripper_angles(controller, arm="left_arm")

            except ValueError:
                print("无效输入，请输入数字或'q'")

    finally:
        controller.disconnect()
        print("已断开连接")

if __name__ == "__main__":
    main()
