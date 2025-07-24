import time
from bessica_d_sdk.controller import ArmController
from bessica_d_sdk.utils import move_joints_dual_arm, move_joint_dual_arm

def main():
    print("=== Bessica-D 双臂控制 Demo ===")

    controller = ArmController(debug_mode=False)
    if not controller.connect():
        print("连接失败")
        return

    try:
        # 启用扭矩
        controller.enable_torque("both")
        time.sleep(1)

        print("1. 设置双臂所有关节角度")
        left_angles = [10, 20, 20, 10, 45, 20, 0]
        right_angles = [10, 20, 20, 10, 45, 20, 0]
        print(f"左臂目标角度: {left_angles}")
        print(f"右臂目标角度: {right_angles}")
        move_joints_dual_arm(controller, left_angles, right_angles)
        time.sleep(2)

        print("2. 设置双臂某个关节的角度")
        left_joint_id = 0
        right_joint_id = 0
        left_target_angle = 20
        right_target_angle = -20
        print(f"设置左臂关节 {left_joint_id} 到 {left_target_angle} 度")
        print(f"设置右臂关节 {right_joint_id} 到 {right_target_angle} 度")
        move_joint_dual_arm(controller, left_joint_id, right_joint_id, left_target_angle, right_target_angle)
        time.sleep(2)

    finally:
        controller.disconnect()
        print("已断开连接")

if __name__ == "__main__":
    main()
