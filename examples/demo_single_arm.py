import time
from bessica_d_sdk.controller import ArmController
from bessica_d_sdk.utils import move_joints, move_joint, move_to_zero

def main():
    print("=== Bessica-D 单臂关节控制 Demo ===")

    controller = ArmController(debug_mode=False)
    if not controller.connect():
        print("连接失败")
        return

    arm = "left_arm"  # 目标单臂 (left_arm/right_arm)
    controller.set_block_order(("left_arm", "right_arm"))
    try:
        move_to_zero(controller,arm)

        # 1. 设置全部7个关节角度
        target_angles = [0, 30, 30, 0, 45, -15, 5]
        print(f"设置 {arm} 所有关节角度为: {target_angles}")
        move_joints(controller, target_angles, arm)
        time.sleep(2)

        # 2. 设置单个关节角度（关节 2）
        joint_id = 2
        target_angle = 80
        print(f"设置 {arm} 第 {joint_id} 个关节为 {target_angle} 度")
        move_joint(controller, joint_id, target_angle, arm)
        time.sleep(2)

        move_to_zero(controller,arm)

    finally:
        controller.disconnect()
        print("已断开连接")

if __name__ == "__main__":
    main()
