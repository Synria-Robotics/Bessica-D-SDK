import time
from bessica_d_sdk.controller import ArmController
from bessica_d_sdk.utils import move_joints, move_joint, move_to_zero

def ensure_left_right_mapping(controller: ArmController, assume_arm: str):
    """自动检测固件返回的双臂顺序是否与期望一致。
    过程:
      1. 读取一次双臂角度 baseline
      2. 提示用户轻微晃动 目标 arm (例如 left_arm) 的一个大关节
      3. 再读取一次
      4. 计算 baseline 与 新读数 每个臂变化量总和 (L1)
      5. 若另一只手臂变化更大, 说明 block_order 需要翻转
    仅在 assume_arm == 'left_arm' 且 block_order 当前为 ("right_arm","left_arm") 时尝试
    """
    try:
        if assume_arm not in ("left_arm","right_arm"):
            return
        # 只在需要 left_arm 时检测, 避免不必要的提示
        baseline = controller.read_joint_angles('both')
        if not baseline or len(baseline) != 2:
            return
        print("请轻微晃动一下你要操作的那只手臂(例如左臂)的某个关节, 1 秒后自动检测...")
        time.sleep(1.2)
        after = controller.read_joint_angles('both')
        if not after or len(after) != 2:
            return
        delta0 = sum(abs(a-b) for a,b in zip(baseline[0], after[0]))
        delta1 = sum(abs(a-b) for a,b in zip(baseline[1], after[1]))
        # baseline[0] 按约定是 left_arm (我们在 read_joint_angles 中固定返回 [left,right])
        # 如果用户实际晃动了 left_arm, 期望 delta0 > delta1
        if assume_arm == 'left_arm' and delta1 > delta0 * 1.5:  # 差距阈值
            print("检测到左右臂顺序可能反了, 自动切换解析顺序 -> (left_arm,right_arm)")
            controller.set_block_order(("left_arm","right_arm"))
            time.sleep(0.2)
        elif assume_arm == 'right_arm' and delta0 > delta1 * 1.5:
            print("检测到左右臂顺序可能反了, 自动切换解析顺序 -> (right_arm,left_arm)")
            controller.set_block_order(("right_arm","left_arm"))
    except Exception as e:
        print(f"自动映射检测失败: {e}")

def main():
    print("=== Bessica-D 单臂关节控制 Demo ===")

    controller = ArmController(debug_mode=False)
    if not controller.connect():
        print("连接失败")
        return

    arm = "left_arm"  # 目标单臂 (left_arm/right_arm)

    # 自动检测左右映射 (仅在使用 left_arm 时做一次)
    ensure_left_right_mapping(controller, arm)

    try:
        move_to_zero(controller,arm)

        # 1. 设置全部7个关节角度
        target_angles = [30, 0, 30, 0, 45, -15, 5]
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
