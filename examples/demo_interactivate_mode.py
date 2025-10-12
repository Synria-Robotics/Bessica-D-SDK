import time
from bessica_d_sdk.controller import ArmController
from bessica_d_sdk.utils import (
    move_to_zero,
    move_joint,
    move_joint_dual_arm,
    move_joints_dual_arm,
    move_joints,
    open_gripper,
    close_gripper,
    print_joint_angles,
    set_gripper_angle,
    set_dual_gripper,
    print_gripper_angles
)

import sys
import select

def select_arm():
    print("\n请选择控制的机械臂：")
    print("1. 左臂 ")
    print("2. 右臂 ")
    print("3. 双臂 (both)")
    choice = input("请输入编号 (1/2/3): ").strip()
    if choice == '1':
        return 'left_arm'
    elif choice == '2':
        return 'right_arm'
    elif choice == '3':
        return 'both'
    else:
        print("无效输入，默认为 right_arm")
        return 'right_arm'

def teaching_mode(controller: ArmController):
    # 如果发现“选择左臂却记录的是右臂”可在运行时调用：
    # controller.set_block_order(("left_arm","right_arm"))
    # 这会告诉解析器第一块14字节属于左臂。
    arm = select_arm()
    loop_mode = input("是否让机械臂在两个点间往复运动？(y/n): ").strip().lower() == 'y'

    print("记录当前关节角度... (若左右颠倒, 先 Ctrl+C 退出后调用 set_block_order 调整)")
    controller.set_block_order(("left_arm","right_arm"))
    if arm != "both":
        start_pose = [a * controller.RAD_TO_DEG for a in controller.read_joint_angles(arm)]
    else:
        raw_angles = controller.read_joint_angles(arm)  # [left, right]
        start_pose = [
            [a * controller.RAD_TO_DEG for a in raw_angles[0]],
            [a * controller.RAD_TO_DEG for a in raw_angles[1]]
        ]

    input("按enter关闭扭矩，请手动拖拽到目标位置...")
    controller.disable_torque(arm)
    input("拖拽完成后按 Enter 继续...")

    print("重新打开扭矩...")
    controller.enable_torque(arm)
    time.sleep(1.0)

    print("记录拖拽后的目标关节角度...")

    if arm != "both":
        goal_pose = [a * controller.RAD_TO_DEG for a in controller.read_joint_angles(arm)]
    else:
        raw_angles = controller.read_joint_angles(arm)  # [left, right]
        goal_pose = [
            [a * controller.RAD_TO_DEG for a in raw_angles[0]],
            [a * controller.RAD_TO_DEG for a in raw_angles[1]]
        ]

    input("按回车执行轨迹")

    def move(a, b):
        if arm == "both":
            move_joints_dual_arm(controller, a[0], a[1])
            time.sleep(1)
            move_joints_dual_arm(controller, b[0], b[1])
        else:
            move_joints(controller, a, arm)
            time.sleep(1)
            move_joints(controller, b, arm)

    if loop_mode:
        print("开始往复运动（Ctrl+C 停止）...")
        try:
            i = 1
            while True:
                print(f"第{i}次")
                i += 1
                move(start_pose, goal_pose)
                time.sleep(1)
    
        except KeyboardInterrupt:
            print("用户中断，结束拖拽示教模式。")
    else:
        print("执行单次来回移动...")
        move(start_pose, goal_pose)


def main():
    controller = ArmController(debug_mode=False)
    if not controller.connect():
        print("无法连接到机械臂")
        return
    
    while True:
        print("\n========= Bessica-D 终极测试菜单 =========")
        print("1. 控制夹爪（开/关）")
        print("2. 设置全关节角度")
        print("3. 设置单个关节角度")
        print("4. 读取当前关节角度")
        print("5. 机械臂归零")
        print("6. 扭矩开关控制")
        print("7. 回到零点")
        print("8. 拖拽示教模式")
        print("9. 云台控制 (X/Y)")
        print("10. 退出")
        choice = input("请输入操作编号：").strip()

        if choice == '1':
            arm = select_arm()
            
            if arm == 'both':
                # 双臂模式下分别输入左右角度
                try:
                    left = float(input("左臂角度(0~100): ")) 
                    right = float(input("右臂角度(0~100): ")) 
                    set_dual_gripper(controller, left, right)
                except ValueError:
                    print("非法输入：请输入数值角度")
            
            else:
                # 单臂模式
                mode = input("输入 'open' 打开夹爪，'close' 关闭夹爪，或输入角度 (0~100): ").strip()
                try:
                    angle = float(mode)
                    set_gripper_angle(controller, angle, arm)
                except ValueError:
                    if mode == 'open':
                        open_gripper(controller, arm=arm)
                    elif mode == 'close':
                        close_gripper(controller, arm=arm)
                    else:
                        print("非法输入")

            print_gripper_angles(controller, arm)


        elif choice == '2':
            arm = select_arm()
            if arm == 'both':
                left = input("请输入左臂7个关节角度 (单位:度，用逗号分隔): ").strip()
                right = input("请输入右臂7个关节角度 (单位:度，用逗号分隔): ").strip()
                try:
                    left_angles = [float(a) for a in left.split(',')]
                    right_angles = [float(a) for a in right.split(',')]
                    move_joints_dual_arm(controller, left_angles, right_angles)
                except:
                    print("输入格式错误")
            else:
                angles = input("请输入7个关节角度 (单位:度，用逗号分隔): ").strip()
                try:
                    angles = [float(a) for a in angles.split(',')]
                    move_joints(controller, angles, arm)
                except:
                    print("输入格式错误")

        elif choice == '3':
            arm = select_arm()
            try:
                if arm == 'both':
                    left_idx = int(input("请输入左臂关节编号 (0~6): "))
                    right_idx = int(input("请输入右臂关节编号 (0~6): "))
                    left_angle = float(input("请输入左臂目标角度 (单位:度): "))
                    right_angles = float(input("请输入右臂目标角度 (单位:度): "))
                    move_joint_dual_arm(controller,left_idx, right_idx, left_angle,right_angles)
                else:
                    idx = int(input("请输入关节编号 (0~6): "))
                    angle = float(input("请输入目标角度 (单位:度): "))
                    move_joint(controller, idx, angle, arm)
            except:
                print("输入格式错误")

        elif choice == '4':
            arm = select_arm()
            mode = input("是否循环读取关节角度？(y/n): ").strip().lower()
            read_gripper = input("是否读取夹爪角度？（y/n）: ").strip().lower()
            if read_gripper == 'y':
                read_gripper = True
            else:
                read_gripper = False
            if mode == 'y':
                print("按 Enter 停止循环读取...")
                try:
                    while True:
                        print_joint_angles(controller, arm, read_gripper)
                        time.sleep(0.5)
                        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                            input()  # 停止读取
                            print("已停止循环读取。")
                            break
                except KeyboardInterrupt:
                    print("用户中断，已退出读取。")
            else:
                print_joint_angles(controller, arm, read_gripper)

        elif choice == '5':
            arm = select_arm()
            print("扭矩将临时关闭，请确保安全。")
            input("准备好后按 Enter 继续...")
            controller.disable_torque(arm)
            input("按回车确认零点位置，请继续用手扶着机械臂")
            result = controller.set_zero_position(arm)
            time.sleep(1)
            print(f"归零结果为{result}")
            

        elif choice == '6':
            arm = select_arm()
            action = input("输入 'on' 开启扭矩，'off' 关闭扭矩: ").strip()
            if action == 'on':
                controller.enable_torque(arm)
            elif action == 'off':
                controller.disable_torque(arm)
            else:
                print("非法输入")

        elif choice == '7':
            arm = select_arm()
            move_to_zero(controller, arm)

        elif choice == '8':
            teaching_mode(controller)

        elif choice == '9':
            try:
                mode = input("选择模式: 1=设置角度(度)  2=回中心(0,0): ").strip()
                if mode == '2':
                    ok = controller.set_gimbal_deg(0.0, 0.0)
                    print(f"云台回中心: {'成功' if ok else '失败'}")
                else:
                    x_deg = float(input("输入 X 轴角度(度, -180~180): ").strip())
                    y_deg = float(input("输入 Y 轴角度(度, -180~180): ").strip())
                    ok = controller.set_gimbal_deg(x_deg, y_deg)
                    print(f"设置云台 X={x_deg}°, Y={y_deg}°: {'成功' if ok else '失败'}")
            except ValueError:
                print("非法输入：请输入数字")

        elif choice == '10':
            print("退出程序")
            break

        else:
            print("无效输入，请重新选择。")

    controller.disconnect()

if __name__ == "__main__":
    main()