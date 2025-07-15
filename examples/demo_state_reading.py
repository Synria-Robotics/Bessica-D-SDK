from bessica_d_sdk import ArmController

def main():
    controller = ArmController()
    controller.connect()

    print("=== 读取双臂状态 ===")

    # joint_angles = controller.read_joint_angles()  读取双臂的关节角度

    joint_states = controller.read_joint_state()
    print("左臂角度:", [round(a, 4) for a in joint_states["left_arm"].angles])
    print("右臂角度:", [round(a, 4) for a in joint_states["right_arm"].angles])
    print("左臂夹爪:", round(joint_states["left_arm"].gripper, 4))
    print("右臂夹爪:", round(joint_states["right_arm"].gripper, 4))

    print("\n=== 单臂读取示例：仅读取右臂 ===")

    # right_angle = controller.read_joint_angles("right_arm") 读取单臂角度

    right_state = controller.read_joint_state("right_arm")
    print("右臂角度:", [round(a, 4) for a in right_state.angles])
    print("右臂夹爪:", round(right_state.gripper, 4))

    controller.disconnect()

if __name__ == "__main__":
    main()