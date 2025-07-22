from bessica_d_sdk import ArmController
import time
from bessica_d_sdk.utils import control

def main():
    controller = ArmController()
    last_time = 0
    try:
        if not controller.connect():
            print("未连接")
            return False
        # time.sleep(0.05)
        control.wait_for_valid_state(controller, arm='left_arm')
        print(controller.get_update_thread_status())
        control
        while True:
            # joint_state = controller.read_joint_state() 
            now = time.time()
            # if now - last_time > 1.0:
            # print("=== 读取双臂状态 ===")
            # print("左臂角度:", [round(a, 4) for a in joint_state['left_arm'].angles])
            # print("左臂夹爪:", round(joint_state['left_arm'].gripper, 4))
            # print("右臂角度:", [round(a, 4) for a in joint_state['right_arm'].angles])
            # print("右臂夹爪:", round(joint_state['right_arm'].gripper, 4))

            print("\n=== 单臂读取示例：仅读取右臂 ===")
            # right_angle = controller.read_joint_angles("right_arm") 读取单臂角度
            right_state = controller.read_joint_state("right_arm")
            print("右臂角度:", [round(a, 4) for a in right_state.angles])
            print("右臂夹爪:", round(right_state.gripper, 4))
            last_time = now
    



    except KeyboardInterrupt:
        print("\n\n程序已停止")
    finally:
        # 断开连接
        controller.disconnect()
        print("已断开连接")

if __name__ == "__main__":
    main()