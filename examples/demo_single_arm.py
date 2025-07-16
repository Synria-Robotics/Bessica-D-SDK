from bessica_d_sdk import ArmController
from bessica_d_sdk.utils import control 
import time
def main():
    controller = ArmController(debug_mode=False)
    if not controller.connect():
        print("未能连接上机械臂")
        return False
    
    print("=== 左臂控制示例 ===")
    # control.move_to_zero(controller, arm="left_arm")
    # time.sleep(1)

    control.move_joint(controller, joint_id=3, angle_deg=20, arm="left_arm",interpolate=False)
    time.sleep(2)
    # control.move_all_joints(controller,[10,20,20,0,15,0,0], arm="left_arm")
    control.print_joint_angles(controller, arm="left_arm")

    controller.disconnect()

if __name__ == "__main__":
    main()
