from re import T
import time

from robocore.modeling import robot_model
from bessica_d_sdk.hardware import ServoDriver
from bessica_d_sdk.utils import move_joints_dual_arm, move_joints,close_gripper,set_dual_gripper
from bessica_d_sdk.api import SynriaBessicaRobotAPI
import logging
logger = logging.getLogger("demo_dual_arm")
def main():
    print("=== Bessica-D 双臂控制 Demo ===")

    controller = ServoDriver(debug_mode=False)
    robot_api = SynriaBessicaRobotAPI(controller)
    print(robot_api.connect())
    if not controller.connect():
        print("连接失败")
        return

    try:
        #robot_api.get_pose(arm="left_arm")
        robot_api.torque_control(command="on", arm="both")
        logger.info("设置双臂所有关节角度")
        left_angles = [10, 20, 20, 10, 45, 20, 0]
        right_angles = [10, 20, 20, 10, 45, 20, 0]
        home_angles_l = [0.0] * 7
        home_angles_r = [0.0] * 7
        logger.info(f"左臂目标角度: {left_angles}")
        logger.info(f"右臂目标角度: {right_angles}")
        robot_api.set_joint_target(right_angles, arm="right_arm")
        time.sleep(2)
        pose_target = robot_api.get_pose(arm="right_arm")['input_for_ik']
        robot_api.set_home()
        time.sleep(2)
        #logger.info(f"右臂输入 for ik: {pose_target}")
        # left_joint_id = 0
        # right_joint_id = 0
        # left_target_angle = 20
        # right_target_angle = -20
        # print(f"设置左臂关节 {left_joint_id} 到 {left_target_angle} 度")
        # print(f"设置右臂关节 {right_joint_id} 到 {right_target_angle} 度")
        # print(robot_api.robot_model)
        #robot_api.set_pose_target(target_pose=[0.01, 0.02, 0.03, 0.5, 0.4, -0.3, 1.0], arm="left_arm")
        #robot_api.set_joint_target(left_angles, arm="left_arm")
        #time.sleep(2)
        print("设置双臂回到零位")
        # robot_api.set_home()
        print(robot_api.set_pose_target(target_pose=pose_target, arm="right_arm"))
        # robot_api.set_joint_target(right_angles, arm="right_arm")
        # robot_api.get_pose(arm="right_arm")
        # robot_api.set_home( arm="both")
        # robot_api.get_joints(arm="both")

    finally:
        robot_api.disconnect()
        print("已断开连接")

if __name__ == "__main__":
    main()
