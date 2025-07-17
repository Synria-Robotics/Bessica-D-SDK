from bessica_d_sdk import ArmController
from bessica_d_sdk.utils import control
from serial_diagnostic import SerialDiagnostic  # 你可直接 import

import time
import threading

def run_diagnostic_in_background():
    diag = SerialDiagnostic(
        port="/dev/ttyUSB0",
        baudrate=921600,
        save_path="controller_diag_log.json",
        read_interval=0.01
    )
    diag.start()
    return diag

def main():
    diag = run_diagnostic_in_background()
    controller = ArmController(debug_mode=False)

    if not controller.connect():
        print("机械臂未连接")
        return

    print("开始插值运动测试")
    control.move_joint(controller, joint_id=3, angle_deg=20, arm="left_arm", interpolate=True)
    time.sleep(2)
    control.move_joint(controller, joint_id=3, angle_deg=0, arm="left_arm", interpolate=True)
    time.sleep(2)

    print("当前状态：", controller.read_joint_state("left_arm").angles)

    diag.save_log()
    diag.stop()
    controller.disconnect()

if __name__ == "__main__":
    main()
