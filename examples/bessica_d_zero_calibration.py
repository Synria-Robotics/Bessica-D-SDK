"""

for Bessica-D arm zero calibration example
left_arm: python bessica_d_zero_calibration.py --follower True

right_arm: python bessica_d_zero_calibration.py --follower True

both: python bessica_d_zero_calibration.py --follower True

"""

import os
import sys
import argparse
import time 
# 添加项目根目录到Python路径
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from bessica_d_sdk.controller import ArmController



def main():
    """主函数"""
    print("=== Bessica-D调零示例 ===")
    
    parser = argparse.ArgumentParser(description="调零示例")
    parser.add_argument('--arm', type=str, default='right_arm',
                       help='选择需要调零的机械臂 (left_arm, right_arm, 或both 默认: left_arm)')
    # 创建控制器实例 (可选参数: port="/dev/ttyUSB0", debug_mode=True)
    args = parser.parse_args()

    arm_type = args.arm
    print(f"=== {arm_type}臂调零示例 ===")
    controller = ArmController(debug_mode=False)
    try:
        # 连接到机械臂
        if not controller.connect():
            print("无法连接到机械臂，请检查连接")
            return
            
        print("连接成功，开始读取数据...")
        print("按 Ctrl+C 退出")
        print("-" * 50)
        if arm_type == "right_arm":
            print("操作臂调零前需进入无力矩状态导致无法维持当前状态，请确保安全")
            print(f"当前调零{arm_type}")
            input("按 Enter 键开始...")
            controller.disable_torque(arm_type)
            try:
                input("准备好后请按 Enter 键开始操作臂调零...")
                print("正在执行调零校准...")
                time.sleep(0.5)
                controller.set_zero_position(arm_type)
                time.sleep(0.5)
            except KeyboardInterrupt:
                print("\n调零操作已取消")
                return
        
       
        controller.enable_torque(arm_type)
        time.sleep(1)
    except KeyboardInterrupt:
        print("\n\n程序已停止")
    finally:
        # 断开连接
        controller.disconnect()
        print("已断开连接")

if __name__ == "__main__":
    main()