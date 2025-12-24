"""
Demo: MuJoCo → Real Robot Bridge (将 MuJoCo 拖动同步到真实机器人)

Features:
- 交互拖动双臂（MuJoCo）
- 按设定频率同步关节角到真实机器人
- 使用 Bessica-D SDK 的 SynriaBessicaRobotAPI 下发指令

Usage examples:
python examples/11_demo_mujoco_real_robot_bridge.py --port COM1
"""

import time
from typing import Optional

import numpy as np
from bessica_d_sdk import create_mujoco_controller
import bessica_d_sdk
from bessica_d_sdk.utils.logger import logger


class RealRobotBridge:
    """MuJoCo 到真实机器人的桥接类
    
    在 InteractiveDualArmIK 基础上添加真实机器人同步功能。
    参考 RoboCore_full/examples/bridge/demo_mujoco_real_robot_bridge.py 的实现方式。
    """

    def __init__(
        self,
        robot: bessica_d_sdk.SynriaBessicaRobotAPI,
        controller,
        send_interval: float = 1.5,  # 发送间隔（秒）
    ):
        """
        :param robot: 真实机器人 API 实例
        :param controller: MuJoCo 交互控制器 (InteractiveDualArmIK)
        :param send_interval: 发送到真实机器人的间隔（秒），默认 2.0 秒
        """
        self.robot = robot
        self.controller = controller
        self.send_interval = send_interval
        
        # 跟踪上次发送时间
        self.last_send_time = 0.2
        
        logger.info("[OK] 真实机器人桥接器已初始化")

    def _send_to_real_robot_if_needed(self) -> None:
        """如果需要，发送关节角度到真实机器人（参考 RoboCore 实现）"""
        if self.robot is None:
            return
        
        # 控制发送频率
        current_time = time.time()
        if current_time - self.last_send_time < self.send_interval:
            return

        # 从 MuJoCo 控制器读取当前关节角（弧度）
        q_left_rad = np.array(self.controller.q_left)
        q_right_rad = np.array(self.controller.q_right)
        
        # 验证关节数量
        if q_left_rad.size != 7 or q_right_rad.size != 7:
            logger.debug("关节角度未就绪，跳过一次发送")
            return
        
        # 转换为度（Bessica SDK 使用度制）
        q_left_deg = np.degrees(q_right_rad).tolist()
        q_right_deg = np.degrees(q_left_rad).tolist()

        # 下发到真实机器人（双臂）
        try:
            ok = self.robot.set_joint_target(
                target_joints=[q_left_deg, q_right_deg], 
                arm="both",
                joint_format="deg"
            )
            if not ok:
                logger.debug("下发关节角失败（本次跳过）")
            else:
                logger.debug(f"[OK] 已发送关节角度到真实机器人 (间隔: {self.send_interval}s)")
        except Exception as e:
            logger.debug(f"发送关节角异常（本次跳过）：{e}")

        self.last_send_time = current_time

    def run(self, mode: str = "independent") -> None:
        """运行交互式控制，带真实机器人同步
        
        :param mode: 控制模式 - 'independent', 'relative', 或 'mirror'
        """
        # 设置机器人速度
        self.robot.set_speed(speed_deg_s=15.0)
        
        # 修改 controller 的 step 方法以包含真实机器人同步（参考 RoboCore 实现）
        original_step = self.controller.step
        
        def step_with_bridge():
            original_step()
            self._send_to_real_robot_if_needed()
        
        self.controller.step = step_with_bridge
        
        # 运行交互式控制
        try:
            logger.info(f"启动 MuJoCo 交互控制（模式: {mode}，发送间隔: {self.send_interval}s）…")
            self.controller.run(mode=mode)
        finally:
            logger.info("MuJoCo 交互控制已结束")


def main():
    """主函数（参考 RoboCore 实现结构）"""
    import argparse

    parser = argparse.ArgumentParser(description="MuJoCo → Real Robot Bridge Demo")
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--robot_version', type=str, default="v1_1", help="机械臂版本 (默认: v1_0)")
    parser.add_argument('--mode', type=str, default='independent', choices=['independent','relative','mirror'], help="交互控制模式")
    parser.add_argument('--send_interval', type=float, default=2.0, help="发送间隔（秒），默认 2.0")
    args = parser.parse_args()

    print("="*60)
    print("  MuJoCo 拖动 → 真实机器人实时同步")
    print("="*60)
    
    # 1) 创建 MuJoCo 交互控制器（使用 SDK 提供的工厂函数）
    logger.info("创建 MuJoCo 交互控制器…")
    controller = create_mujoco_controller(robot_version=args.robot_version)

    # 2) 创建并连接真实机器人（必需）
    logger.info("初始化真实机器人接口…")
    robot = bessica_d_sdk.create_robot(
        port=args.port,
        robot_version=args.robot_version,
        debug_mode=False
    )
    if not robot.connect():
        logger.error("[ERROR] 连接真实机器人失败，退出")
        return
    logger.info("[OK] 真实机器人连接成功")

    # 3) 创建桥接并运行
    try:
        bridge = RealRobotBridge(
            robot=robot,
            controller=controller,
            send_interval=args.send_interval,
        )
        
        print(f"\n{'='*60}")
        print(f"  控制模式: {args.mode}")
        print(f"  发送间隔: {args.send_interval} 秒")
        print(f"  真实机器人同步: [启用]")
        print(f"{'='*60}\n")
        
        bridge.run(mode=args.mode)
    finally:
        # 清理
        if robot is not None:
            robot.disconnect()
            logger.info("[断开] 真实机器人已断开连接")


if __name__ == '__main__':
    main()
