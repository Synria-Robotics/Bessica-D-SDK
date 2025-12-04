import math
import time
import logging
import threading
from typing import List, Optional, Union, Tuple, Dict
import numpy as np
import traceback
from ..utils.logger import logger
from .serial_comm import SerialComm
from .data_parser import DataParser, JointState, JointStateDict



class ServoDriver:
    """机械臂控制模块"""
    
    # 常量定义
    RAD_TO_DEG = 180.0 / math.pi  # 弧度转角度系数
    DEG_TO_RAD = math.pi / 180.0  # 角度转弧度系数


    # 帧常量
    FRAME_HEADER = 0xAA
    FRAME_FOOTER = 0xFF
    FRAME_MINIMAL_SIZE = 5
    ARM_DATA_SIZE = 21
    GRIPPER_FRAME_SIZE = 8


    # 指令ID
    CMD_GRIPPER = 0x02     # 夹爪控制与行程反馈
    CMD_ZERO_POS = 0x03    # 机械臂以当前位置为零点  
    CMD_DUAL_ARM = 0x06   # 双臂角度反馈与控制
    CMD_TORQUE = 0x13      # 机械臂力矩控制
    CMD_GIMBAL = 0x14      # 云台角度控制 (X/Y)

    # 识别帧
    PRESENT_POSITION = 0x38 #当前机械臂关节角度识别帧
    # PRESENT_SPEED = 0x41    #当前机械臂关节速度识别 （待开发）

    LEFT_ARM = 0X02
    RIGHT_ARM = 0X01
    BOTH_ARM = 0x03


    def __init__(self, port: str = "", baudrate: int = 1000000, debug_mode: bool = False, poll_mode: bool = True):
        """
        初始化机械臂控制器
        
        Args:
            port: 串口名称，留空则自动搜索
            baudrate: 波特率
            debug_mode: 是否启用调试模式
            poll_mode: True 表示响应式协议，需要主动发送请求帧获取当前关节状态
        """
        self.debug_mode = debug_mode
        self.poll_mode = poll_mode  # 新增: 是否主动轮询请求状态
        self._lock = threading.Lock()

        # 创建串口通信模块和数据解析器
        self.serial_comm = SerialComm(lock=self._lock, port=port, baudrate=baudrate, debug_mode=debug_mode)
        self.data_parser = DataParser(lock=self._lock, debug_mode=debug_mode)
        
        # 舵机数量
        self.servo_count = 9
        self.joint_count = 7
        
        self.count = 0
        self.joint_to_servo_map = [
            (0, 1.0),    # 关节1 -> 舵机1 (正向)
            (1, 1.0),    # 关节2 -> 舵机3 (正向)
            (1, -1.0),   # 关节2 -> 舵机4 (反向)
            (2, 1.0),    # 关节3 -> 舵机5 (正向)
            (3, 1.0),    # 关节4 -> 舵机6 (正向)
            (3, -1.0),   # 关节4 -> 舵机7 (反向)
            (4, 1.0),    # 关节5 -> 舵机8 (正向)
            (5, 1.0),    # 关节6 -> 舵机9 (正向)
            (6, 1.0),    # 关节7 -> 舵机10 (正向)
        ]

        # 方向因子：正方向与右臂一致，若左臂需要反向则为 -1
        self.direction_map = {
            "left_arm":  [1, 1, 1, 1, 1, 1, 1],
            "right_arm": [1, 1, 1, 1, 1, 1, 1]
        }

        # 状态更新线程相关
        self._update_thread = None
        self.read_interval = 0.005
        self._stop_thread = threading.Event()
        self._thread_running = False
        
        logger.info("初始化机械臂控制模块")
        logger.info(f"调试模式: {'启用' if debug_mode else '禁用'}")

        self.disconnect()

    def wait_for_valid_state(self, arm: str = "both", timeout: float = 5.0) -> bool:
        """
        等待指定机械臂的状态变为有效（不为全零）

        Args:
            arm (str): "left_arm"、"right_arm" 或 "both"
            timeout (float): 最大等待时间（秒）

        Returns:
            bool: 如果在超时时间内收到有效状态，返回 True；否则返回 False
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            js = self.read_joint_state(arm)
            if arm == "both":
                left_ok = js["left_arm"] and max(abs(a) for a in js["left_arm"].angles) > 1e-3
                right_ok = js["right_arm"] and max(abs(a) for a in js["right_arm"].angles) > 1e-3
                if left_ok and right_ok:
                    return True
            else:
                if js and max(abs(a) for a in js.angles) > 1e-3:
                    return True
            time.sleep(0.05)
        print(f"[超时] 未收到 {arm} 有效关节状态")
        return False


    def __del__(self):
        """析构函数，确保线程和连接在对象销毁时被正确清理"""
        try:
            # 停止状态更新线程
            self.stop_update_thread()
            # 断开连接
            self.disconnect()
        except Exception as e:
            if hasattr(logger, 'error'):  # 在某些情况下logger可能已被销毁
                logger.error(f"析构函数中出现异常: {str(e)}")
    
    def connect(self) -> bool:
        """
        连接到机械臂
        
        Returns:
            bool: 连接是否成功
        """
        result = self.serial_comm.connect()
        if result:
            # 连接成功后启动状态更新线程
            self.start_update_thread()
            self.wait_for_valid_state(arm='both')
        return result
    
    def disconnect(self):
        """断开与机械臂的连接"""
        # 先停止状态更新线程
        self.stop_update_thread()
        self.serial_comm.disconnect()
    
    def start_update_thread(self):
        """启动状态更新线程"""
        if self._update_thread is not None and self._thread_running:
            logger.info("状态更新线程已经在运行")
            return
        
        # 重置停止信号
        self._stop_thread.clear()
        self._thread_running = True
        
        # 创建并启动线程
        self._update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self._update_thread.start()

        
        logger.info("状态更新线程已启动")
    
    def stop_update_thread(self):
        """停止状态更新线程"""
        if self._update_thread is None or not self._thread_running:
            return
        
        # 设置停止信号
        self._stop_thread.set()
        self._thread_running = False
        
        # 等待线程结束
        if self._update_thread.is_alive():
            self._update_thread.join(timeout=2.0)
        
        self._update_thread = None
        logger.info("状态更新线程已停止")
    
    def is_update_thread_running(self) -> bool:
        """
        检查状态更新线程是否正在运行
        
        Returns:
            bool: 线程是否正在运行
        """
        return self._thread_running and self._update_thread is not None and self._update_thread.is_alive()
    
    def get_update_thread_status(self) -> Dict:
        """
        获取状态更新线程的详细信息
        
        Returns:
            Dict: 包含线程状态的字典
        """
        return {
            "running": self.is_update_thread_running(),
            "enabled": self._thread_running,
            "thread_exists": self._update_thread is not None,
            "thread_alive": self._update_thread.is_alive() if self._update_thread else False,
            "stop_flag_set": self._stop_thread.is_set()
        }
    
    def _send_joint_state_request(self, arm: str = 'both'):
        """发送关节状态请求帧 (响应式协议)
        新协议: 请求帧固定 0xAA 0x06 0x01 0x00 0x00 0xFF
        说明: LEN=0x01, DATA区仅1字节(此处恒 0x00 占位), 校验=0x00
        arm 参数保留仅为兼容旧接口, 不再区分左右/双臂
        """
        frame = [self.FRAME_HEADER, self.CMD_DUAL_ARM, 0x01, 0x00, 0x00, self.FRAME_FOOTER]
        return self.serial_comm.send_data(frame)

    def _update_loop(self):
        """状态更新线程主循环 (支持响应式轮询)"""
        logger.info("状态更新线程开始运行")
        while not self._stop_thread.is_set():
            time.sleep(self.read_interval)
            try:
                if self.poll_mode:
                    # 主动请求关节状态
                    self._send_joint_state_request('both')
                with self._lock:
                    frame = self.serial_comm.read_frame()
                if frame and frame != 9999999:
                    self.data_parser.parse_frame(frame)
            except Exception as e:
                logger.error(f"状态线程异常：{e}")
                break
        self._thread_running = False
        logger.info("状态更新线程结束")

    # def read_joint_angles(self, arm: str = 'both') -> Optional[Union[List[float], List[List[float]]]]:
    #     """
    #     读取机械臂的关节角度（单位：弧度）

    #     Args:
    #         arm (str, optional): 指定读取的机械臂名称，可选值为 "left_arm" 或 "right_arm"。
    #                             如果未指定（默认为 None），则同时返回左右两个机械臂的角度。

    #     Returns:
    #         Optional[Union[List[float], List[List[float]]]]:
    #             - 若指定 arm，则返回一个长度为 7 的一维列表：shape = (7,)
    #             - 若 arm 为 both，则返回一个包含两个一维列表的二维列表：shape = (2, 7)，
    #             结构为 [left_arm_angles, right_arm_angles]
    #             - 若读取失败，则返回 None。
    #     """
    #     joint_states = self.data_parser.get_joint_state()

    #     if arm == 'both':
    #         # 确保返回顺序固定 [left, right]
    #         return [joint_states['left_arm'].angles,
    #                 joint_states['right_arm'].angles]
    #     else:
    #         return joint_states[arm].angles

    #     def set_joint_angles_dual_arm(self, left_angles: List[float], right_angles: List[float]) -> bool:
    #         """
    #         设置双臂关节角度
    #         """
    #         return self.set_joint_angles(left_angles, arm="left_arm") and self.set_joint_angles(right_angles, arm="right_arm")
    #     def set_joint_angles_single_arm(self, angles: List[float], arm: str) -> bool:
    #         """
    #         设置单臂关节角度
    #         """
    #         return self.set_joint_angles(angles, arm=arm)


    def set_block_order(self, order: Tuple[str, str]):
        """设置底层解析器双臂数据块顺序。
        默认固件顺序可能为 ("right_arm","left_arm")，如果拖拽示教出现左右互换，可调用：
            controller.set_block_order(("left_arm","right_arm"))
        """
        if not isinstance(order, tuple) or len(order) != 2:
            logger.error(f"block_order 必须是长度为2的元组, 当前: {order}")
            return False
        if set(order) != {"left_arm","right_arm"}:
            logger.error(f"block_order 只允许包含 left_arm / right_arm, 当前: {order}")
            return False
        try:
            self.data_parser.block_order = order
            logger.info(f"已更新解析 block_order = {order}")
            return True
        except Exception as e:
            logger.error(f"更新 block_order 失败: {e}")
            return False


    def read_gripper_data(self, arm: str = 'both') -> Union[float, Tuple[float, float]]:
        """
        读取机械臂夹爪的当前角度（单位：弧度）

        Args:
            arm (str, optional): 指定要读取的机械臂，可选值为 "left_arm" 或 "right_arm"。
                                若为 both，则同时返回左右两个机械臂的夹爪角度。

        Returns:
            Union[float, Tuple[float, float]]:
                - 若指定 arm，则返回该机械臂的夹爪角度（float）。
                - 若未指定 arm，则返回一个元组 (left_gripper, right_gripper)。
        """
        joint_states = self.data_parser.get_joint_state(arm)

        if arm == 'both':
            return (joint_states['left_arm'].gripper,
                    joint_states['right_arm'].gripper)
            
        else:
            return joint_states.gripper

    
    
    def read_joint_state(self, arm: str = 'both') -> Optional[Union[JointState, JointStateDict]]:
        """
        读取机械臂的所有信息。

        Returns:
            Optional[Union[JointState, JointStateDict]]:
                - 若指定 arm，则返回一个 JointState 对象。
                - 若未指定 arm，则返回一个包含两个 JointState 对象的元组：shape = (2, 7)，
                结构为 [left_arm_joint_state, right_arm_joint_state]
                - 若读取失败，则返回 None。
        """
        return self.data_parser.get_joint_state(arm)

    def read_joint_angles(self,arm: str = '') -> Optional[Union[JointState, JointStateDict]]:
        """
        """
        if arm in ["left_arm", "right_arm"]:
            return self.data_parser.get_joint_state(arm).angles
        if arm == "both":
            return [self.data_parser.get_joint_state(arm="left_arm").angles,
                    self.data_parser.get_joint_state(arm="right_arm").angles]
        return None


    def set_joint_angles(self,
                        joint_angles: List[float],
                        arm: str = None,
                        gripper_angle: float = None,
                        wait_for_completion: bool = True,
                        timeout: float = 5.0,
                        tolerance: float = 0.0524) -> bool:
        """
        设置机械臂关节角度（单位：弧度）

        支持：
        - 指定 arm：控制单个机械臂（传入一个 [float] 列表）

        Args:
            joint_angles: 单臂：长度为 7 的列表
            arm: "left_arm" 或 "right_arm"
            gripper_angle:单臂夹爪控制；
            wait_for_completion: 是否等待运动完成
            timeout: 最大等待时间
            tolerance: 每个关节允许的最大误差（弧度）

        Returns:
            bool: 是否发送和完成成功
        """
        if not arm:
            logger.error(f"请输入想要控制的机械臂，当前arm = {arm}")
            return False
        
        else:
            if not isinstance(joint_angles, list) or len(joint_angles) != self.joint_count:
                logger.error(f"{arm}：关节角度数量必须为 {self.joint_count}")
                return False

            mapped_angles = [angle * self.direction_map[arm][i] for i, angle in enumerate(joint_angles)]

            frame = self._build_joint_frame(mapped_angles, arm=arm)
            result = self.serial_comm.send_data(frame)

            # if gripper_angle is not None:
            #     result &= self.set_gripper(gripper_angle, arm=arm)

            if wait_for_completion and result:
                # 等待运动完成（仅在需要时使用，实时同步场景应使用 wait_for_completion=False）
                start_time = time.time()
                while time.time() - start_time < timeout:
                    angles_now = self.data_parser.get_joint_state(arm).angles
                    if all(abs(angles_now[i] - mapped_angles[i]) <= tolerance for i in range(self.joint_count)):
                        break
                    time.sleep(0.01)

                if time.time() - start_time >= timeout:
                    # logger.warning(f"等待目标位置超时")
                    return False

            return result

    
    def set_gripper(self, 
                    angle_rad: float, 
                    arm: str,
                    wait_for_completion: bool = True, 
                    timeout: float = 5.0, 
                    tolerance: float = 0.1) -> bool:
        """
        设置夹爪角度（弧度）

        Args:
            angle_rad: 单臂夹爪角度（用于指定 arm）
            arm: 
                - "left_arm" or "right_arm": 控制指定机械臂夹爪
            wait_for_completion: 是否等待夹爪运动完成
            timeout: 超时时间（秒）
            tolerance: 误差容忍范围（弧度）

        Returns:
            bool: 命令是否成功发送和执行
        """
        # 校验输入
        if not isinstance(angle_rad, (int, float)):
            logger.error(f"{arm} 模式下 angle_rad 应为 float 类型")
            return False
            
        # 构造夹爪控制帧
        frame = self._build_gripper_frame(angle_rad, arm=arm)
        
        # 发送夹爪控制命令
        result = self.serial_comm.send_data(frame)
        
        if not wait_for_completion or not result:
            return result

        # === 等待运动完成 ===
        start_time = time.time()

        if self.debug_mode:
            logger.info(f"等待 {arm} 夹爪运动到目标位置: {round(angle_rad * self.RAD_TO_DEG, 2)}°")

        while time.time() - start_time < timeout:
            gripper_now = self.data_parser.get_joint_state(arm).gripper
            if abs(gripper_now - angle_rad) <= tolerance:
                if self.debug_mode:
                    logger.debug(f"{arm} 夹爪已到达目标位置")
                return True
            time.sleep(0.01)

        # logger.warning(f"{arm} 夹爪运动完成超时")
        return False

    # ======================== 云台控制 ========================
    def set_gimbal(self,
                   x_angle_rad: float,
                   y_angle_rad: float,
                   wait_for_completion: bool = False,
                   timeout: float = 5.0,
                   tolerance: float = 0.05) -> bool:
        """
        设置云台 X/Y 轴角度（单位：弧度）。

        协议: AA 14 04 X_L X_H Y_L Y_H CHECK FF
        - 指令: 0x14
        - 数据长度: 0x04
        - 数据: X(2B, 小端) + Y(2B, 小端)
        - 校验: 从第3字节到倒数第3字节的和，对2取模
        """
        # 基本校验
        if not isinstance(x_angle_rad, (int, float)) or not isinstance(y_angle_rad, (int, float)):
            logger.error("set_gimbal: 角度应为 float 类型")
            return False

        frame = self._build_gimbal_frame(x_angle_rad, y_angle_rad)
        ok = self.serial_comm.send_data(frame)

        # 可选等待到位（当 DataParser 实现 get_gimbal_state 时生效）
        if wait_for_completion and ok and hasattr(self.data_parser, "get_gimbal_state"):
            t0 = time.time()
            while time.time() - t0 < timeout:
                try:
                    gs = self.data_parser.get_gimbal_state()  # 期望返回 (x_rad, y_rad)
                    if gs is not None:
                        pan, tilt = gs
                        if abs(pan - x_angle_rad) <= tolerance and abs(tilt - y_angle_rad) <= tolerance:
                            return True
                except Exception:
                    pass
                time.sleep(0.02)
            logger.warning("set_gimbal: 等待到位超时")
            return False

        return ok

    def set_gimbal_deg(self,
                       x_angle_deg: float,
                       y_angle_deg: float,
                       wait_for_completion: bool = False,
                       timeout: float = 5.0,
                       tolerance: float = 0.05) -> bool:
        """按角度(度)设置云台 X/Y 轴角度。"""
        return self.set_gimbal(x_angle_deg * self.DEG_TO_RAD,
                               y_angle_deg * self.DEG_TO_RAD,
                               wait_for_completion=wait_for_completion,
                               timeout=timeout,
                               tolerance=tolerance)

    
    def set_zero_position(self, arm:str="both") -> bool:
        """
        设置当前位置为零点
        
        Returns:
            bool: 命令是否成功发送
        """
        if not isinstance(arm, str) or arm not in ['left_arm', 'right_arm', 'both']:
                logger.error(f"请检查需要取消扭矩的arm名称，当前为{arm}")
                return False

        
        if arm == 'left_arm':
            data = self.LEFT_ARM
            arm_info = '左臂'
        elif arm == 'right_arm':
            data = self.RIGHT_ARM
            arm_info = '右臂'
        elif arm == 'both':
            data = self.BOTH_ARM
            arm_info = '双臂'
        # 构造零点设置帧
        frame = self._build_command_frame(self.CMD_ZERO_POS, arm=arm, data=[data])
       
        # 发送零点设置命令
        result = self.serial_comm.send_data(frame)
        time.sleep(1)  #延时等待指令生效

        if result:
            logger.info(f"{arm_info}归零成功")
        return result
    
    def enable_torque(self, arm:str) -> bool:
        """
        使能力矩控制（使机械臂保持当前位置）
        
        Returns:
            bool: 命令是否成功发送
        """
        if not isinstance(arm, str) or arm not in ['left_arm', 'right_arm', 'both']:
                logger.error(f"请检查需要取消扭矩的arm名称，当前为{arm}")
                return False
        
        data = [0] * 2
        data[1] = 0x01
        if arm == 'left_arm':
            data[0] = self.LEFT_ARM
            arm_info = '左臂'
        elif arm == 'right_arm':
            data[0] = self.RIGHT_ARM
            arm_info = '右臂'
        elif arm == 'both':
            data[0] = self.BOTH_ARM
            arm_info = '双臂'

        # 构造力矩使能帧
        frame = self._build_command_frame(self.CMD_TORQUE, arm=arm, data=data)
        
        # 发送力矩使能命令
        result = self.serial_comm.send_data(frame)
        time.sleep(1)

        if result:
            logger.info(f"{arm_info}扭矩已经开启")
        return result
    
    def disable_torque(self, arm:str) -> bool:
        """
        禁用力矩控制（使机械臂可以自由移动）
        
        Returns:
            bool: 命令是否成功发送
        """
        if not isinstance(arm, str) or arm not in ['left_arm', 'right_arm', 'both']:
                logger.error(f"请检查需要取消扭矩的arm名称，当前为{arm}")
                return False
        
        data = [0] * 2
        data[1] = 0x00
        if arm == 'left_arm':
            data[0] = self.LEFT_ARM
            arm_info = '左臂'
        elif arm == 'right_arm':
            data[0] = self.RIGHT_ARM
            arm_info = '右臂'
        elif arm == 'both':
            data[0] = self.BOTH_ARM
            arm_info = '双臂'

        # 构造力矩禁用帧
        frame = self._build_command_frame(cmd_id=self.CMD_TORQUE, arm=arm, data=data)
        
        # 发送力矩禁用命令
        result = self.serial_comm.send_data(frame)
        time.sleep(1)

        if result:
            logger.info(f"{arm_info}扭矩已经关闭")
        return result
    
    
    def _build_joint_frame(self, 
                       joint_angles: List[float],
                       arm: str = None) -> List[int]:
        """构建单臂关节控制帧 (新协议)
        协议: AA 06 LEN IDENT DATA(7*2B) CHECK FF
          IDENT: 0x01 右臂 / 0x02 左臂
          LEN = 1(IDENT) + 14(DATA) = 0x0F
          DATA: 7个关节，每关节 2 字节 little-endian (value 0-4095)
          CHECK = (IDENT + sum(DATA字节)) % 2
        joint_angles: 目标关节角（弧度）长度=7
        arm: 'left_arm' / 'right_arm'
        """
        if arm not in ['left_arm','right_arm']:
            logger.error(f"关节控制需指定单臂(left_arm/right_arm)，当前: {arm}")
            return []
        if len(joint_angles) != 7:
            logger.error(f"关节角数量应为7，当前: {len(joint_angles)}")
            return []
        # IDENT 约定: 0x01 = LEFT_ARM(left_arm), 0x02 = RIGHT_ARM(right_arm)
        ident = 0x01 if arm == 'left_arm' else 0x02
        # 方向映射
        mapped = [joint_angles[i] * self.direction_map[arm][i] for i in range(7)]
        # 转换为硬件值
        data_bytes: List[int] = []
        for ang in mapped:
            v = self._rad_to_hardware_value(ang)
            data_bytes.append(v & 0xFF)
            data_bytes.append((v >> 8) & 0xFF)
        length = 1 + len(data_bytes)  # IDENT + DATA
        frame = [0] * (length + 5)  # 头 指令 长度 IDENT+DATA 校验 尾
        frame[0] = self.FRAME_HEADER
        frame[1] = self.CMD_DUAL_ARM
        frame[2] = length
        frame[3] = ident
        # 写入 DATA
        for i, b in enumerate(data_bytes):
            frame[4 + i] = b
        # 计算校验
        checksum = (ident + sum(data_bytes)) % 2
        frame[-2] = checksum
        frame[-1] = self.FRAME_FOOTER
        if self.debug_mode:
            logger.debug(f"构建关节帧 {arm} angles(deg)={[round(a * self.RAD_TO_DEG, 1) for a in joint_angles]}")
        return frame


    
    def _build_gripper_frame(self, 
                         angle_rad: Union[float, Tuple[float, float]],
                         arm: str = None) -> List[int]:
        """
        构建夹爪控制帧（通过 joint_frame 中的夹爪位直接写入）

        Args:
            angle_rad: 
                - float: 单臂夹爪角度（弧度）
                - tuple: 双臂夹爪角度 (left_rad, right_rad)
            arm: 
                - None: 双臂控制
                - "left_arm" 或 "right_arm"

        Returns:
            List[int]: 控制帧字节列表
        """
        # 创建夹爪控制帧 (固定长度)
        frame = [0] * self.GRIPPER_FRAME_SIZE
        frame[0] = self.FRAME_HEADER
        frame[1] = self.CMD_GRIPPER
        frame[2] = self.GRIPPER_FRAME_SIZE - 5  # 数据长度
        frame[-1] = self.FRAME_FOOTER

        if arm == 'left_arm':
            frame[3] = self.LEFT_ARM
    
        elif arm == 'right_arm':
            frame[3] = self.RIGHT_ARM
        
        # 转换为硬件值
        gripper_value = self._rad_to_hardware_value_grip(angle_rad)
        
        # 写入夹爪角度
        offset = 4
        frame[offset] = gripper_value & 0xFF  # 低字节
        frame[offset+1] = (gripper_value >> 8) & 0xFF  # 高字节
        
        # 计算并设置校验和
        frame[-2] = self._calculate_checksum(frame)
        
        
        if self.debug_mode:
            angle_deg = round(angle_rad * self.RAD_TO_DEG, 2)
            logger.debug(f"发送夹爪角度: {angle_deg}度 ({angle_rad:.4f}弧度)")
        return frame

    
    def _build_command_frame(self, cmd_id: int, arm: str, data: List[int]) -> List[int]:
        """
        构建命令帧
        
        Args:
            cmd_id: 命令ID
            data: 数据字节列表
            
        Returns:
            List[int]: 控制帧字节列表
        """
        # 计算帧大小：帧头(1)+命令(1)+长度(1)+数据(n)+校验(1)+帧尾(1)
        frame_size = len(data) + self.FRAME_MINIMAL_SIZE
        
        # 创建帧
        frame = [0] * frame_size
        frame[0] = self.FRAME_HEADER
        frame[1] = cmd_id
        frame[2] = len(data)  # 数据长度
        
        if len(data) >= 1:
            # 写入数据
            for i, d in enumerate(data):
                frame[3 + i] = d
            
        # 设置帧尾
        frame[-1] = self.FRAME_FOOTER
        
        # 计算并设置校验和
        frame[-2] = self._calculate_checksum(frame)
            
        return frame
    
    def _rad_to_hardware_value(self, angle_rad: float) -> int:
        """
        将弧度转换为硬件值(0-4095)
        
        Args:
            angle_rad: 角度（弧度）
            
        Returns:
            int: 硬件值
        """
        # 先转换为角度
        angle_deg = angle_rad * self.RAD_TO_DEG
        
        # 范围检查
        if angle_deg < -180.0 or angle_deg > 180.0:
            logger.warning(f"角度值超出范围: {angle_deg:.2f}度，会被截断")
            angle_deg = max(-180.0, min(180.0, angle_deg))
        
        # 转换公式: -180° → 0, 0° → 2048, +180° → 4095
        value = int((angle_deg + 180.0) / 360.0 * 4096)
        
        # 范围限制
        return max(0, min(4095, value))
    
    def _rad_to_hardware_value_grip(self, angle_rad: float) -> int:
        """
        将弧度转换为夹爪舵机值(2048-2900)
        
        Args:
            angle_rad: 角度（弧度）
            
        Returns:
            int: 硬件值
        """
        # 先转换为角度
        angle_deg = angle_rad * self.RAD_TO_DEG
        
        # 范围检查
        if angle_deg < 0:
            logger.warning(f"夹爪角度值超出范围: {angle_deg:.2f}度，会被截断")
            angle_deg = 0
        elif angle_deg > 100.0:
            logger.warning(f"夹爪角度值超出范围: {angle_deg:.2f}度，会被截断")
            angle_deg = 100.0
        
        servo_value = 3590
        # 转换公式：0度对应2048，100度对应servo_value
        value = int(2048 + (angle_deg * (servo_value-2048)/100))
        
        # 范围限制
        return max(2048, min(servo_value, value))
    
    def _calculate_checksum(self, frame: List[int]) -> int:
        """
        计算校验和
        
        Args:
            frame: 完整的数据帧
            
        Returns:
            int: 校验和
        """
        # 计算从第3个字节到倒数第3个字节的所有元素之和
        checksum = 0
        for i in range(3, len(frame) - 2):
            checksum += frame[i]
        
        # 对2取模
        return checksum % 2

    def _build_gimbal_frame(self, x_angle_rad: float, y_angle_rad: float) -> List[int]:
        """
        构建云台角度控制帧
        帧格式: AA 14 04 X_L X_H Y_L Y_H CHECK FF
        """
        # 复用关节角度的硬件编码（-180~+180 -> 0~4095；0度为2048）
        x_val = self._rad_to_hardware_value(x_angle_rad)
        y_val = self._rad_to_hardware_value(y_angle_rad)

        # 总长度: 数据4 + 固定5 = 9 字节
        frame = [0] * (4 + self.FRAME_MINIMAL_SIZE)
        frame[0] = self.FRAME_HEADER
        frame[1] = self.CMD_GIMBAL
        frame[2] = 0x04  # 数据长度
        # X 轴 (小端)
        frame[3] = x_val & 0xFF
        frame[4] = (x_val >> 8) & 0xFF
        # Y 轴 (小端)
        frame[5] = y_val & 0xFF
        frame[6] = (y_val >> 8) & 0xFF
        # 帧尾与校验
        frame[-1] = self.FRAME_FOOTER
        frame[-2] = self._calculate_checksum(frame)
        if self.debug_mode:
            logger.debug(f"构建云台帧 X={x_angle_rad*self.RAD_TO_DEG:.1f}°, Y={y_angle_rad*self.RAD_TO_DEG:.1f}° -> x={x_val}, y={y_val}")
        return frame



    # ======================== 高层度制 API 与插值（吸收 utils/control.py） ========================
    def _build_speed_frame(self, speed_value: int) -> List[int]:
        """构建速度设置帧。
        帧格式: AA 06 03 04 [SPEED_L] [SPEED_H] CHECK FF
        - 指令: 0x06（沿用 CMD_DUAL_ARM 常量值）
        - 长度: 0x03 (识别 1B + 速度 2B)
        - 识别: 0x04
        - 速度: 0~3400, 小端
        - 校验: 从第3字节到倒数第3字节求和 % 2
        """
        speed = max(0, min(3400, int(speed_value)))
        frame = [0] * (3 + 5)  # 数据长度3 + 固定5字节（头、指令、长度、...、校验、尾）
        frame[0] = self.FRAME_HEADER
        frame[1] = self.CMD_DUAL_ARM  # 0x06
        frame[2] = 0x03
        frame[3] = 0x04
        frame[4] = speed & 0xFF
        frame[5] = (speed >> 8) & 0xFF
        frame[-1] = self.FRAME_FOOTER
        frame[-2] = self._calculate_checksum(frame)
        return frame

    def set_speed_raw(self, speed_value: int) -> bool:
        """设置速度原始值(0~3400)。"""
        frame = self._build_speed_frame(speed_value)
        return self.serial_comm.send_data(frame)

    def set_speed_deg_s(self, speed_deg_s: float) -> bool:
        """按度/秒设置速度：先转 rad/s，再按 (rad_s / 2π) * 3400 映射为原始值。"""
        try:
            deg_s = float(speed_deg_s)
        except Exception:
            return False
        rad_s = deg_s * self.DEG_TO_RAD
        max_angle_rad_per_sec = math.pi * 2.0
        raw = int(max(1, min(3400, (rad_s / max_angle_rad_per_sec) * 3400.0)))
        return self.set_speed_raw(raw)

    def set_speed_factor(self, speed_factor: float) -> bool:
        """按系数设置速度，对齐 Alicia：factor∈[0,1] → raw = clip(factor * 3400)。"""
        try:
            f = float(speed_factor)
        except Exception:
            return False
        raw = int(max(0.0, min(3400.0, f * 3400.0)))
        return self.set_speed_raw(raw)

    def move_joints_deg(self, arm: str, angles_deg: List[float], wait_for_completion: bool = True, tolerance_deg: float = 3.0) -> bool:
        """控制单臂7关节到指定角度（度制）。
        
        Args:
            arm: "left_arm" 或 "right_arm"
            angles_deg: 7个关节角度（度）
            wait_for_completion: 是否等待运动完成
            tolerance_deg: 每个关节允许的最大误差（度）
        """
        if arm not in ["left_arm", "right_arm"]:
            logger.error(f"move_joints_deg: arm 参数无效: {arm}")
            return False
        if not isinstance(angles_deg, list) or len(angles_deg) != 7:
            logger.error("move_joints_deg: 必须提供7个关节角度(度)")
            return False
        current = self.read_joint_state(arm)
        if not current or len(current) != 7:
            # logger.warning(f"move_joints_deg: 当前未获取到 {arm} 有效角度，直接发送目标")
            target = [a * self.DEG_TO_RAD for a in angles_deg]
            tolerance_rad = tolerance_deg * self.DEG_TO_RAD
            return self.set_joint_angles(joint_angles=target, arm=arm, wait_for_completion=wait_for_completion, tolerance=tolerance_rad)
        target = [a * self.DEG_TO_RAD for a in angles_deg]
        tolerance_rad = tolerance_deg * self.DEG_TO_RAD
        return self.set_joint_angles(joint_angles=target, arm=arm, wait_for_completion=wait_for_completion, tolerance=tolerance_rad)


    def move_dual_joints_deg(self,
                             left_angles_deg: List[float],
                             right_angles_deg: List[float],
                             wait_for_completion: bool = True,
                             tolerance_deg: float = 3.0,
                             ) -> bool:
        """控制双臂14关节到指定角度（度制）。
        
        Args:
            left_angles_deg: 左臂7个关节角度（度）
            right_angles_deg: 右臂7个关节角度（度）
            wait_for_completion: 是否等待运动完成（默认True）
            tolerance_deg: 每个关节允许的最大误差（度，默认3.0）
        """
        if not (isinstance(left_angles_deg, list) and len(left_angles_deg) == 7 and isinstance(right_angles_deg, list) and len(right_angles_deg) == 7):
            logger.error("move_dual_joints_deg: 左右臂都必须提供7个角度(度)")
            return False
        # current = self.read_joint_state(arm="both")
        target_left = [a * self.DEG_TO_RAD for a in left_angles_deg]
        target_right = [a * self.DEG_TO_RAD for a in right_angles_deg]
        tolerance_rad = tolerance_deg * self.DEG_TO_RAD

        # 双臂同步等待完成
        success = self.set_joint_angles(joint_angles=target_left, arm="left_arm", wait_for_completion=wait_for_completion, tolerance=tolerance_rad)
        success &= self.set_joint_angles(joint_angles=target_right, arm="right_arm", wait_for_completion=wait_for_completion, tolerance=tolerance_rad)
        return success


    def set_gripper_deg(self, arm: str, angle_deg: float, wait: bool = False) -> bool:
        """设置夹爪角度（度制）。"""
        if arm not in ["left_arm", "right_arm" , "both"]:
            logger.error(f"set_gripper_deg: arm 参数无效: {arm}")
            return False
        angle_rad = angle_deg * self.DEG_TO_RAD
        if arm in ["left_arm", "right_arm"]:
            return self.set_gripper(angle_rad, arm=arm, wait_for_completion=wait)
        elif arm == "both":
            return self.set_gripper(angle_rad, arm="left_arm", wait_for_completion=wait) and self.set_gripper(angle_rad, arm="right_arm", wait_for_completion=wait)
        else:
            logger.error(f"set_gripper_deg: arm 参数无效: {arm}")
            return False


    def print_joint_angles_deg(self, arm: str = "both", read_gripper: bool = True):
        """打印当前关节角度（度）。"""
        joint_angles = self.read_joint_state(arm)
        if arm == "both":
            left_deg = [round(a * self.RAD_TO_DEG, 2) for a in joint_angles[0]]
            right_deg = [round(a * self.RAD_TO_DEG, 2) for a in joint_angles[1]]
            print(f"左臂关节角度: {left_deg}")
            print(f"右臂关节角度: {right_deg}")
            if read_gripper:
                self.print_gripper_angles_deg(arm)
        else:
            angles_deg = [round(a * self.RAD_TO_DEG, 2) for a in joint_angles]
            print(f"{arm} 关节角度: {angles_deg}")
            if read_gripper:
                self.print_gripper_angles_deg(arm)

    def print_gripper_angles_deg(self, arm: str = "both"):
        """打印当前夹爪角度（度, 0~100）。"""
        gripper_angles = self.read_gripper_data(arm)
        if arm == "both":
            left_deg = round(gripper_angles[0] * self.RAD_TO_DEG, 2)
            right_deg = round(gripper_angles[1] * self.RAD_TO_DEG, 2)
            print(f"左夹爪角度: {left_deg}")
            print(f"右夹爪角度: {right_deg}")
        else:
            angle_deg = round(gripper_angles * self.RAD_TO_DEG, 2)
            if arm == 'left_arm':
                print(f"左夹爪角度: {angle_deg}")
            else:
                print(f"右夹爪角度: {angle_deg}")