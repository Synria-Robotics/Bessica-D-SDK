import math
import time
import logging
import threading
from typing import List, Optional, Union, Tuple, Dict
import numpy as np
import traceback
from ..utils.logger import logger, hex_print
from .serial_comm import SerialComm
from .data_parser import DataParser, JointState, JointStateDict



class ServoDriver:
    """机械臂控制模块"""
    
    # 常量定义
    RAD_TO_DEG = 180.0 / math.pi  # 弧度转角度系数
    DEG_TO_RAD = math.pi / 180.0  # 角度转弧度系数

    # Command IDs
    CMD_JOINT = 0x06       # Arm joint angle feedback and control

    # Gripper type configuration
    GRI_MAX_50MM = 3290
    GRI_MAX_100MM = 3590


    # 帧常量
    FRAME_HEADER = 0xAA
    FRAME_FOOTER = 0xFF
    FRAME_MINIMAL_SIZE = 6
    ARM_DATA_SIZE = 21
    GRIPPER_FRAME_SIZE = 8


    # 指令ID
    CMD_GRIPPER = 0x02     # 夹爪控制与行程反馈
    CMD_ZERO_POS = 0x03    # 机械臂以当前位置为零点  
    CMD_DUAL_ARM = 0x06   # 双臂角度反馈与控制
    CMD_TORQUE = 0x05      # 机械臂力矩控制 (协议: 0x05)
    CMD_GIMBAL = 0x14      # 云台角度控制 (X/Y)
    
    # 功能码 (Function codes)
    FUNC_TORQUE_BOTH = 0x00    # 双臂力矩控制
    FUNC_TORQUE_RIGHT = 0x01   # 右臂力矩控制
    FUNC_TORQUE_LEFT = 0x02    # 左臂力矩控制
    FUNC_ZERO_BOTH = 0x00      # 双臂零点设置
    FUNC_ZERO_RIGHT = 0x01     # 右臂零点设置
    FUNC_ZERO_LEFT = 0x02      # 左臂零点设置
    FUNC_JOINT_BOTH = 0x03     # 双臂关节控制
    FUNC_JOINT_RIGHT = 0x04    # 右臂关节控制
    FUNC_JOINT_LEFT = 0x05     # 左臂关节控制

    # 识别帧
    PRESENT_POSITION = 0x38 #当前机械臂关节角度识别帧
    # PRESENT_SPEED = 0x41    #当前机械臂关节速度识别 （待开发）

    left = 0X02
    right = 0X01
    BOTH_ARM = 0x03


    # Raw instruction mapping table for information retrieval and control
    # Note: torque_on/torque_off are handled dynamically via _build_torque_frame() to support arm selection
    # Note: joint_gripper, temperature, velocity are handled dynamically via _build_joint_gripper_frame()
    INFO_COMMAND_MAP: Dict[str, List[int]] = {
        # Get firmware version
        "version": [0xAA, 0x01, 0x00, 0x01, 0xFE, 0x23, 0xFF],
        # Set current position as zero
        "zero_cali": [0xAA, 0x03, 0x00, 0x01, 0xFE, 0xA8, 0xFF],
        # Joint information acquisition (position and status) - built dynamically
        # "joint_gripper": built via _build_joint_gripper_frame(0x00)
        # Temperature information acquisition - built dynamically
        # "temperature": built via _build_joint_gripper_frame(0x01)
        # Velocity information acquisition - built dynamically
        # "velocity": built via _build_joint_gripper_frame(0x02)
        "self_check": [0xAA, 0xFE, 0x00, 0x00, 0xFE, 0x93, 0xFF],
        # Gripper type acquisition
        "gripper_type": [0xAA, 0x04, 0x0E, 0x01, 0xFE, 0x1B, 0xFF],
    }

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
        self._lock = threading.Lock()

        # 创建串口通信模块和数据解析器
        self.serial_comm = SerialComm(lock=self._lock, port=port, debug_mode=debug_mode)
        self.data_parser = DataParser(lock=self._lock, debug_mode=debug_mode)
        
        # 舵机数量
        self.servo_count = 9
        self.joint_count = 7
        
        self.count = 0


        self.direction_map = {
            "left":  [1, -1, 1, -1, 1, 1, -1],
            "right": [1, 1, 1, -1, 1, 1, 1]
        }

        # 状态更新线程相关
        self._update_thread = None
        self.read_interval = 0.005
        self._stop_thread = threading.Event()
        self._thread_running = False
        


        self.disconnect()

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
        return result
    
    def disconnect(self):
        """断开与机械臂的连接"""
        # 先停止状态更新线程
        self.stop_update_thread()
        self.serial_comm.disconnect()
    
    def start_update_thread(self):
        """启动状态更新线程"""
        if self._update_thread is not None and self._thread_running:
            return
        
        # 重置停止信号
        self._stop_thread.clear()
        self._thread_running = True
        
        # 创建并启动线程
        self._update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self._update_thread.start()

        
    
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
        while not self._stop_thread.is_set():
            time.sleep(self.read_interval)
            try:

                with self._lock:
                    frame = self.serial_comm.read_frame()
                if frame and frame != 9999999:
                    self.data_parser.parse_frame(frame)
            except Exception as e:
                logger.error(f"状态线程异常：{e}")
                break
        self._thread_running = False



    def set_block_order(self, order: Tuple[str, str]):
        """设置底层解析器双臂数据块顺序。
        默认固件顺序可能为 ("right","left")，如果拖拽示教出现左右互换，可调用：
            controller.set_block_order(("left","right"))
        """
        if not isinstance(order, tuple) or len(order) != 2:
            logger.error(f"block_order 必须是长度为2的元组, 当前: {order}")
            return False
        if set(order) != {"left","right"}:
            logger.error(f"block_order 只允许包含 left / right, 当前: {order}")
            return False
        try:
            self.data_parser.block_order = order
            logger.info(f"已更新解析 block_order = {order}")
            return True
        except Exception as e:
            logger.error(f"更新 block_order 失败: {e}")
            return False


    def acquire_info(self, info_type: str, wait: bool = False, timeout: float = 2.0, 
                     retry_interval: float = 0.2, arm: str = 'both') -> bool:
        """
        General information acquisition interface, selecting different commands by type.

        :param info_type: Type of information to acquire (version, zero_cali, torque_on, torque_off, joint, etc.)
        :param wait: If True, wait for the response to be received and parsed
        :param timeout: Maximum time to wait in seconds (only used if wait=True)
        :param retry_interval: Time interval between retry attempts in seconds (default 0.2s)
        :param arm: Arm selection for torque commands ('left', 'right', 'both'). Default: 'both'
        :return: True if successful
        """
        # Handle torque commands dynamically (support arm selection)
        if info_type in ('torque_on', 'torque_off'):
            enable = (info_type == 'torque_on')
            if arm not in ['left', 'right', 'both']:
                logger.error(f"无效的arm参数: {arm}")
                return False
            
            func_map = {
                'both': self.FUNC_TORQUE_BOTH,
                'right': self.FUNC_TORQUE_RIGHT,
                'left': self.FUNC_TORQUE_LEFT
            }
            func_code = func_map[arm]
            data_value = 0x01 if enable else 0x00
            command = self._build_torque_frame(func_code, data_value)
        # Handle joint_gripper, temperature, velocity commands dynamically
        elif info_type in ('joint_gripper', 'temperature', 'velocity'):
            func_code_map = {
                'joint_gripper': 0x00,
                'temperature': 0x01,
                'velocity': 0x02
            }
            func_code = func_code_map[info_type]
            command = self._build_joint_gripper_frame(func_code)
        elif info_type not in self.INFO_COMMAND_MAP:
            raise ValueError(f"Unsupported info type: {info_type}")
        else:
            command = self.INFO_COMMAND_MAP[info_type]

        # Clear the corresponding event before sending request (if applicable)
        if info_type in self.data_parser._info_event_map:
            event = self.data_parser._info_event_map[info_type]
            event.clear()

        # If not waiting, just send once
        if not wait:
            success = self.serial_comm.send_data(command)
            return success

        # If waiting and has an event, implement retry logic
        if info_type in self.data_parser._info_event_map:
            event = self.data_parser._info_event_map[info_type]
            start_time = time.time()

            while time.time() - start_time < timeout:
                # Send command
                success = self.serial_comm.send_data(command)
                if not success:
                    logger.warning(f"Failed to send {info_type} command, retrying...")
                    time.sleep(retry_interval)
                    continue

                # Wait for response with a short timeout (retry_interval)
                remaining_time = timeout - (time.time() - start_time)
                wait_time = min(retry_interval, remaining_time)

                if event.wait(wait_time):
                    # Successfully received response
                    return True

            # Timeout exceeded
            logger.warning(f"Failed to get {info_type} within timeout period after multiple retries")
            return False
        else:
            # For commands without events, just send once
            success = self.serial_comm.send_data(command)
            return success

    def read_gripper_data(self, arm: str = 'both') -> Union[float, Tuple[float, float]]:
        """
        读取机械臂夹爪的当前角度（单位：弧度）

        Args:
            arm (str, optional): 指定要读取的机械臂，可选值为 "left" 或 "right"。
                                若为 both，则同时返回左右两个机械臂的夹爪角度。

        Returns:
            Union[float, Tuple[float, float]]:
                - 若指定 arm，则返回该机械臂的夹爪角度（float）。
                - 若未指定 arm，则返回一个元组 (left_gripper, right_gripper)。
        """
        joint_states = self.data_parser.get_joint_state(arm)

        if arm == 'both':
            return (joint_states['left'].gripper,
                    joint_states['right'].gripper)
            
        else:
            return joint_states.gripper

    
    
    def read_joint_state(self, arm: str = 'both') -> Optional[Union[JointState, JointStateDict]]:
        """
        读取机械臂的所有信息。

        Returns:
            Optional[Union[JointState, JointStateDict]]:
                - 若指定 arm，则返回一个 JointState 对象。
                - 若未指定 arm，则返回一个包含两个 JointState 对象的元组：shape = (2, 7)，
                结构为 [left_joint_state, right_joint_state]
                - 若读取失败，则返回 None。
        """
        return self.data_parser.get_joint_state(arm)

    def read_joint_angles(self,arm: str = '') -> Optional[Union[JointState, JointStateDict]]:
        """
        """
        if arm in ["left", "right"]:
            return self.data_parser.get_joint_state(arm).angles
        if arm == "both":
            return [self.data_parser.get_joint_state(arm="left").angles,
                    self.data_parser.get_joint_state(arm="right").angles]
        return None


    def set_joint_angles(self,
                        joint_angles: Union[List[float], List[List[float]]],
                        arm: str = None,
                        gripper_angle: float = None,
                        wait_for_completion: bool = True,
                        timeout: float = 5.0,
                        tolerance: float = 0.0524) -> bool:
        """
        设置机械臂关节角度（单位：弧度）

        支持：
        - 单臂：joint_angles 为 List[float] (7个角度), arm 为 "left" 或 "right"
        - 双臂：joint_angles 为 List[List[float]] (2x7角度), arm 为 "both"

        Args:
            joint_angles: 单臂：长度为 7 的列表；双臂：长度为 2 的列表，每个元素为 7 个角度
            arm: "left", "right", 或 "both"
            gripper_angle: 单臂夹爪控制（暂未实现）
            wait_for_completion: 是否等待运动完成
            timeout: 最大等待时间
            tolerance: 每个关节允许的最大误差（弧度）

        Returns:
            bool: 是否发送和完成成功
        """
        if not arm:
            logger.error(f"请输入想要控制的机械臂，当前arm = {arm}")
            return False
        
        # 判断是单臂还是双臂
        is_dual = isinstance(joint_angles, list) and len(joint_angles) == 2 and isinstance(joint_angles[0], list)
        
        if is_dual:
            if arm != 'both':
                logger.error(f"双臂模式时arm必须为'both'，当前: {arm}")
                return False
        else:
            if arm not in ['left', 'right']:
                logger.error(f"单臂模式时arm必须为'left'或'right'，当前: {arm}")
                return False
            if not isinstance(joint_angles, list) or len(joint_angles) != self.joint_count:
                logger.error(f"{arm}：关节角度数量必须为 {self.joint_count}，当前: {len(joint_angles) if isinstance(joint_angles, list) else '非列表'}")
                return False

        frame = self._build_joint_frame(joint_angles, arm=arm)
        if not frame:
            return False
        result = self.serial_comm.send_data(frame)
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
                - "left" or "right": 控制指定机械臂夹爪
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
        # frame_hex = " ".join([f"{byte:02X}" for byte in frame])
        # print("frame gripper: ", frame_hex)
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
        if not isinstance(arm, str) or arm not in ['left', 'right', 'both']:
                logger.error(f"请检查需要取消扭矩的arm名称，当前为{arm}")
                return False


        # 构造零点设置帧
        zero_map = {
            'both': 0x00,
            'right': 0x01,
            'left': 0x02
        }
        data = zero_map[arm]
        frame = self._build_zero_calibration_frame(self.CMD_ZERO_POS, arm=arm, data=[data])
       
        # 发送零点设置命令
        result = self.serial_comm.send_data(frame)

        return result
    
    def enable_torque(self, arm: str) -> bool:
        """使能力矩控制（使机械臂保持当前位置）"""
        return self._set_torque(arm, enable=True)
    
    def disable_torque(self, arm: str) -> bool:
        """禁用力矩控制（使机械臂可以自由移动）"""
        return self._set_torque(arm, enable=False)
    
    def _set_torque(self, arm: str, enable: bool) -> bool:
        """
        统一力矩控制方法
        
        Args:
            arm: 'left', 'right', or 'both'
            enable: True to enable, False to disable
        """
        if arm not in ['left', 'right', 'both']:
            logger.error(f"无效的arm参数: {arm}")
            return False
        
        # 映射arm到功能码
        func_map = {
            'both': self.FUNC_TORQUE_BOTH,
            'right': self.FUNC_TORQUE_RIGHT,
            'left': self.FUNC_TORQUE_LEFT
        }
        func_code = func_map[arm]
        data_value = 0x01 if enable else 0x00
        
        # 构建帧: AA 05 FUNC_CODE 01 DATA CHECK FF
        frame = self._build_torque_frame(func_code, data_value)
        result = self.serial_comm.send_data(frame)
        time.sleep(1)
        
        if result:
            arm_names = {'both': '双臂', 'right': '右臂', 'left': '左臂'}
            status = '开启' if enable else '关闭'
            logger.info(f"{arm_names[arm]}扭矩已经{status}")
        return result
    
    
    def _build_joint_frame(self, 
                       joint_angles: Union[List[float], List[List[float]]],
                       arm: str = None) -> List[int]:
        """
        构建关节控制帧 (支持单臂/双臂)
        协议: AA 06 FUNC_CODE LEN DATA CHECK FF
          FUNC_CODE: 0x03(双臂), 0x04(右臂), 0x05(左臂)
          DATA: 关节角度数据 (每关节2字节 little-endian)
        """
        # 确定是单臂还是双臂
        is_dual = isinstance(joint_angles, list) and len(joint_angles) == 2 and isinstance(joint_angles[0], list)
        
        if is_dual:
            if arm != 'both':
                logger.error("双臂模式时arm必须为'both'")
                return []
            if len(joint_angles[0]) != 7 or len(joint_angles[1]) != 7:
                logger.error("双臂模式需要左右各7个关节角度")
                return []
            func_code = self.FUNC_JOINT_BOTH
            # 处理双臂数据: 右臂 + 左臂
            right_angles = [joint_angles[0][i] * self.direction_map['right'][i] for i in range(7)]
            left_angles = [joint_angles[1][i] * self.direction_map['left'][i] for i in range(7)]
            data_bytes = self._angles_to_bytes(right_angles) + self._angles_to_bytes(left_angles)
        else:
            if arm not in ['left', 'right']:
                logger.error(f"单臂模式需指定arm(left/right)，当前: {arm}")
                return []
            if len(joint_angles) != 7:
                logger.error(f"关节角数量应为7，当前: {len(joint_angles)}")
                return []
            func_code = self.FUNC_JOINT_RIGHT if arm == 'right' else self.FUNC_JOINT_LEFT
            mapped = [joint_angles[i] * self.direction_map[arm][i] for i in range(7)]
            data_bytes = self._angles_to_bytes(mapped)
        
        # 构建帧: AA 06 FUNC_CODE LEN DATA CHECK FF
        length = len(data_bytes)
        frame = [0] * (length + 6)  # 头 + 指令 + 功能码 + 长度 + 数据 + 校验 + 尾
        frame[0] = self.FRAME_HEADER
        frame[1] = self.CMD_DUAL_ARM
        frame[2] = func_code
        frame[3] = length
        for i, b in enumerate(data_bytes):
            frame[4 + i] = b
        frame[-1] = self.FRAME_FOOTER
        # 校验: 使用CRC-32计算 Cmd + Func + Len + Data (frame[1:-2])
        frame[-2] = self.serial_comm.calculate_checksum(frame[1:-2])
        
        if self.debug_mode:
            if is_dual:
                logger.debug(f"构建双臂关节帧 angles(deg)={[[round(a * self.RAD_TO_DEG, 1) for a in angles] for angles in joint_angles]}")
            else:
                logger.debug(f"构建关节帧 {arm} angles(deg)={[round(a * self.RAD_TO_DEG, 1) for a in joint_angles]}")
        return frame
    
    def _angles_to_bytes(self, angles: List[float]) -> List[int]:
        """将角度列表转换为字节数组 (每角度2字节 little-endian)"""
        data_bytes = []
        for ang in angles:
            v = self._rad_to_hardware_value(ang)
            data_bytes.append(v & 0xFF)
            data_bytes.append((v >> 8) & 0xFF)
        return data_bytes
    
    def _build_torque_frame(self, func_code: int, data_value: int) -> List[int]:
        """
        构建力矩控制帧
        协议: AA 05 FUNC_CODE 01 DATA CHECK FF
        校验: 使用CRC-32计算，取最后8位 (计算范围: Cmd + Func + Len + Data)
        """
        frame = [0] * 7  # 固定长度: 头 + 指令 + 功能码 + 长度 + 数据 + 校验 + 尾
        frame[0] = self.FRAME_HEADER
        frame[1] = self.CMD_TORQUE
        frame[2] = func_code
        frame[3] = 0x01  # 数据长度
        frame[4] = data_value
        frame[-1] = self.FRAME_FOOTER
        # 校验: 使用CRC-32计算 Cmd + Func + Len + Data (frame[1:-2])
        frame[-2] = self.serial_comm.calculate_checksum(frame[1:-2])
        return frame

    def _build_joint_gripper_frame(self, func_code: int = 0x00) -> List[int]:
        """
        构建关节及夹爪数据获取帧
        协议: AA 06 FUNC_CODE 01 0xFE CHECK FF
        校验: 使用CRC-32计算，取最后8位 (计算范围: Cmd + Func + Len + Data)
        
        Args:
            func_code: 功能码 (0x00: 关节夹爪数据, 0x01: 温度, 0x02: 速度)
        """
        frame = [0] * 7  # 固定长度: 头 + 指令 + 功能码 + 长度 + 数据 + 校验 + 尾
        frame[0] = self.FRAME_HEADER
        frame[1] = self.CMD_JOINT
        frame[2] = func_code
        frame[3] = 0x01  # 数据长度
        frame[4] = 0xFE  # 数据
        frame[-1] = self.FRAME_FOOTER
        # 校验: 使用CRC-32计算 Cmd + Func + Len + Data (frame[1:-2])
        frame[-2] = self.serial_comm.calculate_checksum(frame[1:-2])
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
                - "left" 或 "right"

        Returns:
            List[int]: 控制帧字节列表
        """
        # 创建夹爪控制帧 (固定长度)
        frame = [0] * self.GRIPPER_FRAME_SIZE
        frame[0] = self.FRAME_HEADER
        frame[1] = self.CMD_GRIPPER
        frame[2] = self.GRIPPER_FRAME_SIZE - 5  # 数据长度
        frame[-1] = self.FRAME_FOOTER

        if arm == 'left':
            frame[3] = self.left
    
        elif arm == 'right':
            frame[3] = self.right
        
        # 转换为硬件值
        gripper_value = self._rad_to_hardware_value_grip(angle_rad)
        
        # 写入夹爪角度
        offset = 4
        frame[offset] = gripper_value & 0xFF  # 低字节
        frame[offset+1] = (gripper_value >> 8) & 0xFF  # 高字节
        
        # 计算并设置校验和 (使用CRC-32)
        frame[-2] = self.serial_comm.calculate_checksum(frame[1:-2])
        
        
        if self.debug_mode:
            angle_deg = round(angle_rad * self.RAD_TO_DEG, 2)
            logger.debug(f"发送夹爪角度: {angle_deg}度 ({angle_rad:.4f}弧度)")
        return frame

    
    def _build_command_frame(self, cmd_id: int, arm: str, data: List[int]) -> List[int]:
        """
        构建命令帧 (保留用于向后兼容，如zero_position等)
        
        Args:
            cmd_id: 命令ID
            arm: 臂标识 (用于某些需要arm信息的命令)
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
        
        # 计算并设置校验和 (使用CRC-32)
        frame[-2] = self.serial_comm.calculate_checksum(frame[1:-2])
            
        return frame
    

    def _build_zero_calibration_frame(self, cmd_id: int, arm: str, data: List[int]) -> List[int]:
        """
        Building zero calibration frame
        Frame structure: [Header, CmdID, FunctionCode, DataLength, Data, Checksum, Footer]
        """
        # Frame size: Header(1) + CmdID(1) + FunctionCode(1) + DataLength(1) + Data(1) + Checksum(1) + Footer(1) = 7
        frame = [0] * 7
        frame[0] = self.FRAME_HEADER
        frame[1] = cmd_id
        frame[2] = data[0]  # Function code (0x00 for both, 0x01 for right, 0x02 for left)
        frame[3] = 0x01     # Data length
        frame[4] = 0xFE     # Data

        # 计算并设置校验和 (使用CRC-32)
        frame[-2] = self.serial_comm.calculate_checksum(frame[1:-2])
        frame[-1] = self.FRAME_FOOTER
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
        # 校验: 使用CRC-32计算 Cmd + Len + Data (frame[1:-2])
        frame[-2] = self.serial_comm.calculate_checksum(frame[1:-2])
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
        # 校验: 使用CRC-32计算 Cmd + Func + Len + Data (frame[1:-2])
        frame[-2] = self.serial_comm.calculate_checksum(frame[1:-2])
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




    def set_gripper_deg(self, arm: str, angle_deg: float, wait: bool = False) -> bool:
        """设置夹爪角度（度制）。"""
        if arm not in ["left", "right" , "both"]:
            logger.error(f"set_gripper_deg: arm 参数无效: {arm}")
            return False
        angle_rad = angle_deg * self.DEG_TO_RAD
        if arm in ["left", "right"]:
            return self.set_gripper(angle_rad, arm=arm, wait_for_completion=wait)
        elif arm == "both":
            return self.set_gripper(angle_rad, arm="left", wait_for_completion=wait) and self.set_gripper(angle_rad, arm="right", wait_for_completion=wait)
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
            if arm == 'left':
                print(f"左夹爪角度: {angle_deg}")
            else:
                print(f"右夹爪角度: {angle_deg}")