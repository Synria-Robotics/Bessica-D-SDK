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
    FUNC_GRIPPER_ONLY = 0x06   # 仅夹爪控制（双臂）

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


    
    



    def set_joint_and_gripper(
        self,
        joint_angles: Optional[Union[List[float], List[List[float]]]] = None,
        gripper_value: Optional[Union[float, List[float], Tuple[float, float]]] = None,
        arm: str = "both",
        speed_deg_s: float = 20.0,
    ) -> bool:
        """
        Unified method to set joints, gripper, or both in a single combined frame.
        
        协议: AA 06 FUNC_CODE LEN DATA CHECK FF
        - 单臂: FUNC=0x04(右臂)或0x05(左臂), LEN=0x20 (32 bytes) = 7 joints * 4 + 1 gripper * 4
        - 双臂: FUNC=0x03, LEN=0x40 (64 bytes) = 右臂7关节*4 + 左臂7关节*4 + 左夹爪*4 + 右夹爪*4
        - 仅夹爪: FUNC=0x06, LEN=0x08 (8 bytes) = 左夹爪*4 + 右夹爪*4
        
        :param joint_angles: Optional angle list (radians).
            - Single arm: List[float] (7 angles). If None, keeps current joints
            - Dual arm: List[List[float]] (2x7 angles). If None, keeps current joints
        :param gripper_value: Optional gripper value (0-1000, where 1000 is fully open).
            - Single arm: float. If None, keeps current gripper
            - Dual arm: List[float] or Tuple[float, float] (left, right). If None, keeps current grippers
        :param arm: Arm to control, "left", "right", or "both" (default: "both")
        :param speed_deg_s: Speed in degrees per second (default: 20.0)
        :return: True if successful
        """
        if speed_deg_s <= 0:
            logger.error(f"Speed must be positive: {speed_deg_s} deg/s")
            return False

        frame = self._build_joint_and_gripper_frame(
            joint_angles=joint_angles,
            gripper_value=gripper_value,
            arm=arm,
            speed_deg_s=speed_deg_s
        )

        if not frame:
            return False

        if self.debug_mode:
            hex_print(logger, "Send combined joint+gripper control", frame)

        result = self.serial_comm.send_data(frame)
        return result


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
    
    
    def _build_joint_and_gripper_frame(
        self,
        joint_angles: Optional[Union[List[float], List[List[float]]]] = None,
        gripper_value: Optional[Union[float, List[float], Tuple[float, float]]] = None,
        arm: str = "both",
        speed_deg_s: float = 20.0,
    ) -> Optional[List[int]]:
        """
        Build combined joint + gripper + speed control frame
        
        协议: AA 06 FUNC_CODE LEN DATA CHECK FF
        
        单臂控制:
        - 指令: 0x06
        - 功能码: 0x04(右臂)或0x05(左臂)
        - 数据长度: 0x20 (32 bytes) = 单臂7关节角度、速度(28 bytes) + 夹爪角度、速度(4 bytes)
        - 数据格式: 关节角度 + 速度（每个关节4字节）+ 夹爪角度 + 速度（4字节）
        
        双臂控制:
        - 指令: 0x06
        - 功能码: 0x03
        - 数据长度: 0x40 (64 bytes) = 右臂7关节*4 + 左臂7关节*4 + 左夹爪*4 + 右夹爪*4
        - 数据格式: 关节角度 + 速度（每个关节4字节）数据低位在前高位在后
        
        仅夹爪控制:
        - 指令: 0x06
        - 功能码: 0x06
        - 数据长度: 0x08 (8 bytes) = 左右夹爪角度、速度
        - 数据格式: 夹爪角度 + 速度（每个夹爪4字节）数据低位在前高位在后
        
        :param joint_angles: Optional angle list (radians)
        :param gripper_value: Optional gripper value (0-1000)
        :param arm: Arm to control, "left", "right", or "both"
        :param speed_deg_s: Speed in degrees per second
        :return: Frame byte list or None if error
        """
        # Convert speed to hardware value (0-5000, where 5000 is max speed)
        speed_hw_value = self._deg_s_to_hardware_speed(speed_deg_s)
        # Get current state for optional values
        current_state = self.data_parser.get_joint_state(arm)
        gripper_speed_hw_value = 1000
        # gripper_speed_hw_value = 5500
        # Handle gripper-only case (both arms, no joints)
        if arm == "both" and joint_angles is None and gripper_value is not None:
            # Gripper-only control: FUNC=0x06, LEN=0x08
            DATA_LENGTH = 0x08
            FRAME_SIZE = 1 + 1 + 1 + 1 + DATA_LENGTH + 1 + 1
            
            frame = [0] * FRAME_SIZE
            frame[0] = self.FRAME_HEADER  # 0xAA
            frame[1] = self.CMD_DUAL_ARM  # 0x06
            frame[2] = self.FUNC_GRIPPER_ONLY  # 0x06
            frame[3] = DATA_LENGTH
            frame[-1] = self.FRAME_FOOTER  # 0xFF
            
            # Normalize gripper values
            if isinstance(gripper_value, (list, tuple)) and len(gripper_value) == 2:
                left_gripper = float(gripper_value[0])
                right_gripper = float(gripper_value[1])
            else:
                left_gripper = right_gripper = float(gripper_value)
            
            # Clamp gripper values to 0-1000
            left_gripper = max(0, min(1000, int(left_gripper)))
            right_gripper = max(0, min(1000, int(right_gripper)))
            
            # Write left gripper (4 bytes: 2 bytes value + 2 bytes speed)
            

            frame[4] = left_gripper & 0xFF
            frame[5] = (left_gripper >> 8) & 0xFF
            frame[6] = gripper_speed_hw_value & 0xFF
            frame[7] = (gripper_speed_hw_value >> 8) & 0xFF
            
            # Write right gripper (4 bytes: 2 bytes value + 2 bytes speed)
            frame[8] = right_gripper & 0xFF
            frame[9] = (right_gripper >> 8) & 0xFF
            frame[10] = gripper_speed_hw_value & 0xFF
            frame[11] = (gripper_speed_hw_value >> 8) & 0xFF
            
            # Calculate checksum
            frame[-2] = self.serial_comm.calculate_checksum(frame[1:-2])
            
            if self.debug_mode:
                logger.debug(f"Built gripper-only frame - left: {left_gripper}, right: {right_gripper}, speed: {speed_deg_s} deg/s")
            
            return frame
        
        if arm == "both":
            # Dual arm mode: 
            # For 灵越系列, always use 0x40 as it includes: 右臂7关节 + 左臂7关节 + 左右夹爪
            # 0x40 = 64 bytes = 右臂7关节*4 + 左臂7关节*4 + 左夹爪*4 + 右夹爪*4
            DATA_LENGTH = 0x40
            FRAME_SIZE = 1 + 1 + 1 + 1 + DATA_LENGTH + 1 + 1  # header + cmd + func + len + data + checksum + footer
            
            frame = [0] * FRAME_SIZE
            frame[0] = self.FRAME_HEADER  # 0xAA
            frame[1] = self.CMD_DUAL_ARM  # 0x06
            frame[2] = self.FUNC_JOINT_BOTH  # 0x03
            frame[3] = DATA_LENGTH
            frame[-1] = self.FRAME_FOOTER  # 0xFF
            
            data_start = 4
            
            # Get effective joint angles
            if joint_angles is None:
                if current_state and isinstance(current_state, dict):
                    left_js = current_state.get('left')
                    right_js = current_state.get('right')
                    effective_joints_left = left_js.angles if left_js and hasattr(left_js, 'angles') and left_js.angles else [0.0] * 7
                    effective_joints_right = right_js.angles if right_js and hasattr(right_js, 'angles') and right_js.angles else [0.0] * 7
                else:
                    effective_joints_left = [0.0] * 7
                    effective_joints_right = [0.0] * 7
            else:
                if not isinstance(joint_angles, list) or len(joint_angles) != 2:
                    logger.error("Dual arm mode requires joint_angles as List[List[float]] with 2x7 angles")
                    return None
                effective_joints_left = joint_angles[1]  # Left arm
                effective_joints_right = joint_angles[0]  # Right arm (protocol: right first)
            

            right_angles = effective_joints_right 
            left_angles = effective_joints_left
            # Write right arm joints (7 joints * 4 bytes)
            offset = data_start
            for joint_idx in range(7):
                angle_rad = right_angles[joint_idx]
                hw_value = self._rad_to_hardware_value(angle_rad)
                frame[offset] = hw_value & 0xFF
                frame[offset + 1] = (hw_value >> 8) & 0xFF
                frame[offset + 2] = speed_hw_value & 0xFF
                frame[offset + 3] = (speed_hw_value >> 8) & 0xFF
                offset += 4
            
            # Write left arm joints (7 joints * 4 bytes)
            for joint_idx in range(7):
                angle_rad = left_angles[joint_idx]
                hw_value = self._rad_to_hardware_value(angle_rad)
                frame[offset] = hw_value & 0xFF
                frame[offset + 1] = (hw_value >> 8) & 0xFF
                frame[offset + 2] = speed_hw_value & 0xFF
                frame[offset + 3] = (speed_hw_value >> 8) & 0xFF
                offset += 4
            
            # Get effective gripper values (dual arm always includes grippers in 0x40 frame)
            if gripper_value is None:
                if current_state and isinstance(current_state, dict):
                    left_js = current_state.get('left')
                    right_js = current_state.get('right')
                    left_gripper = left_js.gripper if left_js and hasattr(left_js, 'gripper') and left_js.gripper is not None else 1000.0
                    right_gripper = right_js.gripper if right_js and hasattr(right_js, 'gripper') and right_js.gripper is not None else 1000.0
                else:
                    left_gripper = 1000.0
                    right_gripper = 1000.0
            else:
                if isinstance(gripper_value, (list, tuple)) and len(gripper_value) == 2:
                    left_gripper = float(gripper_value[0])
                    right_gripper = float(gripper_value[1])
                else:
                    left_gripper = right_gripper = float(gripper_value)
            
            # Clamp gripper values to 0-1000
            left_gripper = max(0, min(1000, int(left_gripper)))
            right_gripper = max(0, min(1000, int(right_gripper)))
            
            # Write left gripper (4 bytes: 2 bytes value + 2 bytes speed)
            frame[offset] = left_gripper & 0xFF
            frame[offset + 1] = (left_gripper >> 8) & 0xFF
            frame[offset + 2] = gripper_speed_hw_value & 0xFF
            frame[offset + 3] = (gripper_speed_hw_value >> 8) & 0xFF
            offset += 4
            
            # Write right gripper (4 bytes: 2 bytes value + 2 bytes speed)
            frame[offset] = right_gripper & 0xFF
            frame[offset + 1] = (right_gripper >> 8) & 0xFF
            frame[offset + 2] = gripper_speed_hw_value & 0xFF
            frame[offset + 3] = (gripper_speed_hw_value >> 8) & 0xFF
            
        elif arm in ("left", "right"):
            # Single arm mode: 0x20 (32 bytes) = 7 joints * 4 + 1 gripper * 4
            DATA_LENGTH = 0x20
            FRAME_SIZE = 1 + 1 + 1 + 1 + DATA_LENGTH + 1 + 1
            
            frame = [0] * FRAME_SIZE
            frame[0] = self.FRAME_HEADER  # 0xAA
            frame[1] = self.CMD_DUAL_ARM  # 0x06
            frame[2] = self.FUNC_JOINT_LEFT if arm == "left" else self.FUNC_JOINT_RIGHT  # 0x05 or 0x04
            frame[3] = DATA_LENGTH
            frame[-1] = self.FRAME_FOOTER  # 0xFF
            
            data_start = 4
            
            # Get effective joint angles
            if joint_angles is None:
                if current_state:
                    if isinstance(current_state, dict):
                        js = current_state.get(arm)
                        effective_joints = js.angles if js and hasattr(js, 'angles') and js.angles else [0.0] * 7
                    else:
                        effective_joints = current_state.angles if hasattr(current_state, 'angles') and current_state.angles else [0.0] * 7
                else:
                    effective_joints = [0.0] * 7
            else:
                if not isinstance(joint_angles, list) or len(joint_angles) != 7:
                    logger.error(f"Single arm mode requires joint_angles as List[float] with 7 angles")
                    return None
                effective_joints = joint_angles
            
            # Apply direction mapping
            mapped_angles = effective_joints
            
            # Write joints (7 joints * 4 bytes = 28 bytes)
            offset = data_start
            for joint_idx in range(7):
                angle_rad = mapped_angles[joint_idx]
                hw_value = self._rad_to_hardware_value(angle_rad)
                frame[offset] = hw_value & 0xFF
                frame[offset + 1] = (hw_value >> 8) & 0xFF
                frame[offset + 2] = speed_hw_value & 0xFF
                frame[offset + 3] = (speed_hw_value >> 8) & 0xFF
                offset += 4
            
            # Get effective gripper value
            if gripper_value is None:
                if current_state:
                    if isinstance(current_state, dict):
                        js = current_state.get(arm)
                        effective_gripper = js.gripper if js and hasattr(js, 'gripper') and js.gripper is not None else 1000.0
                    else:
                        effective_gripper = current_state.gripper if hasattr(current_state, 'gripper') and current_state.gripper is not None else 1000.0
                else:
                    effective_gripper = 1000.0
            else:
                if isinstance(gripper_value, (list, tuple)):
                    logger.warning(f"单臂模式但提供了列表形式的gripper_value，使用第一个值")
                    effective_gripper = float(gripper_value[0])
                else:
                    effective_gripper = float(gripper_value)
            
            # Clamp gripper value to 0-1000
            effective_gripper = max(0, min(1000, int(effective_gripper)))
            
            # Write gripper (4 bytes: 2 bytes value + 2 bytes speed)
            frame[offset] = effective_gripper & 0xFF
            frame[offset + 1] = (effective_gripper >> 8) & 0xFF
            frame[offset + 2] = speed_hw_value & 0xFF
            frame[offset + 3] = (speed_hw_value >> 8) & 0xFF
            # Total: 7 joints * 4 + 1 gripper * 4 = 28 + 4 = 32 bytes (0x20)
        else:
            logger.error(f"Invalid arm parameter: {arm}")
            return None
        
        # Calculate checksum
        frame[-2] = self.serial_comm.calculate_checksum(frame[1:-2])
        
        if self.debug_mode:
            if arm == "both":
                logger.debug(f"Built combined frame (both arms) - joints: {joint_angles}, grippers: {gripper_value}, speed: {speed_deg_s} deg/s")
            else:
                logger.debug(f"Built combined frame ({arm} arm) - joints: {joint_angles}, speed: {speed_deg_s} deg/s")
        
        return frame


    def _deg_s_to_hardware_speed(self, speed_deg_s: int) -> int:
        """
        Converts speed from degrees per second to hardware value (50-5000, step 50).
        Mapping: 360 deg/s = 4096 ticks/s, so 50 ticks/s ≈ 4.39 deg/s, 5000 ticks/s ≈ 439.45 deg/s.

        :param speed_deg_s: The desired speed in degrees per second (4.39-439.45, required range)
        :return: A corresponding raw integer speed value (50-5000, multiple of 50)
        """
        # Hardware speed range: 50-5000 ticks/s (step 50)
        MIN_HARDWARE_VALUE = 50
        MAX_HARDWARE_VALUE = 5000
        STEP_SIZE = 50

        # Known mapping: 360 deg/s = 4096 ticks/s
        # Calculate speed range based on hardware range
        # Ratio: 360 / 4096 = 0.087890625 deg/(tick/s)
        DEG_PER_TICK_PER_SEC = 360.0 / 4096.0
        MIN_SPEED_DEG_S = MIN_HARDWARE_VALUE * DEG_PER_TICK_PER_SEC  # ≈ 4.39 deg/s
        MAX_SPEED_DEG_S = MAX_HARDWARE_VALUE * DEG_PER_TICK_PER_SEC  # ≈ 439.45 deg/s

        # Validate and clip speed to required range
        if speed_deg_s < MIN_SPEED_DEG_S:
            logger.warning(f"Speed below range: {speed_deg_s} deg/s (min {MIN_SPEED_DEG_S:.2f}), will be clipped to {MIN_SPEED_DEG_S:.2f}")
            speed_deg_s = MIN_SPEED_DEG_S
        elif speed_deg_s > MAX_SPEED_DEG_S:
            logger.warning(f"Speed above range: {speed_deg_s} deg/s (max {MAX_SPEED_DEG_S:.2f}), will be clipped to {MAX_SPEED_DEG_S:.2f}")
            speed_deg_s = MAX_SPEED_DEG_S

        # Convert deg/s to ticks/s using the known ratio
        hardware_value = speed_deg_s / DEG_PER_TICK_PER_SEC

        # Round to nearest multiple of 50
        hardware_value = round(hardware_value / STEP_SIZE) * STEP_SIZE
        logger.debug(f"Speed: {speed_deg_s} deg/s, Hardware value: {hardware_value}")

        return max(MIN_HARDWARE_VALUE, min(MAX_HARDWARE_VALUE, int(hardware_value)))

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





    def set_gripper_deg(self, arm: str, angle_deg: float, wait: bool = False) -> bool:
        """设置夹爪角度（度制）。使用统一的 set_joint_and_gripper 方法。"""
        if arm not in ["left", "right", "both"]:
            logger.error(f"set_gripper_deg: arm 参数无效: {arm}")
            return False
        
        # Convert degrees to gripper value (0-1000)
        # Assuming angle_deg is 0-100 where 0 is closed and 100 is open
        gripper_value = max(0, min(1000, int(angle_deg * 10)))
        
        if arm == "both":
            gripper_value = [gripper_value, gripper_value]
        
        return self.set_joint_and_gripper(
            joint_angles=None,
            gripper_value=gripper_value,
            arm=arm,
            speed_deg_s=20.0
        )

