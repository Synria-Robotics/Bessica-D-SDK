import math
import time
import logging
import threading
from typing import List, Optional, Union, Tuple, Dict
import numpy as np
import traceback

from .serial_comm import SerialComm
from .data_parser import DataParser, JointState, JointStateDict

# 配置日志
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("Controller")

class ArmController:
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

    # 识别帧
    PRESENT_POSITION = 0x38 #当前机械臂关节角度识别帧
    # PRESENT_SPEED = 0x41    #当前机械臂关节速度识别 （待开发）

    LEFT_ARM = 0X01
    RIGHT_ARM = 0X02
    BOTH_ARM = 0x03


    def __init__(self, port: str = "", baudrate: int = 921600, debug_mode: bool = False):
        """
        初始化机械臂控制器
        
        Args:
            port: 串口名称，留空则自动搜索
            baudrate: 波特率
            debug_mode: 是否启用调试模式
        """
        self.debug_mode = debug_mode
        self._lock = threading.Lock()

        # 创建串口通信模块和数据解析器
        self.serial_comm = SerialComm(lock=self._lock, port=port, baudrate=baudrate, debug_mode=debug_mode)
        self.data_parser = DataParser(lock=self._lock, debug_mode=debug_mode)
        
        # 舵机数量
        self.servo_count = 10
        self.joint_count = 7
        
        self.count = 0
        self.joint_to_servo_map = [
            (0, -1.0),    # 关节1 -> 舵机1 (正向)
            (0, -1.0),    # 关节1 -> 舵机2 (正向)
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
            "left_arm":  [1, 1, 1, 1, 1, -1, 1],  
            "right_arm": [-1, 1, -1, 1, -1, -1, 1]      
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
    
    def _update_loop(self):
        """状态更新线程主循环"""
        logger.info("状态更新线程开始运行")
        while not self._stop_thread.is_set():
            time.sleep(self.read_interval)
            try:
                with self._lock:
                    frame = self.serial_comm.read_frame()

                if frame == 9999999:
                        logger.error("串口读取异常，线程终止")
                        break
                
                if frame:
                        self.data_parser.parse_frame(frame)
                   
            except Exception as e:
                logger.error(f"状态线程异常：{e}")
                break
            
        self._thread_running = False
        logger.info("状态更新线程结束")

    def read_joint_angles(self, arm: str = 'both') -> Optional[Union[List[float], List[List[float]]]]:
        """
        读取机械臂的关节角度（单位：弧度）

        Args:
            arm (str, optional): 指定读取的机械臂名称，可选值为 "left_arm" 或 "right_arm"。
                                如果未指定（默认为 None），则同时返回左右两个机械臂的角度。

        Returns:
            Optional[Union[List[float], List[List[float]]]]:
                - 若指定 arm，则返回一个长度为 7 的一维列表：shape = (7,)
                - 若 arm 为 both，则返回一个包含两个一维列表的二维列表：shape = (2, 7)，
                结构为 [left_arm_angles, right_arm_angles]
                - 若读取失败，则返回 None。
        """
        joint_states = self.data_parser.get_joint_state()

        if arm == 'both':
            return [joint_states['left_arm'].angles ,
                    joint_states['right_arm'].angles]
        else:
            return joint_states[arm].angles


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
        读取机械臂的完整状态信息（关节角度、夹爪、按钮等）。

        Args:
            arm (str, optional): 指定要读取的机械臂。可选值为 "left_arm" 或 "right_arm"。
                                若为 both，则返回左右两个机械臂的完整状态字典。

        Returns:
            Optional[Union[JointState, JointStateDict]]:
                - 如果指定 arm，则返回对应机械臂的 JointState。
                - 如果 arm 为 both，则返回包含左右机械臂的 JointStateDict：
                {"left_arm": JointState, "right_arm": JointState}
                - 若状态尚未可用，则返回 None。
        """
        return self.data_parser.get_joint_state(arm)

    def set_joint_angles(self,
                        joint_angles: List[float],
                        arm: str = None,
                        gripper_angle: float = None,
                        wait_for_completion: bool = True,
                        timeout: float = 5.0,
                        tolerance: float = 0.08) -> bool:
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

            if gripper_angle is not None:
                result &= self.set_gripper(gripper_angle, arm=arm)

            if wait_for_completion and result:
                logger.info(f"等待 {arm} 到达目标位置")
                start_time = time.time()
                while time.time() - start_time < timeout:
                    angles_now = self.data_parser.get_joint_state(arm).angles
                    if all(abs(angles_now[i] - mapped_angles[i]) <= tolerance for i in range(self.joint_count)):
                        break
                    time.sleep(0.02)

                if time.time() - start_time >= timeout:
                    logger.warning(f"{arm} 等待到位超时")
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
            time.sleep(0.02)

        logger.warning(f"{arm} 夹爪运动完成超时")
        return False
    
    def set_zero_position(self, arm:str) -> bool:
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
                       
        """
        构建关节控制帧（支持单臂或双臂）

        Args:
            joint_angles: 
                - 若 arm 为 None，则应为 [[left_arm], [right_arm]]
                - 否则应为 1 个包含 7 个关节角度的列表
            arm: 指定控制的手臂，"left_arm"、"right_arm"，或 both（表示双臂）

        Returns:
            List[int]: 控制帧字节列表
        """

        # === 准备通用帧头结构 ===
        frame_size = self.ARM_DATA_SIZE + self.FRAME_MINIMAL_SIZE #[左右臂识别字节(1) + 舵机字节(20)] + 通用字节(5)
        frame = [0] * frame_size
        frame[0] = self.FRAME_HEADER
        frame[1] = self.CMD_DUAL_ARM
        frame[2] = self.ARM_DATA_SIZE
        frame[-1] = self.FRAME_FOOTER
        
        if arm == 'left_arm':
            frame[3] = self.LEFT_ARM
        elif arm == 'right_arm':
            frame[3] = self.RIGHT_ARM

        offset = 4      # 数据从第四位开始
        for servo_idx, (joint_idx, direction) in enumerate(self.joint_to_servo_map):
            angle_rad = joint_angles[joint_idx] * direction
            value = self._rad_to_hardware_value(angle_rad)

            frame[offset + servo_idx * 2] = value & 0xFF
            frame[offset + servo_idx * 2 + 1] = (value >> 8) & 0xFF

        frame[-2] = self._calculate_checksum(frame)

        # === 日志打印 ===
        if self.debug_mode:
            logger.debug(f"发送关节角度 (度): "
                        f"{arm}关节角度: {[round(a * self.RAD_TO_DEG, 1) for a in joint_angles]}")
   
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
    