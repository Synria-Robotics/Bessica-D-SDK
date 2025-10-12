import math
import time
import logging
from typing import List, Dict, Tuple, Optional, Union, NamedTuple
import threading 
import copy

# 配置日志
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("DataParser")

class JointState(NamedTuple):
    """关节状态数据结构"""
    angles: List[float]  # 六个关节角度(弧度)
    gripper: float       # 夹爪角度(弧度)
    timestamp: float     # 时间戳(秒)
    # button1: bool        # 按钮1状态
    # button2: bool        # 按钮2状态

JointStateDict = Dict[str, JointState]

class DataParser:
    """机械臂数据解析模块"""
    
    # 常量定义
    DEG_TO_RAD = math.pi / 180.0  # 角度转弧度系数
    RAD_TO_DEG = 180.0 / math.pi  # 弧度转角度系数
    
    # 指令ID
    CMD_ZERO_POS = 0x03    # 机械臂以当前位置为零点  
    CMD_DUAL_ARM = 0x06   # 四机械臂角度反馈与控制
    CMD_TORQUE = 0x13      # 机械臂力矩控制
    CMD_ERROR = 0xEE       # 错误反馈

    # 数据长度（兼容旧/新协议）
    JOINT_DATA_SIZE_V1 = 28  # 旧: 仅关节 (右/左 或 左/右 各 14B)
    JOINT_DATA_SIZE_V2 = 32  # 新: 关节(14+14) + 夹爪(2+2)
    GRIPPER_PAIR_BYTES = 4   # 新协议中两个夹爪共 4 字节

    # 识别帧
    PRESENT_POSITION = 0x38 #当前机械臂关节角度识别帧
    # PRESENT_SPEED = 0x41    #当前机械臂关节速度识别帧 （待开发）

    
    def __init__(self, lock: threading.Lock, debug_mode: bool = False, ):
        """
        初始化数据解析器
        
        Args:
            debug_mode: 是否启用调试模式
        """
        self.debug_mode = debug_mode
        
        # 存储最新数据   
        self._joint_states = {"left_arm": JointState([0.0]*7, 0.0, 0.0),
                              "right_arm": JointState([0.0]*7, 0.0, 0.0)}
        self._lock = lock
        self.direction_map = {
            "left_arm":  [1, 1, 1, 1, 1, 1, 1],
            "right_arm": [1, 1, 1, 1, 1, 1, 1]
        }
        # 块顺序配置 (仅用于关节 2*14 字节部分)。
        # 新协议: 前14字节=右臂, 后14字节=左臂, 最后4字节=右夹爪2+左夹爪2
        self.block_order = ("right_arm", "left_arm")

        logger.info("初始化数据解析模块")
        if self.debug_mode:
            logger.info("调试模式: 启用")
    
    def parse_frame(self, frame: List[int]) -> Optional[Dict]:
        """
        解析数据帧
        
        Args:
            frame: 完整的数据帧(字节列表)
            
        Returns:
            Dict: 解析结果，如果解析失败则返回None
        """
        # 基本帧格式检查
        if len(frame) < 5 or frame[0] != 0xAA or frame[-1] != 0xFF:
            if self.debug_mode:
                logger.warning(f"无效数据帧: {self._bytes_to_hex(frame)}")
            return None
        
        # 解析指令ID和数据长度
        cmd_id = frame[1]
        data_len = frame[2]
        
        # 验证数据长度
        # 兼容两种格式：
        # A) 旧格式：LEN=DATA，仅数据，不含参数(ident) → 总长应为 LEN + 6（含 头/指令/长度/参数/校验/尾）
        # B) 新格式：LEN=参数+数据 → 总长应为 LEN + 5（含 头/指令/长度/校验/尾）
        total_len = len(frame)
        ok_len = (total_len == data_len + 6) or (total_len == data_len + 5)
        if not ok_len:
            if self.debug_mode:
                logger.warning(
                    f"数据长度不匹配: LEN=0x{data_len:02X} 实际总长={total_len}, 期望之一=[LEN+6={data_len+6}, LEN+5={data_len+5}]"
                )
            return None
        
        # 校验和验证
        if not self._verify_checksum(frame):
            if self.debug_mode:
                logger.warning(f"校验和错误: {self._bytes_to_hex(frame)}")
            return None
        
        # 根据指令ID解析数据
        if cmd_id == self.CMD_DUAL_ARM:
            return self._parse_joint_data(frame)
        elif cmd_id == self.CMD_ERROR:
            return self._parse_error_data(frame)
        else:
            if self.debug_mode:
                logger.debug(f"未处理的指令ID: 0x{cmd_id:02X}")
            return None
    
    def get_joint_state(self, arm: str = 'both'):
        with self._lock:
            for a in ['left_arm','right_arm']:
                js = self._joint_states[a]
                if js.angles is None or js.timestamp is None:
                    logger.warning(f"{arm} 的状态尚未更新")
                    return None

            if arm == 'both':
                return copy.deepcopy(self._joint_states)
            return copy.deepcopy(self._joint_states[arm])
    
    def _update_joint_state(self, arm: str, angles: List[float], gripper: float):
        with self._lock:
            self._joint_states[arm] = JointState(angles, gripper, time.time())

    def _parse_joint_data(self, frame: List[int]) -> Dict:
        """
        解析关节数据帧 (CMD_DUAL_ARM)
        新协议:
          Frame = 0xAA CMD LEN IDENT DATA CHECK 0xFF
          IDENT = PRESENT_POSITION (0x38)
          LEN = 28 字节: left_arm(7关节*2B) + right_arm(7关节*2B)
          单个关节值: 2 字节 little-endian (0-4095 对应 -180~+180° 映射)
          不包含夹爪数据，gripper 固定 0
        返回统一结构 dual_arm_joint_data
        """
        try:
            if frame[3] != self.PRESENT_POSITION:
                logger.warning(f"关节数据类型错误: ident=0x{frame[3]:02X}")
                return None
            data_len = frame[2]
            # 规范化有效载荷长度（不含 ident/参数）
            if data_len in (self.JOINT_DATA_SIZE_V1, self.JOINT_DATA_SIZE_V2):
                payload_len = data_len  # 旧格式：LEN=DATA
            elif (data_len - 1) in (self.JOINT_DATA_SIZE_V1, self.JOINT_DATA_SIZE_V2):
                payload_len = data_len - 1  # 新格式：LEN=IDENT+DATA
            else:
                logger.warning(f"不支持的数据长度: 0x{data_len:02X}")
                return None

            data_start = 4  # DATA 起始
            per_arm_bytes = 14  # 7 关节 * 2B
            block1 = frame[data_start : data_start + per_arm_bytes]
            block2 = frame[data_start + per_arm_bytes : data_start + 2 * per_arm_bytes]
            gripper_block = None
            if payload_len == self.JOINT_DATA_SIZE_V2:
                gripper_block = frame[data_start + 2 * per_arm_bytes : data_start + 2 * per_arm_bytes + self.GRIPPER_PAIR_BYTES]

            if len(block1) != per_arm_bytes or len(block2) != per_arm_bytes:
                logger.warning("关节数据分块长度异常")
                return None
            if gripper_block is not None and len(gripper_block) != self.GRIPPER_PAIR_BYTES:
                logger.warning("夹爪数据长度异常")
                gripper_block = None  # 继续解析关节

            def check_block(block):
                if len(block) < 2:
                    return False
                v = block[0] | (block[1] << 8)
                return v <= 4096

            if not check_block(block1) or not check_block(block2):
                return None

            def decode(block: List[int]) -> List[float]:
                angles = []
                for i in range(7):
                    lo = block[2 * i]
                    hi = block[2 * i + 1]
                    raw = (lo & 0xFF) | ((hi & 0xFF) << 8)
                    angles.append(self._value_to_radians(raw))
                return angles

            arm1, arm2 = self.block_order  # 新协议默认 (right_arm, left_arm)
            a1_raw = decode(block1)
            a2_raw = decode(block2)
            a1_map = [a * self.direction_map[arm1][i] for i, a in enumerate(a1_raw)]
            a2_map = [a * self.direction_map[arm2][i] for i, a in enumerate(a2_raw)]
            self._update_joint_state(arm1, a1_map, 0.0)
            self._update_joint_state(arm2, a2_map, 0.0)

            # 解析夹爪 (新协议末尾 4 字节: 右夹爪(2B) + 左夹爪(2B)，同小端)
            if gripper_block:
                try:
                    right_val = gripper_block[0] | (gripper_block[1] << 8)
                    left_val  = gripper_block[2] | (gripper_block[3] << 8)
                    # 转换为 0~100 度 (按 controller 中 _rad_to_hardware_value_grip 的反向推导)
                    servo_min = 2048
                    servo_max = 3590
                    def grip_to_rad(v):
                        if v < servo_min: v = servo_min
                        if v > servo_max: v = servo_max
                        deg = (v - servo_min) / (servo_max - servo_min) * 100.0
                        return deg * self.DEG_TO_RAD
                    right_grip = grip_to_rad(right_val)
                    left_grip  = grip_to_rad(left_val)
                    # 更新（保持之前已写入的角度）
                    self._update_joint_state(arm1, self._joint_states[arm1].angles, right_grip if arm1 == 'right_arm' else left_grip)
                    self._update_joint_state(arm2, self._joint_states[arm2].angles, left_grip if arm2 == 'left_arm' else right_grip)
                except Exception as e:
                    if self.debug_mode:
                        logger.warning(f"夹爪解析失败: {e}")

            return {
                "type": "dual_arm_joint_data",
                "timestamp_left": self._joint_states['left_arm'].timestamp,
                "timestamp_right": self._joint_states['right_arm'].timestamp,
                "left_arm_angle": self._joint_states['left_arm'].angles,
                "right_arm_angle": self._joint_states['right_arm'].angles,
                "left_gripper": self._joint_states['left_arm'].gripper,
                "right_gripper": self._joint_states['right_arm'].gripper,
                "protocol_version": 2 if payload_len == self.JOINT_DATA_SIZE_V2 else 1
            }
        except Exception as e:
            logger.error(f"解析关节数据异常: {e}")
            return None


    def _value_to_radians(self, value: int) -> float:
        """
        将舵机值转换为弧度值 - 与ROS代码保持一致
        
        Args:
            value: 舵机值(0-4095)
            
        Returns:
            float: 弧度值
        """
        try:
            # 值范围检查
            if value < 0 or value > 4095:
                #logger.warning(f"舵机值超出范围: {value} (有效范围0-4095)")
                #value = max(0, min(value, 4095))
                return 0

            # 转换为角度: -180到+180度
            # 使用与ROS代码一致的转换公式
            angle_deg = -180.0 + (value / 2048.0) * 180.0
            
            # 转换为弧度并返回
            return angle_deg * self.DEG_TO_RAD
                
        except Exception as e:
            logger.error(f"值转换异常: {str(e)}")
            return 0.0
    
    
    def _parse_error_data(self, frame: List[int]) -> Dict:
        """
        解析错误数据帧 (0xEE)
        
        Args:
            frame: 完整的数据帧
            
        Returns:
            Dict: 解析结果
        """
        # 检查最小长度
        if len(frame) < 7:
            logger.warning("错误数据帧长度不足")
            return None
        
        # 提取错误码和附加信息
        error_code = frame[3]
        error_param = frame[4]
        
        error_types = {
            0x00: "包头/包尾或长度错误",
            0x01: "校验错误",
            0x02: "模式错误",
            0x03: "ID无效",
        }
        
        error_message = error_types.get(error_code, f"未知错误(0x{error_code:02X})")
        
        logger.warning(f"设备错误: {error_message}, 参数: 0x{error_param:02X}")
        
        return {
            "type": "error_data",
            "error_code": error_code,
            "error_param": error_param,
            "error_message": error_message,
            "timestamp": time.time()
        }
    
    def _bytes_to_radians(self, byte_array: List[int]) -> float:
        """
        将字节数组转换为弧度值
        
        Args:
            byte_array: 2字节数组
            
        Returns:
            float: 弧度值
        """
        try:
            if len(byte_array) != 2:
                logger.warning(f"数据长度错误：需要2个字节，实际{len(byte_array)}个字节")
                return 0.0
            
            # 构造16位整数
            hex_value = (byte_array[0] & 0xFF) | ((byte_array[1] & 0xFF) << 8)
            
            # 值范围检查
            if hex_value < 0 or hex_value > 4095:
                logger.warning(f"舵机值超出范围: {hex_value} (有效范围0-4095)")
                hex_value = max(0, min(hex_value, 4095))
            
            # 转换为角度: -180到+180度
            angle_deg = -180.0 + (hex_value / 2048.0) * 180.0
            
            # 转换为弧度并返回
            return angle_deg * self.DEG_TO_RAD
                
        except Exception as e:
            logger.error(f"字节转换异常: {str(e)}")
            return 0.0
    
    def _verify_checksum(self, frame: List[int]) -> bool:
        """
        验证帧的校验和
        
        Args:
            frame: 完整的数据帧
            
        Returns:
            bool: 校验是否通过
        """
        if len(frame) < 4:
            return False
        
        # 计算从第3个字节到倒数第3个字节的所有元素之和
        checksum = 0
        for i in range(3, len(frame) - 2):
            checksum += frame[i]
        
        checksum %= 2
        received_checksum = frame[-2]
        
        return checksum == received_checksum
    
    def _bytes_to_hex(self, data: List[int]) -> str:
        """
        将字节数组转换为十六进制字符串
        
        Args:
            data: 字节数组
            
        Returns:
            str: 十六进制字符串
        """
        return " ".join([f"{b:02X}" for b in data])
