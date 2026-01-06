# Copyright (c) 2025 Synria Robotics Co., Ltd.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
#
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai


import math
import time
from ..utils.logger import logger, hex_print
from typing import List, Dict, Tuple, Optional, Union, NamedTuple
import threading 
import copy

class JointState(NamedTuple):
    """关节状态数据结构"""
    angles: List[float]  # 关节角度(弧度)
    gripper: float       # 夹爪值(0-1000范围，与Alicia API一致)
    timestamp: float     # 时间戳(秒)

JointStateDict = Dict[str, JointState]

class DataParser:
    """机械臂数据解析模块"""
    
    # 常量定义
    DEG_TO_RAD = math.pi / 180.0  # 角度转弧度系数
    RAD_TO_DEG = 180.0 / math.pi  # 弧度转角度系数
    
    # 指令ID
    # CMD_ZERO_POS = 0x03    # 机械臂以当前位置为零点  
    # CMD_DUAL_ARM = 0x06   # 四机械臂角度反馈与控制
    # CMD_TORQUE = 0x13      # 机械臂力矩控制
    # CMD_ERROR = 0xEE       # 错误反馈
    # Command IDs
    CMD_GRIPPER = 0x04     # Gripper control and travel feedback
    CMD_ZERO_POS = 0x03    # Set current position as zero
    CMD_JOINT = 0x06       # Joint angle feedback and control
    CMD_VERSION = 0x01     # Firmware version feedback
    CMD_TORQUE = 0x05      # Torque control
    CMD_ERROR = 0xEE       # Error feedback
    CMD_SELF_CHECK = 0xFE  # Machine self-check (servo health)
    GRI_MAX_50MM = 3290
    GRI_MAX_100MM = 3600


    # Data size constants
    JOINT_DATA_SIZE = 33  # 关节数据: 右臂7关节(14B) + 左臂7关节(14B) + 右夹爪(2B) + 左夹爪(2B) + 运行状态(1B)
    PER_ARM_JOINT_BYTES = 14  # 7关节 * 2字节
    GRIPPER_PAIR_BYTES = 4   # 两个夹爪共 4 字节
    RUN_STATUS_BYTES = 1     # 运行状态 1 字节

    
    def __init__(self, lock: threading.Lock, debug_mode: bool = False, ):
        """
        初始化数据解析器
        
        Args:
            debug_mode: 是否启用调试模式
        """
        self.debug_mode = debug_mode
        
        # 存储最新数据   
        self._version_info: Optional[Dict[str, str]] = None


        self._joint_states = {"left": JointState([0.0]*7, 0.0, 0.0),
                              "right": JointState([0.0]*7, 0.0, 0.0)}
        self._lock = lock

        # Block order: first 14 bytes = right arm, next 14 bytes = left arm
        # Gripper: 4 bytes = right gripper (2B) + left gripper (2B)
        self._version_event = threading.Event()
        
        # Store run status from joint data
        self._run_status: Optional[int] = None
        self._run_status_text: Optional[str] = None
        
        # Store temperature data (in Celsius)
        self._temperature_data: Optional[List[float]] = None
        self._temperature_timestamp: Optional[float] = None
        
        # Store velocity data (in degrees per second)
        self._velocity_data: Optional[List[float]] = None
        self._velocity_timestamp: Optional[float] = None
        
        # Store gripper type data
        self._gripper_type: Optional[int] = None
        self._gripper_type_name_map = {
            0x00: "50mm",
            0x02: "100mm",
        }
        self._gripper_type_timestamp: Optional[float] = None
        
        # Store self-check (servo health) data
        self._self_check_raw_mask: Optional[int] = None
        self._self_check_bits: Optional[List[bool]] = None
        self._self_check_timestamp: Optional[float] = None
        
        # Event-based synchronization for async data acquisition
        self._joint_event = threading.Event()
        self._temperature_event = threading.Event()
        self._velocity_event = threading.Event()
        self._self_check_event = threading.Event()
        self._gripper_type_event = threading.Event()

        # Mapping from info type to corresponding event
        self._info_event_map = {
            "version": self._version_event,
            "joint_gripper": self._joint_event,
            "temperature": self._temperature_event,
            "velocity": self._velocity_event,
            "self_check": self._self_check_event,
            "gripper_type": self._gripper_type_event,
        }

    

    def parse_frame(self, frame: List[int]) -> Optional[Dict]:
        """
        Parse a full data frame.

        :param frame: Complete data frame (byte list)
        """
        cmd_id = frame[1]
        if cmd_id == self.CMD_VERSION:
            return self._parse_version_data(frame)
        elif cmd_id == self.CMD_JOINT:
            # Check function code to determine which parser to use
            func_code = frame[2]
            if func_code == 0x00:
                return self._parse_joint_data(frame)
            elif func_code == 0x01:
                return self._parse_temperature_data(frame)
            elif func_code == 0x02:
                return self._parse_velocity_data(frame)
            else:
                if self.debug_mode:
                    logger.debug(f"Unhandled function code in CMD_JOINT: 0x{func_code:02X}")
                return None
        elif cmd_id == self.CMD_GRIPPER:
            # Check function code to determine which parser to use
            func_code = frame[2]
            if func_code == 0x0E:
                return self._parse_gripper_type_data(frame)
            else:
                if self.debug_mode:
                    logger.debug(f"Unhandled function code in CMD_GRIPPER: 0x{func_code:02X}")
                return None
        elif cmd_id == self.CMD_ERROR:
            return self._parse_error_data(frame)
        elif cmd_id == self.CMD_SELF_CHECK:
            return self._parse_self_check_data(frame)
        else:
            if self.debug_mode:
                logger.debug(f"Unhandled command ID: 0x{cmd_id:02X}")
            return None

    
    def get_joint_state(self, arm: str = ''):
        with self._lock:
            for a in ['left','right']:
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


    def get_info(self, info_type: str):
        """
        Unified getter for parsed information, for cooperation with high-level APIs.

        :param info_type: 'joint_gripper' | 'joint' | 'gripper' | 'version' | 'temperature' | 'velocity' | 'self_check' | 'gripper_type'
        :return: Parsed data for the given type, or None if unavailable
        """
        with self._lock:
            if info_type == "joint_gripper":
                # Return both arms' joint states
                left_js = self._joint_states.get('left')
                right_js = self._joint_states.get('right')
                if left_js is None or right_js is None:
                    return None
                if left_js.angles is None or left_js.timestamp is None:
                    return None
                if right_js.angles is None or right_js.timestamp is None:
                    return None
                return copy.deepcopy(self._joint_states)
            elif info_type == "joint":
                # Return both arms' joint angles
                left_js = self._joint_states.get('left')
                right_js = self._joint_states.get('right')
                if left_js is None or right_js is None:
                    return None
                if left_js.angles is None or right_js.angles is None:
                    return None
                return {
                    'left': list(left_js.angles),
                    'right': list(right_js.angles)
                }
            elif info_type == "gripper":
                # Return both arms' gripper values
                left_js = self._joint_states.get('left')
                right_js = self._joint_states.get('right')
                if left_js is None or right_js is None:
                    return None
                return {
                    'left': left_js.gripper,
                    'right': right_js.gripper
                }
            elif info_type == "version":
                return dict(self._version_info) if self._version_info else None
            elif info_type == "temperature":
                return list(self._temperature_data) if self._temperature_data else None
            elif info_type == "velocity":
                return list(self._velocity_data) if self._velocity_data else None
            elif info_type == "self_check":
                if self._self_check_raw_mask is None or self._self_check_bits is None:
                    return None
                return {
                    "raw_mask": int(self._self_check_raw_mask),
                    "bits": list(self._self_check_bits),
                    "timestamp": self._self_check_timestamp,
                }
            elif info_type == "gripper_type":
                if self._gripper_type is None:
                    return None
                return self._gripper_type_name_map.get(
                    self._gripper_type,
                    f"unknown(0x{self._gripper_type:02X})",
                )
            else:
                raise ValueError(f"Unsupported info type: {info_type}")



    def _parse_version_data(self, frame: List[int]) -> Dict:
        """
        Parse version data frame (CMD=0x01).

        :param frame: Complete data frame
        """
        # Basic length check: header(1)+CMD(1)+func(1)+LEN(1)+DATA(LEN)+checksum(1)+footer(1)
        if len(frame) < 4 + frame[3] + 2:
            logger.warning(f"Version frame too short: expect ≥{4 + frame[3] + 2}, got {len(frame)}")
            return None
        data_len = frame[3]
        data_start = 4
        data_end = data_start + data_len
        data_bytes = frame[data_start:data_end]

        if data_len < 24:
            logger.warning(f"Version data length too short: expect 24, got {data_len}")
            return None

        # Split fields according to protocol
        serial_bytes = data_bytes[0:16]
        hardware_bytes = data_bytes[16:20]
        firmware_bytes = data_bytes[20:24]

        def _bytes_to_ascii(b: List[int]) -> str:
            try:
                return "".join(chr(x) for x in b).strip()
            except Exception as e:
                logger.error(f"Version ASCII parse exception: {e}")
                return ""

        def _bytes_to_decimal(b: List[int]) -> int:
            """Convert little-endian byte array to decimal integer."""
            result = 0
            for i, byte in enumerate(b):
                result |= (byte & 0xFF) << (i * 8)
            return result

        # Parse serial number as ASCII string
        serial_number = _bytes_to_ascii(serial_bytes)
        # Parse hardware and firmware versions as little-endian decimal values
        hardware_decimal = _bytes_to_decimal(hardware_bytes)
        firmware_decimal = _bytes_to_decimal(firmware_bytes)

        # Convert decimal values to version strings
        hardware_str = self._decimal_to_version_string(hardware_decimal)
        firmware_str = self._decimal_to_version_string(firmware_decimal)
        # Store firmware version (for upper-level API)
        with self._lock:
            self._firmware_version = firmware_str
            self._version_info = {
                "serial_number": serial_number,
                "hardware_version": hardware_str,
                "firmware_version": firmware_str,
            }

        # Signal that version info has been received and parsed
        self._version_event.set()

        if self.debug_mode:
            logger.debug(
                f"Version parsed: SN='{serial_number}', HW={hardware_decimal}('{hardware_str}'), FW={firmware_decimal}('{firmware_str}')"
            )

        return {
            "type": "version_data",
            "serial_number": serial_number,
            "hardware_version": hardware_str,
            "firmware_version_raw": firmware_decimal,
            "version": firmware_str,
            "timestamp": time.time(),
        }



    def _parse_joint_data(self, frame: List[int]) -> Dict:
        """
        Parse joint data frame (CMD=0x06, FUNC=0x00).
        
        Frame structure: 0xAA CMD FUNC LEN DATA CHECK 0xFF
        Data format: 右臂7关节(14B) + 左臂7关节(14B) + 右夹爪(2B) + 左夹爪(2B) + 运行状态(1B) = 33B
        Joint value: 2 bytes little-endian (0-4095 maps to -180~+180°)
        
        :param frame: Complete data frame
        :return: Parsed joint data dictionary
        """
        try:
            # Validate frame structure
            if not self._validate_joint_frame(frame):
                return None
            
            data_start = 4
            data_len = frame[3]
            
            # Extract data blocks
            right_arm_block = frame[data_start : data_start + self.PER_ARM_JOINT_BYTES]
            # hex_print(logger, "right_arm_block", right_arm_block)
            left_arm_block = frame[data_start + self.PER_ARM_JOINT_BYTES : data_start + 2 * self.PER_ARM_JOINT_BYTES]
            # hex_print(logger, "left_arm_block", left_arm_block)
            gripper_block = frame[data_start + 2 * self.PER_ARM_JOINT_BYTES : data_start + 2 * self.PER_ARM_JOINT_BYTES + self.GRIPPER_PAIR_BYTES]
            # hex_print(logger, "gripper_block", gripper_block)
            run_status_byte = frame[data_start + 2 * self.PER_ARM_JOINT_BYTES + self.GRIPPER_PAIR_BYTES] if data_len >= self.JOINT_DATA_SIZE else None
            
            # Decode joint angles
            right_angles = self._decode_joint_block(right_arm_block, "right")
            left_angles = self._decode_joint_block(left_arm_block, "left")
            
            if right_angles is None or left_angles is None:
                return None
            
            # Decode grippers (returns raw values 0-1000, same as Alicia API)
            right_gripper, left_gripper = self._decode_gripper_block(gripper_block)
            
            # Update joint states
            self._update_joint_state("right", right_angles, right_gripper)
            self._update_joint_state("left", left_angles, left_gripper)
            
            # Parse run status
            run_status, run_status_text = self._decode_run_status(run_status_byte)
            if run_status is not None:
                with self._lock:
                    self._run_status = run_status
                    self._run_status_text = run_status_text
            
            # Signal that joint state has been updated
            self._joint_event.set()
            
            return {
                "type": "dual_arm_joint_data",
                "timestamp_left": self._joint_states['left'].timestamp,
                "timestamp_right": self._joint_states['right'].timestamp,
                "left_angle": self._joint_states['left'].angles,
                "right_angle": self._joint_states['right'].angles,
                "left_gripper": self._joint_states['left'].gripper,
                "right_gripper": self._joint_states['right'].gripper,
                "run_status": run_status,
                "run_status_text": run_status_text,
            }
        except Exception as e:
            logger.error(f"解析关节数据异常: {e}")
            return None

    def _validate_joint_frame(self, frame: List[int]) -> bool:
        """Validate joint data frame structure."""
        if len(frame) < 4:
            logger.warning("关节数据帧长度不足")
            return False
        
        func_code = frame[2]
        if func_code != 0x00:
            logger.warning(f"关节数据功能码错误: 期望0x00, 得到0x{func_code:02X}")
            return False
        
        data_len = frame[3]
        expected_min_len = 4 + data_len + 2  # header+cmd+func+len + data + checksum+footer
        if len(frame) < expected_min_len:
            logger.warning(f"关节数据帧长度不匹配: LEN={data_len}, frame_len={len(frame)}")
            return False
        
        if data_len != self.JOINT_DATA_SIZE:
            logger.warning(f"不支持的数据长度: 期望{self.JOINT_DATA_SIZE}, 得到{data_len}")
            return False
        
        return True

    def _decode_joint_block(self, block: List[int], arm: str) -> Optional[List[float]]:
        """Decode joint angle block (14 bytes for 7 joints)."""
        if len(block) != self.PER_ARM_JOINT_BYTES:
            logger.warning(f"{arm}关节数据块长度异常: 期望{self.PER_ARM_JOINT_BYTES}, 得到{len(block)}")
            return None
        
        angles = []
        for i in range(7):
            lo = block[2 * i] & 0xFF
            hi = block[2 * i + 1] & 0xFF
            raw_value = lo | (hi << 8)
            # Range check
            if raw_value < 0 or raw_value > 4095:
                logger.warning(f"{arm}关节{i+1}值超出范围: {raw_value} (有效范围0-4095)")
                raw_value = max(0, min(raw_value, 4095))
            
            # Directly map raw value to radians: 0–4095 -> [-π, π] 
            angle_rad = (raw_value / 4096.0) * (2 * math.pi) - math.pi
            angles.append(angle_rad)
        
        return angles

    def _decode_gripper_block(self, gripper_block: List[int]) -> Tuple[float, float]:
        """Decode gripper block (4 bytes: right gripper 2B + left gripper 2B).
        
        Returns raw gripper values in 0-1000 range.
        No conversion to radians or degrees - use raw value directly.
        """
        if len(gripper_block) != self.GRIPPER_PAIR_BYTES:
            if self.debug_mode:
                logger.warning(f"夹爪数据长度异常: 期望{self.GRIPPER_PAIR_BYTES}, 得到{len(gripper_block)}")
            return 0.0, 0.0
        
        try:
            # Read little-endian 16-bit values 
            right_raw = (gripper_block[0] & 0xFF) | ((gripper_block[1] & 0xFF) << 8)
            left_raw = (gripper_block[2] & 0xFF) | ((gripper_block[3] & 0xFF) << 8)
            
            # Map raw gripper value to 0-1000 range
            # Hardware uses 0-1000 range, return directly without conversion
            right_gripper = float(max(0.0, min(1000.0, right_raw)))
            left_gripper = float(max(0.0, min(1000.0, left_raw)))
            
            return right_gripper, left_gripper
        except Exception as e:
            if self.debug_mode:
                logger.warning(f"夹爪解析失败: {e}")
            return 0.0, 0.0

    def _decode_run_status(self, status_byte: Optional[int]) -> Tuple[Optional[int], str]:
        """Decode run status byte."""
        if status_byte is None:
            return None, "unknown"
        
        run_status_map = {
            0x00: "idle",
            0x01: "locked",
            0x10: "sync",
            0x11: "sync_locked",
            0xE1: "overheat",
            0xE2: "overheat_protect",
        }
        
        status_text = run_status_map.get(status_byte, "unknown")
        return status_byte, status_text


    def _value_to_radians(self, value: int) -> float:
        """
        将舵机值转换为弧度值
        
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


    def _decimal_to_version_string(self, decimal_value: int) -> str:
        """
        Convert decimal value to version string.
        :param decimal_value: Decimal version value
        :return: Version string in format "X.Y.Z" (MAJOR.MINOR.PATCH)
        """
        if decimal_value < 0:
            return "unknown"
        
        decimal_str = str(decimal_value)
        
        if len(decimal_str) == 1:
            version_str = f"0.0.{decimal_str}"
        elif len(decimal_str) == 2:
            version_str = f"{decimal_str[0]}.{decimal_str[1]}.0"
        elif len(decimal_str) == 3:
            version_str = f"{decimal_str[0]}.{decimal_str[1]}.{decimal_str[2]}"
        else:
            version_str = f"{decimal_str[0]}.{decimal_str[1]}.{decimal_str[2:]}"
        
        return version_str

    def _parse_temperature_data(self, frame: List[int]) -> Dict:
        """
        Parse temperature data frame (CMD=0x06, FUNC=0x01).

        :param frame: Complete data frame
        """
        data_len = frame[3]
        expected_min_len = 4 + data_len + 2
        if len(frame) < expected_min_len:
            logger.warning(f"Temperature frame length mismatch: LEN={data_len}, frame_len={len(frame)}")
            return None

        data_start = 4
        data_end = data_start + data_len
        data_bytes = frame[data_start:data_end]

        # Parse temperature values (each byte represents temperature in Celsius)
        temperatures = [float(byte) for byte in data_bytes]

        # Store temperature data
        with self._lock:
            self._temperature_data = temperatures
            self._temperature_timestamp = time.time()

        # Signal that temperature data has been updated
        self._temperature_event.set()

        if self.debug_mode:
            logger.debug(f"Temperature data: {temperatures}°C")

        return {
            "type": "temperature_data",
            "temperatures": temperatures,
            "timestamp": self._temperature_timestamp,
        }

    def _parse_velocity_data(self, frame: List[int]) -> Dict:
        """
        Parse velocity data frame (CMD=0x06, FUNC=0x02).

        :param frame: Complete data frame
        """
        data_len = frame[3]
        expected_min_len = 4 + data_len + 2
        if len(frame) < expected_min_len:
            logger.warning(f"Velocity frame length mismatch: LEN={data_len}, frame_len={len(frame)}")
            return None

        data_start = 4
        data_end = data_start + data_len
        data_bytes = frame[data_start:data_end]

        # Parse velocity values (2 bytes per servo, low byte first)
        num_servos = data_len // 2
        velocities = []
        for i in range(num_servos):
            low_byte = data_bytes[i * 2]
            high_byte = data_bytes[i * 2 + 1]
            velocity_raw = (low_byte & 0xFF) | ((high_byte & 0xFF) << 8)
            # Convert raw velocity to degrees per second
            # Note: velocity_raw can exceed the expected limit of 5000
            velocity_deg_s = self._raw_velocity_to_deg_per_sec(velocity_raw)
            velocities.append(velocity_deg_s)

        # Store velocity data
        with self._lock:
            self._velocity_data = velocities
            self._velocity_timestamp = time.time()

        # Signal that velocity data has been updated
        self._velocity_event.set()

        if self.debug_mode:
            logger.debug(f"Velocity data (deg/s): {velocities}")

        return {
            "type": "velocity_data",
            "velocities": velocities,
            "timestamp": self._velocity_timestamp,
        }

    def _raw_velocity_to_deg_per_sec(self, velocity_raw: int) -> float:
        """
        Convert raw velocity value to degrees per second.
        
        :param velocity_raw: Raw velocity value from hardware
        :return: Velocity in degrees per second
        """
        # This is a placeholder - adjust based on actual hardware specification
        # Assuming raw value is in some unit that needs conversion
        return float(velocity_raw) * 0.1  # Example conversion, adjust as needed

    def _parse_self_check_data(self, frame: List[int]) -> Dict:
        """
        Parse machine self-check frame (CMD=0xFE, FUNC=0x00).

        :param frame: Complete data frame
        """
        data_len = frame[3]
        expected_min_len = 4 + data_len + 2
        if len(frame) < expected_min_len:
            logger.warning(f"Self-check frame length mismatch: LEN={data_len}, frame_len={len(frame)}")
            return None

        data_start = 4
        data_end = data_start + data_len
        data_bytes = frame[data_start:data_end]

        if data_len < 2:
            logger.warning(f"Self-check DATA too short: expect ≥2 bytes, got {data_len}")
            return None

        low = data_bytes[0]
        high = data_bytes[1]
        raw_mask = (low & 0xFF) | ((high & 0xFF) << 8)
        # Decode to boolean list (LSB first), up to 16 bits to be safe
        bits: List[bool] = [(raw_mask >> i) & 0x1 == 1 for i in range(10)]

        with self._lock:
            self._self_check_raw_mask = raw_mask
            self._self_check_bits = bits
            self._self_check_timestamp = time.time()

        # Signal that self-check data has been updated
        self._self_check_event.set()

        if self.debug_mode:
            logger.debug(
                f"Self-check result: raw_mask=0x{raw_mask:04X}, "
                f"bits={bits}"
            )

        return {
            "type": "self_check_data",
            "raw_mask": raw_mask,
            "bits": bits,
            "timestamp": self._self_check_timestamp,
        }

    def _parse_gripper_type_data(self, frame: List[int]) -> Dict:
        """
        Parse gripper type data frame (CMD=0x04, FUNC=0x0E).

        :param frame: Complete data frame
        """
        data_len = frame[3]
        expected_min_len = 4 + data_len + 2
        if len(frame) < expected_min_len:
            logger.warning(f"Gripper type frame length mismatch: LEN={data_len}, frame_len={len(frame)}")
            return None

        data_start = 4
        data_end = data_start + data_len
        data_bytes = frame[data_start:data_end]

        if data_len < 1:
            logger.warning(f"Gripper type DATA too short: expect ≥1 byte, got {data_len}")
            return None

        gripper_type_raw = data_bytes[0] & 0xFF

        # Map gripper type values
        type_name = self._gripper_type_name_map.get(
            gripper_type_raw,
            f"unknown(0x{gripper_type_raw:02X})",
        )

        # Store gripper type data
        with self._lock:
            self._gripper_type = gripper_type_raw
            self._gripper_type_timestamp = time.time()

        # Signal that gripper type data has been updated
        self._gripper_type_event.set()

        if self.debug_mode:
            logger.debug(f"Gripper type: {type_name} (0x{gripper_type_raw:02X})")

        return {
            "type": "gripper_type_data",
            "gripper_type": gripper_type_raw,
            "type_name": type_name,
            "timestamp": self._gripper_type_timestamp,
        }
