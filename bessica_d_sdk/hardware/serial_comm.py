import serial
import platform
import serial.tools.list_ports
import time
from ..utils.logger import logger
import os
from typing import List, Optional, Tuple
import threading
from datetime import datetime
import json

DEFAULT_LENGTH = 6   # 头(1)+指令(1)+长度(1)+识别(1)+校验(1)+尾(1)
FRAME_LENGTH = 34
FRAME_HEADER = 0xAA
FRAME_TAIL = 0xFF
    
class SerialComm:
    """机械臂串口通信模块 - 简化版"""
    
    def __init__(self, lock: threading.Lock, port: str = "", baudrate: int = 1000000, 
                timeout: float = 1.0, debug_mode: bool = False):
        """
        初始化串口通信模块
        
        Args:
            port: 串口名称，留空则自动搜索
            baudrate: 波特率
            timeout: 超时时间(秒)
            debug_mode: 是否启用调试模式
        """
        self.port_name = port
        self.baudrate = baudrate
        self.baudrate_default = 921600
        self.baudrate_macOS = 1000000
        self.timeout = timeout
        self.debug_mode = debug_mode
        
        self.serial_port = None
        self.last_log_time = 0
        self._last_print_time = 0

        self._lock = lock
        self._rx_buffer = bytearray()
        self._frame_fail_count = 0  # 新增: 帧校验失败计数器
        # 最小发送间隔(秒)，要求>=2ms
        self._min_send_interval = 0.002
        self._last_send_time = 0.0  # perf_counter 时间戳

        logger.info(f"初始化串口通信模块: 端口={port or '自动'}, 波特率={baudrate}")
        logger.info(f"调试模式: {'启用' if debug_mode else '禁用'}")
    
    def __del__(self):
        """析构函数，确保关闭串口"""
        self.disconnect()
    
    def connect(self) -> bool:
        """
        连接到串口设备
        
        Returns:
            bool: 连接是否成功
        """
        try:
            # 查找可用串口
            port = self.find_serial_port()

            # 没有找到可用串口
            if not port:
                logger.warning("未找到可用串口")
                return False

            logger.info(f"正在连接端口: {port}")

            # 关闭已有连接
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()

            if '/dev/tty.' in port:
                cu_candidate = port.replace('/dev/tty.', '/dev/cu.')
                if os.path.exists(cu_candidate) and os.access(cu_candidate, os.R_OK | os.W_OK):
                    port = cu_candidate

            # Serial parameters: add write timeout and disable flow control
            self.serial_port = serial.Serial(
                port=port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            try:
                # Ensure buffers and handshake lines are in a sane state
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
                try:
                    # Keep DTR asserted; some controllers ignore TX when DTR is low
                    self.serial_port.setDTR(True)
                except Exception:
                    pass
                try:
                    self.serial_port.setRTS(False)
                except Exception:
                    pass
            except Exception:
                pass

            if self.serial_port.is_open:
                logger.info("串口连接成功")
                return True

            return False

        except Exception as e:
            logger.error(f"连接串口异常: {str(e)}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            logger.info("串口已关闭")
    
    def find_serial_port(self) -> str:
        """
        查找可用的串口设备
        
        Returns:
            str: 可用串口的路径，未找到则返回空字符串
        """
        # 如果指定了端口，优先使用
        if self.port_name:
            device = self._normalize_port_name(self.port_name)
            if self._is_port_accessible(device):
                return device
        
        # 获取所有可用端口
        try:
            ports = list(serial.tools.list_ports.comports())
        except Exception:
            return ""
        
        if not ports:
            return ""
        
        # 按平台优先级查找
        priorities = self._get_port_priorities()
        for prefix in priorities:
            for port in ports:
                if prefix in port.device:
                    device = self._normalize_port_name(port.device)
                    if self._is_port_accessible(device):
                        return device
        
        # macOS: 尝试将 tty.* 映射到 cu.*
        if platform.system() == "Darwin":
            for port in ports:
                if port.device.startswith('/dev/tty.'):
                    cu_port = port.device.replace('/dev/tty.', '/dev/cu.')
                    if self._is_port_accessible(cu_port):
                        return cu_port
        
        return ""
    
    def _normalize_port_name(self, port_name: str) -> str:
        """标准化端口名称（Windows COM端口处理）"""
        if platform.system() == "Windows" and port_name.startswith("COM"):
            try:
                port_num = int(port_name[3:])
                if port_num > 9 and not port_name.startswith("\\\\.\\"):
                    return f"\\\\.\\{port_name}"
            except ValueError:
                pass
        return port_name
    
    def _is_port_accessible(self, port_name: str) -> bool:
        """检查端口是否可访问"""
        if platform.system() == "Windows" and port_name.startswith(("COM", "\\\\.\\COM")):
            return True
        return os.path.exists(port_name) and os.access(port_name, os.R_OK | os.W_OK)
    
    def _get_port_priorities(self) -> List[str]:
        """根据平台返回端口优先级列表"""
        system = platform.system()
        if system == "Darwin":  # macOS
            return ["cu.wchusbserial", "cu.SLAB_USBtoUART", "cu.usbserial", "cu.usbmodem", "ttyUSB", "COM"]
        elif system == "Linux":  # Linux
            return ["ttyUSB", "ttyACM", "ttyCH343USB", "cu.wchusbserial", "cu.SLAB_USBtoUART", "cu.usbserial", "cu.usbmodem", "COM"]
        else:  # Windows
            return ["COM", "ttyUSB", "cu.usbserial", "cu.usbmodem"]
    
    def send_data(self, data: List[int]) -> bool:
        """
        发送数据到串口
        
        Args:
            data: 要发送的字节数据列表
            
        Returns:
            bool: 是否发送成功
        """
        with self._lock:
            try:
                if not self.serial_port or not self.serial_port.is_open:
                    logger.warning("串口未打开，尝试重新连接")
                    if not self.connect():
                        logger.error("无法连接到串口")
                        return False
                
                # 节流：保证两帧发送间隔 >= _min_send_interval
                if self._last_send_time > 0:
                    now = time.perf_counter()
                    gap = now - self._last_send_time
                    if gap < self._min_send_interval:
                        time.sleep(self._min_send_interval - gap)
                # 转换为字节数组
                data_bytes = bytes(data)
                
                # 写入数据
                bytes_written = self.serial_port.write(data_bytes)
                if bytes_written != len(data):
                    logger.warning(f"只写入了 {bytes_written} 字节，应为 {len(data)} 字节")
                    return False
                
                # 记录时间戳
                self._last_send_time = time.perf_counter()
                # 无条件打印发送帧
               # self._print_hex_frame(data, 0)

                return True
                    
            except Exception as e:
                logger.error(f"发送数据时异常: {str(e)}")
                return False
    

    def read_frame(self) -> Optional[List[int]]:
        """
        按新协议解码一帧:
        Frame = 0xAA | CMD | LEN | IDENT | DATA[LEN] | CHECK | 0xFF
        CHECK = (IDENT + sum(DATA)) % 2
        返回:
          成功: List[int]
          暂无: None
          异常: None
        """
        try:
            if not self.serial_port or not self.serial_port.is_open:
                if not self.connect():
                    return None

            if self.serial_port.in_waiting == 0:
                return None

            # 读入全部等待字节
            self._rx_buffer += self.serial_port.read(self.serial_port.in_waiting)

            # 循环尝试解帧（支持粘包）
            while True:
                # 至少需要 4 字节才能读到 LEN (AA CMD LEN IDENT)
                if len(self._rx_buffer) < 4:
                    break

                # 同步帧头
                if self._rx_buffer[0] != FRAME_HEADER:
                    self._rx_buffer.pop(0)
                    continue

                # 读取长度字段（数据区长度）
                data_len = self._rx_buffer[2]
                total_len = data_len + DEFAULT_LENGTH  # 动态帧总长

                # 数据还不完整，等待更多字节
                if len(self._rx_buffer) < total_len:
                    break

                candidate = self._rx_buffer[:total_len]

                # 基本尾字节检查
                tail_ok = candidate[-1] == FRAME_TAIL
                valid = tail_ok and self._serial_data_check(candidate)

                if self.debug_mode:
                    now = time.time()
                    if now - self._last_print_time > 1.0:
                        print(f"[Frame] {list(candidate)} {'(OK)' if valid else '(Invalid)'}")
                        self._last_print_time = now

                # 无论是否有效，都弹出本帧长度，避免卡死
                self._rx_buffer = self._rx_buffer[total_len:]

                if valid:
                    # 用户当前需求: 仅打印发送数据, 不打印接收帧
                    # 若后续需要调试接收, 可临时解除下面注释
                    # 记录时间戳
                    # self._last_send_time = time.perf_counter()
                    # self._print_hex_frame(list(candidate), 1)
                    return list(candidate)
                else:
                    if not hasattr(self, '_frame_fail_count'):
                        self._frame_fail_count = 0
                    self._frame_fail_count += 1
                    # 继续下一轮（可能后面还有粘包）

                # 安全阈值：缓存过大时尝试丢弃到下一个 0xAA
                if len(self._rx_buffer) > 4096:
                    idx = self._rx_buffer.find(FRAME_HEADER)
                    if idx <= 0:
                        self._rx_buffer.clear()
                    else:
                        self._rx_buffer = self._rx_buffer[idx:]

            return None

        except Exception as e:
            logger.error(f"读取数据异常: {e}")
            if not hasattr(self, '_frame_fail_count'):
                self._frame_fail_count = 0
            self._frame_fail_count += 1
            return None

    def _serial_data_check(self, frame: bytearray) -> bool:
        """
        校验单帧:
          长度匹配
          头尾正确
          校验位 = (IDENT + sum(DATA)) % 2
        """
        if len(frame) < DEFAULT_LENGTH:
            return False
        if frame[0] != FRAME_HEADER or frame[-1] != FRAME_TAIL:
            return False

        cmd = frame[1]
        data_len = frame[2]
        ident = frame[3]

        expected_len = data_len + DEFAULT_LENGTH
        if len(frame) != expected_len:
            return False

        data_start = 4
        data_end = data_start + data_len
        payload = frame[data_start:data_end]
        checksum = frame[-2]

        return checksum == self._calculate_checksum(ident, payload)

    def _calculate_checksum(self, ident: int, data: bytes | bytearray | List[int]) -> int:
        """
        新校验算法: (IDENT + sum(DATA)) % 2
        """
        return (ident + sum(data)) % 2

    
    def _sum_elements(self, data: List[int]) -> int:
        """
        计算数据的校验和
        
        Args:
            data: 数据帧
            
        Returns:
            int: 校验和
        """
        if len(data) < 4:
            logger.error("数据数组太小，无法计算校验和")
            return 0
        
        # 计算从第3个字节到倒数第2个字节之前的所有元素的和
        sum_value = 0
        for i in range(3, len(data) - 2):
            sum_value += data[i]
        
        return sum_value % 2
    
    def _print_hex_frame(self, data: List[int], type_code: int):
        """
        打印十六进制数据
        
        Args:
            data: 数据帧
            type_code: 0=发送数据, 1=接收数据, 其他=部分数据
        """
        # 过滤不需要打印的特定请求帧
        if type_code == 0 and (
            data == [0xAA,0x06,0x01,0x00,0x00,0xFF] or  # LEN=1 版本
            data == [0xAA,0x06,0x00,0x00,0x00,0xFF]     # LEN=0 版本
        ):
            return
        prefix = {0: "发送数据: ", 1: "接收数据: ", 2: "部分数据: "}.get(type_code, "未知数据: ")
        hex_str = " ".join([f"{byte:02X}" for byte in data])
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]  # 毫秒级时间戳
        print(f"[{ts}] {prefix}{hex_str}")

