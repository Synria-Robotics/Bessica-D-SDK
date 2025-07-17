import threading
import time
import serial
import json
import argparse
from datetime import datetime
from data_parser import DataParser, JointState, JointStateDict
from typing import List

DEFAULT_LENGTH = 5
FRAME_LENGTH = 50

class SerialDiagnostic:
    def __init__(self, port: str='/dev/ttyUSB0', baudrate: int=921600 , 
                 save_path: str='/home/senyu/Bessica_github/Bessica-D-SDK/bessica_d_sdk/logs/serial_log.json', 
                 read_interval: float=0.05, dummy_mode=False, enable_parser = False):
        self.port = port
        self.baudrate = baudrate
        self.save_path = save_path
        self.read_interval = read_interval
        self.dummy_mode = dummy_mode


        self.serial = None
        self._rx_buffer = bytearray()
        self._log = []
        self._last_print_time = 0

        self.running = False
        self.paused = False
        self._lock = threading.Lock()
        self._read_thread = None

        self._last_print_time = 0
        self._last_clear_time = 0

        if enable_parser:
            self.parser = DataParser(debug_mode=False)
        else:
            self.parser = None

    def connect(self):
        if self.dummy_mode:
            print("[Dummy Mode] Using dummy data source")
            return True
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.01)
            print(f"[Connected] {self.port} at {self.baudrate} bps")
            return True
        except Exception as e:
            print(f"[Error] Serial connection failed: {e}")
            return False

    def start(self):
        if not self.dummy_mode and not self.connect():
            return

        self.running = True
        if self.dummy_mode:
            print("[Dummy Mode] Using dummy data source")
            self._read_thread = threading.Thread(target=self._dummy_loop, daemon=True)
        else:
            self._read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self._read_thread.start()
        print("[Thread] Serial read thread started")

    def stop(self):
        self.running = False
        if self._read_thread:
            self._read_thread.join()
        self.save_log()
        if self.serial:
            self.serial.close()
        print("[Shutdown] Diagnostic stopped and serial port closed")

    def pause(self):
        self.paused = True
        print("[Paused] Data reading paused")

    def resume(self):
        self.paused = False
        print("[Resumed] Data reading resumed")

    def read_loop(self):
        while self.running:
            if self.paused:
                time.sleep(0.1)
                continue

            try:
                with self._lock:
                    if self.dummy_mode:
                        self._rx_buffer += self.generate_dummy_frame()
                    elif self.serial.in_waiting > 0:
                        self._rx_buffer += self.serial.read(self.serial.in_waiting)
                    self._parse_and_log()
            except Exception as e:
                print(f"[Read Error] {e}")

            time.sleep(self.read_interval)

    def _dummy_loop(self):
        while self.running:
            if self.paused:
                time.sleep(0.1)
                continue

            dummy = self.generate_dummy_frame()

            with self._lock:
                self._rx_buffer += dummy
                self._parse_and_log()

            time.sleep(self.read_interval)

    def send_data(self, data: List[int]) -> bool:
        with self._lock:
            try:
                if not self.serial or not self.serial.is_open:
                    print("[warning]串口未打开，尝试重新连接")
                    if not self.connect():
                        print("[error]无法连接到串口")
                        return False
                
                # 转换为字节数组
                data_bytes = bytes(data)
                
                # 写入数据
                bytes_written = self.serial.write(data_bytes)
                
                if bytes_written != len(data):
                    print(f"只写入了 {bytes_written} 字节，应为 {len(data)} 字节")
                    return False
                
                return True
                    
            except Exception as e:
                print(f"发送数据时异常: {str(e)}")
                return False
            
    def _parse_and_log(self):
        while len(self._rx_buffer) >= FRAME_LENGTH:
            # Step 1: 同步到帧头 0xAA
            if self._rx_buffer[0] != 0xAA:
                self._rx_buffer.pop(0)
                continue

            candidate = self._rx_buffer[:FRAME_LENGTH]

            # Step 2: 验证帧尾和校验
            valid_tail = candidate[-1] == 0xFF
            valid_checksum = self._serial_data_check(candidate)
            

            if self.parser:
                content_length = FRAME_LENGTH - DEFAULT_LENGTH
                valid_type = candidate[3] == self.parser.PRESENT_POSITION
                valid_length = candidate[2] == content_length
                
                # 舵机到关节的映射表
                servo_to_joint_map = {
                    0: (0, 1.0),    # 舵机1 -> 关节1 (正向)
                    1: None,        # 舵机2 -> 忽略  (重复)
                    2: (1, 1.0),    # 舵机3 -> 关节2 (正向)
                    3: None,        # 舵机4 -> 忽略  (重复反向)
                    4: (2, 1.0),    # 舵机5 -> 关节3 (正向)
                    5: (3, 1.0),    # 舵机6 -> 关节4  (正向）
                    6: None,        # 舵机7 -> 忽略  （重复反向）
                    7: (4, 1.0),    # 舵机8 -> 关节5  (正向)
                    8: (5, 1.0),    # 舵机9 -> 关节6  （正向）
                    9: (6, 1.0),    # 舵机10 -> 关节7 （正向）
                    10: (7, 1,0)    # 舵机11 -> 夹爪  （正向）
                }

                for arm, start_idx in zip(["left_arm", "right_arm"], [4,26]):
                
                    # 初始化关节和夹爪角度数组 
                    joint_values = [0.0] * 7
                    servo_values = []
                    
                    # 处理前10个舵机机械臂关节数据
                    for i in range(10):
                        # 数据索引计算
                        byte_idx = start_idx + i * 2
                        if byte_idx + 1 >= len(candidate):
                            print(f"[warning]舵机数据越界: 索引{byte_idx}超出范围")
                            break
                        
                        # 解析舵机原始值
                        low_byte = candidate[byte_idx]
                        high_byte = candidate[byte_idx + 1]
                        servo_value = (low_byte & 0xFF) | ((high_byte & 0xFF) << 8)
                        servo_values.append(servo_value)
                        
                        # 映射到关节
                        mapping = servo_to_joint_map.get(i)
                        if mapping is not None:
                            joint_idx, direction = mapping
                            # 转换为弧度并应用方向系数
                            angle_rad = self.parser._value_to_radians(servo_value) * direction
                            joint_values[joint_idx] = angle_rad
                
                    # 处理夹爪数据
                    gripper_raw = candidate[start_idx + 10*2] | (candidate[start_idx + 10*2 + 1])

                    # 范围检查
                    if gripper_raw < 2048 or gripper_raw > 2900:
                        gripper_raw = max(2048, min(gripper_raw, 2900))
                    
                    # 转换为角度 (0-100度)
                    servo_to_angle_ratio = (2900-2048)/100
                    angle_deg = (gripper_raw - 2048) / servo_to_angle_ratio
                    
                    # 转换为弧度
                    gripper_rad = angle_deg * self.parser.DEG_TO_RAD

                
                    self.parser._joint_states[arm] = JointState(joint_values, gripper_rad, time.time())
            
                parsed = {
                    "timestamp": datetime.now().isoformat(),
                    "raw": ' '.join(f"{b:02X}" for b in candidate),
                    "arm_state": self.parser.get_joint_state(),
                    "left_arm_angles": self.parser.get_joint_state(arm='left_arm').angles,
                    "right_arm_angles": self.parser.get_joint_state(arm='right_arm').angles,
                    "valid": valid_tail and valid_checksum and valid_type and valid_length
                }
                self._log.append(parsed)

                now = time.time()
                if now - self._last_print_time > self.read_interval:
                    print(f"[Frame] {parsed['raw']} {'(OK)' if parsed['valid'] else '(Invalid)'}")
                    print(f"[Left_angle]{parsed['left_arm_angles']}, [Right_angle]{parsed['right_arm_angles']}")
                    self._last_print_time = now
                    # print(f"[buffer] 长度：{len(self._rx_buffer)}")
                if parsed["valid"]:
                    return parsed["arm_state"]

            else:
                parsed = {
                    "timestamp": datetime.now().isoformat(),
                    "raw": ' '.join(f"{b:02X}" for b in candidate),
                    "raw_decimal": list(candidate),
                    "valid": valid_tail and valid_checksum
                }
                self._log.append(parsed)

                now = time.time()
                if now - self._last_print_time > 1.0:
                    print(f"[Frame] {parsed['raw_decimal']} {'(OK)' if parsed['valid'] else '(Invalid)'}")
                    self._last_print_time = now
                    # print(f"[buffer] 长度：{len(self._rx_buffer)}")
            
            # Step 3: 清除当前帧数据
            self._rx_buffer = self._rx_buffer[FRAME_LENGTH:]

            # Step 4: 若缓存过大，强制同步（防炸）
            if len(self._rx_buffer) > 1000:
                now = time.time()
                # if now - self._last_clear_time > 1.0:
                #     print(f"[Warning] Buffer too large ({len(self._rx_buffer)}), force cleaning...")
                #     print(f"帧头：{self._rx_buffer[0]}, frame tail: {self._rx_buffer[FRAME_LENGTH-1]}")
                    # self._last_clear_time = now
                aa_index = self._rx_buffer.find(0xAA)
                if aa_index == -1:
                    self._rx_buffer.clear()
                else:
                    self._rx_buffer = self._rx_buffer[aa_index:]

                    # print(f"[buffer] 长度：{len(self._rx_buffer)}")

    def _serial_data_check(self, frame: bytearray) -> bool:
        data_len = frame[2]
        if len(frame) != data_len + DEFAULT_LENGTH:
            return False
        
        # print(data_len)
        # print(len(frame))
        payload = frame[3:3 + data_len]
        checksum = frame[3 + data_len]
        return checksum == self._calculate_checksum(payload)

    
    def _sum_elements(self, data) -> int:
        """
        计算数据的校验和
        
        Args:
            data: 数据帧
            
        Returns:
            int: 校验和
        """
        if len(data) < 4:
            print("数据数组太小，无法计算校验和")
            return 0
        
        # 计算从第3个字节到倒数第2个字节之前的所有元素的和
        sum_value = 0
        for i in range(3, len(data) - 2):
            sum_value += data[i]
        
        return sum_value % 2

    def _calculate_checksum(self,data) -> int:
        sum_value = 0
        for i in range(len(data)):
            sum_value += data[i]
        return sum_value % 2
    
    def save_log(self):
        try:
            with open(self.save_path, 'w') as f:
                json.dump(self._log, f, indent=2)
            print(f"[Saved] Log saved to {self.save_path}")
        except Exception as e:
            print(f"[Save Error] {e}")

    def generate_dummy_frame(self):
        payload = list(range(1, 46))  # payload: 01~0D
        header = [0xAA, 0x06]
        length = [len(payload)]
        content = payload
        checksum = self._calculate_checksum(content)
        frame = header + length + content + [checksum, 0xFF]
        return bytearray(frame)


    

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=str, default='/dev/ttyUSB0', help="Serial port, e.g., /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=921600, help="Baud rate")
    parser.add_argument("--save-log", type=str, default="/home/senyu/Bessica_github/Bessica-D-SDK/bessica_d_sdk/logs/serial_log.json", help="Log save file")
    parser.add_argument("--dummy", action="store_true", help="Use dummy data source")
    parser.add_argument("--read-interval", type=float, default=0.05, help="Type in read interval, eg,0.05")
    args = parser.parse_args()

    # diag = SerialDiagnostic(port=args.port, baudrate=args.baudrate, 
    #                         save_path=args.save_log, read_interval=args.read_interval, dummy_mode=args.dummy)
    diag = SerialDiagnostic(read_interval=2.0,dummy_mode=False,enable_parser=True)
    diag.start()

    try:
        while True:
            
            cmd = input("[CMD] > ").strip()
            if cmd == "pause":
                diag.pause()
            elif cmd == "resume":
                diag.resume()
            elif cmd == "exit":
                break
            elif cmd == "save":
                diag.save_log()
            else:
                print("[Commands] pause | resume | save | exit")
    except KeyboardInterrupt:
        print("\n[Ctrl+C] Exiting...")
    finally:
        diag.stop()
