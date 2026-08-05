import math
import threading
import unittest
import zlib

from bessica_d_sdk.hardware.data_parser import DataParser
from bessica_d_sdk.hardware.serial_comm import SerialComm
from bessica_d_sdk.hardware.servo_driver import ServoDriver


class FakeSerial:
    """In-memory serial input exposing only the reader's required interface."""

    def __init__(self, data):
        self._data = bytearray(data)
        self.is_open = True

    @property
    def in_waiting(self):
        return len(self._data)

    def read(self, size):
        chunk = self._data[:size]
        del self._data[:size]
        return bytes(chunk)

    def close(self):
        self.is_open = False


class JointResponseProtocolTests(unittest.TestCase):
    def test_joint_gripper_request_frame_matches_protocol(self):
        driver = ServoDriver()

        self.assertEqual(
            driver._build_joint_gripper_frame(0x00),
            [0xAA, 0x06, 0x00, 0x01, 0xFE, 0x9A, 0xFF],
        )

    def test_read_frame_rejects_truncated_malformed_firmware_response(self):
        serial_comm = SerialComm()
        malformed_frame = [0] * 38
        malformed_frame[0:4] = [0xAA, 0x06, 0x20, 0x38]
        malformed_frame[-1] = 0xFF
        malformed_frame[-2] = zlib.crc32(bytes(malformed_frame[1:-2])) & 0xFF
        fake_serial = FakeSerial(malformed_frame)
        serial_comm.serial_port = fake_serial

        self.assertIsNone(serial_comm.read_frame())
        fake_serial.close()

    def test_reads_and_parses_complete_dual_arm_joint_response(self):
        serial_comm = SerialComm()
        right_joints = [101, 203, 307, 401, 509, 601, 701]
        left_joints = [809, 907, 1009, 1103, 1201, 1301, 1409]
        right_gripper = 345
        left_gripper = 678
        run_status = 0x11

        joint_data = self._little_endian_values(right_joints + left_joints)
        joint_data += self._little_endian_values([right_gripper, left_gripper])
        joint_data.append(run_status)
        frame = [0xAA, 0x06, 0x00, 0x21, *joint_data, 0x00, 0xFF]
        golden_crc = zlib.crc32(bytes(frame[1:-2])) & 0xFF
        self.assertEqual(golden_crc, 0x3D)
        frame[-2] = golden_crc
        self.assertEqual(len(frame), 39)

        fake_serial = FakeSerial(frame)
        serial_comm.serial_port = fake_serial
        received_frame = serial_comm.read_frame()

        self.assertEqual(received_frame, frame)
        parser = DataParser(lock=threading.Lock())
        parsed = parser.parse_frame(received_frame)

        self.assertIsNotNone(parsed)
        self.assertTrue(parser._joint_event.is_set())
        self._assert_joint_angles(parsed["right_angle"], right_joints)
        self._assert_joint_angles(parsed["left_angle"], left_joints)
        self.assertEqual(parsed["right_gripper"], float(right_gripper))
        self.assertEqual(parsed["left_gripper"], float(left_gripper))
        self.assertEqual(parsed["run_status"], run_status)
        fake_serial.close()

    def test_recovers_valid_frame_after_malformed_legacy_frame(self):
        serial_comm = SerialComm()
        malformed_frame = [0] * 38
        malformed_frame[0:4] = [0xAA, 0x06, 0x20, 0x38]
        malformed_frame[-2] = zlib.crc32(bytes(malformed_frame[1:-2])) & 0xFF
        malformed_frame[-1] = 0xFF

        right_joints = [101, 203, 307, 401, 509, 601, 701]
        left_joints = [809, 907, 1009, 1103, 1201, 1301, 1409]
        joint_data = self._little_endian_values(right_joints + left_joints)
        joint_data += self._little_endian_values([345, 678])
        joint_data.append(0x11)
        valid_frame = [0xAA, 0x06, 0x00, 0x21, *joint_data, 0x00, 0xFF]
        valid_frame[-2] = zlib.crc32(bytes(valid_frame[1:-2])) & 0xFF
        self.assertEqual(valid_frame[-2], 0x3D)
        self.assertEqual(len(valid_frame), 39)

        fake_serial = FakeSerial(malformed_frame + valid_frame)
        serial_comm.serial_port = fake_serial
        received_frame = None
        for _ in range(3):
            received_frame = serial_comm.read_frame()
            if received_frame is not None:
                break

        self.assertEqual(received_frame, valid_frame)
        fake_serial.close()

    @staticmethod
    def _little_endian_values(values):
        bytes_ = []
        for value in values:
            bytes_.extend([value & 0xFF, (value >> 8) & 0xFF])
        return bytes_

    def _assert_joint_angles(self, actual_angles, raw_values):
        self.assertEqual(len(actual_angles), len(raw_values))
        for actual, raw in zip(actual_angles, raw_values):
            expected = (raw / 4096.0) * (2 * math.pi) - math.pi
            self.assertAlmostEqual(actual, expected)


if __name__ == "__main__":
    unittest.main()
