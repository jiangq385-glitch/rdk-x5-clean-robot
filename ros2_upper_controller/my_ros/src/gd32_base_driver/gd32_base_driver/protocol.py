"""串口协议解析 - 定长帧格式。

该协议使用 12 字节定长帧：

    Byte0   : 0x55 (FRAME_HEADER)
    Byte1   : type (帧类型)
    Byte2   : len  (固定为 0x07)
    Byte3-4 : vx_hi, vx_lo  (无符号 16bit，配合 flag 表示正负)
    Byte5-6 : vy_hi, vy_lo
    Byte7-8 : wz_hi, wz_lo
    Byte9   : flag (符号位标志)
    Byte10  : checksum = (type + len + 6个数据字节 + flag) & 0xFF
    Byte11  : 0xBB (FRAME_TAIL)

数值缩放：
    - 三个数据字段单位采用 *1000 的整数传输（PACKET_SCALE=1000）
    - 解析时除以 1000.0 得到浮点

符号规则：
    - flag 的 bit0 对应 wz，bit1 对应 vy，bit2 对应 vx
    - 该实现约定：对应 bit 置 1 表示非负，置 0 表示负数。
"""
import threading
import math
from dataclasses import dataclass
from typing import Iterable

# 帧边界与长度
FRAME_HEADER = 0x55
FRAME_TAIL = 0xBB
PACKET_LEN = 12

# 该协议把 payload len 固定为 0x07（与 PACKET_LEN 组合成定长帧）
FRAME_LEN = 0x07

# 放大倍数：把 m/s 或 rad/s 等浮点量放大 1000 后以整数发送
PACKET_SCALE = 1000.0

# 这里按 16bit 无符号范围做饱和（0..65535），并用 flag 表示正负
PACKET_MAX_INT16 = 65535
PACKET_MAX_ABS = PACKET_MAX_INT16 / PACKET_SCALE

# 帧类型定义：
# - 0x01：上位机->下位机 速度控制
# - 0x2x：上位机->下位机 机械臂0控制（仍使用同一 12 字节定长帧）
# - 0x3x：上位机->下位机 机械臂1控制
# - 0x8x：下位机->上位机 状态回传（里程计/IMU/电池等）
# - 0x9x：下位机->上位机 机械臂0状态回传
# - 0xAx：下位机->上位机 机械臂1状态回传
TYPE_CMD_VEL = 0x01
TYPE_ARM0_JOINTS_123 = 0x21
TYPE_ARM0_JOINTS_456 = 0x22
TYPE_ARM0_GO_HOME = 0x25
TYPE_ARM1_JOINTS_123 = 0x31
TYPE_ARM1_JOINTS_456 = 0x32
TYPE_ARM1_GO_HOME = 0x35
TYPE_ODOM_POSE = 0x81
TYPE_IMU_ACC = 0x82
TYPE_IMU_GYR = 0x83
TYPE_BAT_MV = 0x84
TYPE_ROBOT_VEL = 0x85
TYPE_IMU_YAW = 0x86
TYPE_ARM0_STATUS = 0x93
TYPE_ARM1_STATUS = 0xA3

# Backward-compatible aliases for the original single-arm names.
TYPE_ARM_JOINTS_123 = TYPE_ARM0_JOINTS_123
TYPE_ARM_JOINTS_456 = TYPE_ARM0_JOINTS_456
TYPE_ARM_GO_HOME = TYPE_ARM0_GO_HOME
TYPE_ARM_STATUS = TYPE_ARM0_STATUS

ARM_COUNT = 2
ARM_MAX_JOINTS = 6
ARM_JOINT_SCALE = 1000.0
ARM_STATUS_ARRIVED_BIT = 0x01
ARM_STATUS_FAULT_BIT = 0x02


@dataclass
class Vel3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class Protocol:
    def __init__(self, log=None, debug_rx: bool = False):
        # buffer: 串口是字节流，可能出现“半包/粘包”，需要先缓存再按帧切分。
        self.buffer = bytearray()
        self.state_lock = threading.Lock()

        self._log = log
        self._debug_rx = bool(debug_rx)
        self._rx_ok_count = 0

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_yaw = 0.0
        self.imu_yaw = 0.0
        self.acc = Vel3()
        self.gyr = Vel3()
        self.battery_v = 0.0
        self.vel_fb = Vel3()
        self.arm_arrived = [False] * ARM_COUNT
        self.arm_fault = [False] * ARM_COUNT
        self.arm_fault_code = [0] * ARM_COUNT

        self.rx_bad_header = 0
        self.rx_bad_tail = 0
        self.rx_bad_checksum = 0

    def _emit_log(self, level: str, msg: str):
        if self._log is None:
            return
        try:
            self._log(level, msg)
        except Exception:
            pass

    def _log_rx_issue(self, label: str, count: int):
        if count % 100 == 1:
            self._emit_log('debug', f"接收异常 {label}: {count}")

    def feed(self, data: bytes):
        """接收串口原始数据，自动解析并更新状态。

        - 允许一次喂入任意长度的数据
        - 内部会反复在 buffer 中寻找 0x55 作为帧头，并检查帧尾/校验
        - 成功解析后调用 _handle_packet 更新状态
        """
        self.buffer += data
        while len(self.buffer) >= PACKET_LEN:
            header_idx = self.buffer.find(bytes([FRAME_HEADER]))
            if header_idx == -1:
                self.buffer = self.buffer[1:]
                self.rx_bad_header += 1
                self._log_rx_issue('bad_header', self.rx_bad_header)
                continue
            if header_idx > 0:
                self.buffer = self.buffer[header_idx:]
            if len(self.buffer) < PACKET_LEN:
                return
            if self.buffer[PACKET_LEN - 1] != FRAME_TAIL:
                self.buffer = self.buffer[1:]
                self.rx_bad_tail += 1
                self._log_rx_issue('bad_tail', self.rx_bad_tail)
                continue

            pkt = self.buffer[:PACKET_LEN]
            self.buffer = self.buffer[PACKET_LEN:]
            self._handle_packet(pkt)

    def _handle_packet(self, pkt: bytes):
        # 这里假设 pkt 已经是一个完整的 12 字节帧
        if len(pkt) != PACKET_LEN:
            return
        if pkt[0] != FRAME_HEADER or pkt[-1] != FRAME_TAIL:
            return

        pkt_type = pkt[1]
        data_len = pkt[2]
        if data_len != FRAME_LEN:
            return

        vx_hi, vx_lo = pkt[3], pkt[4]
        vy_hi, vy_lo = pkt[5], pkt[6]
        wz_hi, wz_lo = pkt[7], pkt[8]
        flag = pkt[9]
        checksum = pkt[10]
        calc = (pkt_type + data_len + vx_hi + vx_lo + vy_hi + vy_lo + wz_hi + wz_lo + flag) & 0xFF
        if checksum != calc:
            self.rx_bad_checksum += 1
            self._log_rx_issue('bad_checksum', self.rx_bad_checksum)
            return

        self._rx_ok_count += 1
        if self._debug_rx and (self._rx_ok_count % 50 == 1):
            self._emit_log('info', f"RX ok type=0x{pkt_type:02X} flag=0x{flag:02X} | {pkt.hex(' ')}")

        def _from_flag(hi: int, lo: int, sign_bit: int) -> float:
            """把 (hi,lo) 与 flag 指定的符号位合成浮点值。"""
            value = ((hi << 8) | lo) & 0xFFFF
            if (flag & sign_bit) == 0:
                value = -value
            return value / 1000.0

        with self.state_lock:
            if pkt_type == TYPE_ODOM_POSE:
                self.pos_x = _from_flag(vx_hi, vx_lo, 0x04)
                self.pos_y = _from_flag(vy_hi, vy_lo, 0x02)
                self.pos_yaw = _from_flag(wz_hi, wz_lo, 0x01)
            elif pkt_type == TYPE_IMU_ACC:
                self.acc.x = _from_flag(vx_hi, vx_lo, 0x04)
                self.acc.y = _from_flag(vy_hi, vy_lo, 0x02)
                self.acc.z = _from_flag(wz_hi, wz_lo, 0x01)
            elif pkt_type == TYPE_IMU_GYR:
                self.gyr.x = _from_flag(vx_hi, vx_lo, 0x04)
                self.gyr.y = _from_flag(vy_hi, vy_lo, 0x02)
                self.gyr.z = _from_flag(wz_hi, wz_lo, 0x01)
            elif pkt_type == TYPE_IMU_YAW:
                self.imu_yaw = _from_flag(vx_hi, vx_lo, 0x04)
            elif pkt_type == TYPE_BAT_MV:
                self.battery_v = _from_flag(vx_hi, vx_lo, 0x04)
            elif pkt_type == TYPE_ROBOT_VEL:
                self.vel_fb.x = (_from_flag(vx_hi, vx_lo, 0x04) + _from_flag(vy_hi, vy_lo, 0x02)) / 2.0
                self.vel_fb.y = 0.0
                self.vel_fb.z = _from_flag(wz_hi, wz_lo, 0x01)
            elif pkt_type in (TYPE_ARM0_STATUS, TYPE_ARM1_STATUS):
                arm_id = 0 if pkt_type == TYPE_ARM0_STATUS else 1
                data1 = vx_hi * 256 + vx_lo
                data2 = vy_hi * 256 + vy_lo
                data3 = wz_hi * 256 + wz_lo
                # Arm status uses data1 as task-complete flag: 1 means done, 0 means running.
                self.arm_arrived[arm_id] = data1 != 0
                self.arm_fault[arm_id] = data2 != 0
                self.arm_fault_code[arm_id] = data3

    def get_state(self):
        """获取当前状态快照（线程安全）"""
        with self.state_lock:
            return {
                'pos_x': self.pos_x,
                'pos_y': self.pos_y,
                'pos_yaw': self.pos_yaw,
                'imu_yaw': self.imu_yaw,
                'acc': Vel3(self.acc.x, self.acc.y, self.acc.z),
                'gyr': Vel3(self.gyr.x, self.gyr.y, self.gyr.z),
                'battery_v': self.battery_v,
                'vel_fb': Vel3(self.vel_fb.x, self.vel_fb.y, self.vel_fb.z),
                'arms': [
                    {
                        'arrived': self.arm_arrived[index],
                        'fault': self.arm_fault[index],
                        'fault_code': self.arm_fault_code[index],
                    }
                    for index in range(ARM_COUNT)
                ],
                # Compatibility keys expose arm0 under the original single-arm names.
                'arm_arrived': self.arm_arrived[0],
                'arm_fault': self.arm_fault[0],
                'arm_fault_code': self.arm_fault_code[0],
            }

    @staticmethod
    def _build_three_value_packet(pkt_type: int, values: Iterable[float], scale: float = PACKET_SCALE) -> bytes:
        """Pack 3 signed values with the existing fixed 12-byte protocol."""
        packed_values = list(values)
        if len(packed_values) != 3:
            raise ValueError('packet needs exactly 3 values')

        flag = 0
        raw = []
        max_abs = PACKET_MAX_INT16 / scale
        for index, value in enumerate(packed_values):
            value = max(-max_abs, min(max_abs, float(value)))
            if value >= 0:
                flag |= 1 << (2 - index)
            raw.append(int(abs(value) * scale) & 0xFFFF)

        b0_hi, b0_lo = (raw[0] >> 8) & 0xFF, raw[0] & 0xFF
        b1_hi, b1_lo = (raw[1] >> 8) & 0xFF, raw[1] & 0xFF
        b2_hi, b2_lo = (raw[2] >> 8) & 0xFF, raw[2] & 0xFF
        checksum = (pkt_type + FRAME_LEN + b0_hi + b0_lo + b1_hi + b1_lo + b2_hi + b2_lo + flag) & 0xFF

        return bytes([
            FRAME_HEADER, pkt_type, FRAME_LEN,
            b0_hi, b0_lo, b1_hi, b1_lo, b2_hi, b2_lo,
            flag, checksum, FRAME_TAIL
        ])

    @staticmethod
    def _arm_command_types(arm_id: int) -> tuple[int, int, int]:
        if arm_id == 0:
            return (TYPE_ARM0_JOINTS_123, TYPE_ARM0_JOINTS_456, TYPE_ARM0_GO_HOME)
        if arm_id == 1:
            return (TYPE_ARM1_JOINTS_123, TYPE_ARM1_JOINTS_456, TYPE_ARM1_GO_HOME)
        raise ValueError(f'arm_id must be 0 or 1, got {arm_id}')

    @staticmethod
    def build_arm_joint_packets(positions: Iterable[float], arm_id: int = 0) -> list[bytes]:
        """Build arm joint target packets. Joint positions are radians scaled by 1000."""
        joints_123_type, joints_456_type, _ = Protocol._arm_command_types(arm_id)
        joints = [float(value) for value in positions]
        joint_count = len(joints)
        if not joints:
            raise ValueError('positions must not be empty')
        if joint_count > ARM_MAX_JOINTS:
            raise ValueError(f'positions supports at most {ARM_MAX_JOINTS} joints')

        joints = joints + [0.0] * (ARM_MAX_JOINTS - joint_count)
        packets = [Protocol._build_three_value_packet(joints_123_type, joints[:3], ARM_JOINT_SCALE)]
        if joint_count > 3:
            packets.append(Protocol._build_three_value_packet(joints_456_type, joints[3:6], ARM_JOINT_SCALE))
        return packets

    @staticmethod
    def build_arm_go_home(arm_id: int = 0) -> bytes:
        """Build arm go-home command packet."""
        _, _, go_home_type = Protocol._arm_command_types(arm_id)
        return Protocol._build_three_value_packet(go_home_type, [1.0, 0.0, 0.0])

    @staticmethod
    def build_cmd_vel(vx: float, vy: float, wz: float) -> bytes:
        """打包速度控制命令（上位机 -> 下位机）。

        - 输入为 vx/vy/wz 浮点
        - 输出为 12 字节帧
        - 取绝对值做量化，符号用 flag 表达
        - 超出范围会做饱和（PACKET_MAX_ABS）
        """
        flag = 0
        if vx >= 0:
            flag |= 0x04
        if vy >= 0:
            flag |= 0x02
        if wz >= 0:
            flag |= 0x01

        vx = max(-PACKET_MAX_ABS, min(PACKET_MAX_ABS, vx))
        vy = max(-PACKET_MAX_ABS, min(PACKET_MAX_ABS, vy))
        wz = max(-PACKET_MAX_ABS, min(PACKET_MAX_ABS, wz))

        vx_int = int(abs(vx) * PACKET_SCALE) & 0xFFFF
        vy_int = int(abs(vy) * PACKET_SCALE) & 0xFFFF
        wz_int = int(abs(wz) * PACKET_SCALE) & 0xFFFF

        vx_hi, vx_lo = (vx_int >> 8) & 0xFF, vx_int & 0xFF
        vy_hi, vy_lo = (vy_int >> 8) & 0xFF, vy_int & 0xFF
        wz_hi, wz_lo = (wz_int >> 8) & 0xFF, wz_int & 0xFF

        checksum = (TYPE_CMD_VEL + FRAME_LEN + vx_hi + vx_lo + vy_hi + vy_lo + wz_hi + wz_lo + flag) & 0xFF

        return bytes([
            FRAME_HEADER, TYPE_CMD_VEL, FRAME_LEN,
            vx_hi, vx_lo, vy_hi, vy_lo, wz_hi, wz_lo,
            flag, checksum, FRAME_TAIL
        ])


def yaw_to_quat(yaw: float):
    """yaw(rad) -> quaternion (x,y,z,w)"""
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)
