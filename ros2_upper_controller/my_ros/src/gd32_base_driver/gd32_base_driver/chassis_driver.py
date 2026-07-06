#!/usr/bin/env python3
"""底盘驱动节点 - 串口桥接。

本节点负责把 ROS2 的速度指令 /cmd_vel 通过串口发送给下位机，并把下位机回传的
里程计/IMU/电池等数据解析后发布成 ROS2 话题。

整体数据流：
    - 订阅：/cmd_vel (geometry_msgs/Twist)
    - 串口 TX：Protocol.build_cmd_vel(...) 生成 12 字节定长帧
    - 串口 RX：后台线程读取字节流 -> Protocol.feed(...) 解析并更新状态
    - 定时发布：20ms 定时器读取协议状态快照 -> 发布 /odom、/imu、/battery_voltage
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import serial
import select
import threading
import time

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Bool, Empty, Float32, Int32

from gd32_base_driver.protocol import Protocol, yaw_to_quat


class ChassisDriver(Node):
    def __init__(self):
        super().__init__('chassis_driver')

        # 串口配置（最小集）：port/baud
        # 说明：更复杂的 data_bits/parity/stop_bits/流控 如需支持，可在此扩展参数。
        self.declare_parameter('port', '/dev/wheeltec_controller')
        self.declare_parameter('baud', 115200)

        # 调试开关：
        # - debug_tx: 打印发送的帧（hex）以及对应的 vx/vy/wz
        # - debug_rx: 由 protocol.py 控制打印接收解析成功的帧（按采样频率输出）
        self.declare_parameter('debug_tx', False)
        self.declare_parameter('debug_rx', False)
        self.declare_parameter('arm_joint_target_topic', '/arm/joint_target')
        self.declare_parameter('arm_go_home_topic', '/arm/go_home')
        self.declare_parameter('arm_arrived_topic', '/arm/arrived')
        self.declare_parameter('arm_fault_topic', '/arm/fault')
        self.declare_parameter('arm_fault_code_topic', '/arm/fault_code')
        self.declare_parameter('arm0_joint_target_topic', '/arm0/joint_target')
        self.declare_parameter('arm0_go_home_topic', '/arm0/go_home')
        self.declare_parameter('arm0_arrived_topic', '/arm0/arrived')
        self.declare_parameter('arm0_fault_topic', '/arm0/fault')
        self.declare_parameter('arm0_fault_code_topic', '/arm0/fault_code')
        self.declare_parameter('arm1_joint_target_topic', '/arm1/joint_target')
        self.declare_parameter('arm1_go_home_topic', '/arm1/go_home')
        self.declare_parameter('arm1_arrived_topic', '/arm1/arrived')
        self.declare_parameter('arm1_fault_topic', '/arm1/fault')
        self.declare_parameter('arm1_fault_code_topic', '/arm1/fault_code')

        # 从参数服务器读取配置（按类型取值，避免 str->bool/int 的坑）。
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.debug_tx = self.get_parameter('debug_tx').get_parameter_value().bool_value
        self.debug_rx = self.get_parameter('debug_rx').get_parameter_value().bool_value
        self.arm_joint_target_topic = self.get_parameter('arm_joint_target_topic').value
        self.arm_go_home_topic = self.get_parameter('arm_go_home_topic').value
        self.arm_arrived_topic = self.get_parameter('arm_arrived_topic').value
        self.arm_fault_topic = self.get_parameter('arm_fault_topic').value
        self.arm_fault_code_topic = self.get_parameter('arm_fault_code_topic').value
        self.arm_topics = [
            {
                'joint_target': self.get_parameter('arm0_joint_target_topic').value,
                'go_home': self.get_parameter('arm0_go_home_topic').value,
                'arrived': self.get_parameter('arm0_arrived_topic').value,
                'fault': self.get_parameter('arm0_fault_topic').value,
                'fault_code': self.get_parameter('arm0_fault_code_topic').value,
            },
            {
                'joint_target': self.get_parameter('arm1_joint_target_topic').value,
                'go_home': self.get_parameter('arm1_go_home_topic').value,
                'arrived': self.get_parameter('arm1_arrived_topic').value,
                'fault': self.get_parameter('arm1_fault_topic').value,
                'fault_code': self.get_parameter('arm1_fault_code_topic').value,
            },
        ]

        # 最小兜底：若类型不匹配导致取到空/0，则回退到声明时的默认值。
        if not port:
            port = '/dev/wheeltec_controller'
        if baud == 0:
            baud = 115200

        self._last_tx_log_ts = 0.0
        self._last_serial_warn_ts = 0.0
        self._serial_empty_read_errors = 0
        self._serial_write_lock = threading.Lock()

        # 串口句柄：若打开失败，节点仍会运行，但无法发送/接收。
        self.serial = None
        try:
            serial_kwargs = {
                'port': port,
                'baudrate': baud,
                'timeout': 0.0,
                'bytesize': serial.EIGHTBITS,
                'parity': serial.PARITY_NONE,
                'stopbits': serial.STOPBITS_ONE,
                'xonxoff': False,
                'rtscts': False,
                'dsrdtr': False,
            }
            # POSIX 下开启独占访问，避免两个进程同时打开同一串口。
            try:
                self.serial = serial.Serial(exclusive=True, **serial_kwargs)
            except TypeError:
                self.serial = serial.Serial(**serial_kwargs)
                self.get_logger().warn('当前 pyserial 不支持 exclusive 参数，无法启用串口独占访问')

            try:
                self.serial.setDTR(False)
                self.serial.setRTS(False)

                # 等待串口芯片和 termios 参数稳定
                time.sleep(0.3)

                # 丢弃打开串口瞬间产生/积累的乱码
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()

            except Exception as e:
                self.get_logger().warn(f'串口初始化清缓冲失败: {e}')
            self.get_logger().info(f'串口已打开: {port}, 波特率: {baud}')
        except Exception as e:
            self.get_logger().error(f'无法打开串口 {port} (baud={baud}): {type(e).__name__}: {e}')

        # 协议解析器：
        # - feed(data): 投喂串口字节流
        # - get_state(): 获取线程安全的状态快照
        self.protocol = Protocol(log=self._proto_log, debug_rx=self.debug_rx)

        # QoS 选型：底盘里程计/IMU 一般希望可靠传输（RELIABLE），不需要持久化（VOLATILE）。
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.imu_pub = self.create_publisher(Imu, '/imu', qos)
        self.bat_pub = self.create_publisher(Float32, '/battery_voltage', qos)
        self.arm_arrived_pub = self.create_publisher(Bool, self.arm_arrived_topic, qos)
        self.arm_fault_pub = self.create_publisher(Bool, self.arm_fault_topic, qos)
        self.arm_fault_code_pub = self.create_publisher(Int32, self.arm_fault_code_topic, qos)
        self.arm_publishers = []
        for topics in self.arm_topics:
            self.arm_publishers.append({
                'arrived': self.create_publisher(Bool, topics['arrived'], qos),
                'fault': self.create_publisher(Bool, topics['fault'], qos),
                'fault_code': self.create_publisher(Int32, topics['fault_code'], qos),
            })

        # 速度指令订阅：收到 /cmd_vel 就封包并通过串口发送给下位机。
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.arm_joint_sub = self.create_subscription(
            JointTrajectory, self.arm_joint_target_topic, self._make_arm_joint_target_cb(0), 10
        )
        self.arm_home_sub = self.create_subscription(Empty, self.arm_go_home_topic, self._make_arm_go_home_cb(0), 10)
        self.arm_subscriptions = [self.arm_joint_sub, self.arm_home_sub]
        for arm_id, topics in enumerate(self.arm_topics):
            self.arm_subscriptions.extend([
                self.create_subscription(
                    JointTrajectory, topics['joint_target'], self._make_arm_joint_target_cb(arm_id), 10
                ),
                self.create_subscription(Empty, topics['go_home'], self._make_arm_go_home_cb(arm_id), 10),
            ])

        # RX 线程：从串口读取字节流，喂给 protocol 进行解包与状态更新。
        self._stop_event = threading.Event()
        self._rx_thread = None
        if self.serial is not None and self.serial.is_open:
            self._rx_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._rx_thread.start()
        else:
            self._rx_thread = None

        # 发布循环：20ms 一次，把协议状态快照转换为 ROS2 消息并发布。
        self.create_timer(0.02, self._publish_loop)
        self.get_logger().info(f'Started on {port} @ {baud}')

    def _on_cmd(self, msg: Twist):
        """/cmd_vel 回调：打包并发送速度命令帧。"""
        if self.serial is None or not self.serial.is_open:
            self.get_logger().warn('串口未打开，无法发送 /cmd_vel')
            return

        cmd = Protocol.build_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z)
        try:
            with self._serial_write_lock:
                self.serial.write(cmd)
            if self.debug_tx:
                now = time.monotonic()
                if now - self._last_tx_log_ts >= 0.2:
                    self._last_tx_log_ts = now
                    self.get_logger().info(
                        f"TX cmd_vel vx={msg.linear.x:.3f} vy={msg.linear.y:.3f} wz={msg.angular.z:.3f} | {cmd.hex(' ')}"
                    )
        except serial.SerialException as e:
            self.get_logger().error(f'串口写入失败: {e}')

    def _write_serial_bytes(self, data):
        if self.serial is None or not self.serial.is_open:
            return False
        try:
            with self._serial_write_lock:
                self.serial.write(data)
            return True
        except serial.SerialException as e:
            self.get_logger().error(str(e))
            return False

    def _trajectory_duration_ms(self, msg):
        point = msg.points[0]
        duration = point.time_from_start
        return int(duration.sec * 1000 + duration.nanosec / 1000000)

    def _make_arm_joint_target_cb(self, arm_id: int):
        def _on_arm_joint_target(msg):
            if not msg.points:
                return
            point = msg.points[0]
            if not point.positions:
                return
            try:
                packets = Protocol.build_arm_joint_packets(point.positions, arm_id)
            except ValueError as e:
                self.get_logger().warn(str(e))
                return
            self._write_serial_bytes(bytes().join(packets))
        return _on_arm_joint_target

    def _make_arm_go_home_cb(self, arm_id: int):
        def _on_arm_go_home(msg):
            cmd = Protocol.build_arm_go_home(arm_id)
            self._write_serial_bytes(cmd)
        return _on_arm_go_home


    def _read_loop(self):
        """后台线程：select 等待串口可读，读取缓冲区全部字节并喂给协议解析器。"""
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                if self.serial is None or not self.serial.is_open:
                    break

                fd = self.serial.fileno()
                if fd is None:
                    break

                readable, _, _ = select.select([fd], [], [], 0.1)
                if fd in readable:
                    if self._stop_event.is_set() or self.serial is None or not self.serial.is_open:
                        break
                    n = self.serial.in_waiting
                    if n > 0:
                        data = self.serial.read(n)
                        if data:
                            self._serial_empty_read_errors = 0
                            self.protocol.feed(data)
                        else:
                            self._serial_empty_read_errors += 1
                            self._log_serial_warn(
                                '串口可读但未读到数据，可能是设备断开或端口被其他进程占用'
                            )
                            if self._serial_empty_read_errors >= 5:
                                self.get_logger().error('串口连续读空达到阈值，停止监听线程，请检查串口连接与占用情况')
                                break
            except Exception as e:
                self._serial_empty_read_errors += 1
                self._log_serial_warn(f'串口监听异常: {e}')
                if self._serial_empty_read_errors >= 5:
                    self.get_logger().error('串口连续异常达到阈值，停止监听线程，请检查串口连接与占用情况')
                    break
                if self.serial is None or not self.serial.is_open or self._stop_event.is_set():
                    break

    def _log_serial_warn(self, msg: str):
        now = time.monotonic()
        if now - self._last_serial_warn_ts >= 1.0:
            self._last_serial_warn_ts = now
            self.get_logger().warn(msg)

    def _publish_loop(self):
        """定时器回调：发布 odom/imu/battery。"""
        now = self.get_clock().now()
        state = self.protocol.get_state()
        # if abs(state['vel_fb'].z) > 0.05 or abs(state['gyr'].z) > 1.0:
        #     self.get_logger().info(
        #         f"raw odom: x={state['pos_x']:.3f}, y={state['pos_y']:.3f}, "
        #         f"yaw={state['pos_yaw']:.3f}, wz={state['vel_fb'].z:.3f}, imu_gz={state['gyr'].z:.3f}"
        #     )
        qx, qy, qz, qw = yaw_to_quat(state['pos_yaw'])

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = state['pos_x']
        odom.pose.pose.position.y = state['pos_y']
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = state['vel_fb'].x
        odom.twist.twist.linear.y = state['vel_fb'].y
        odom.twist.twist.angular.z = state['vel_fb'].z
        c = 0.01
        odom.pose.covariance = [
            c, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, c, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, c, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, c, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, c,
        ]
        odom.twist.covariance = odom.pose.covariance[:]
        self.odom_pub.publish(odom)

        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = 'imu_link'
        imu_yaw = state.get('imu_yaw', state['pos_yaw'])
        imu_qx, imu_qy, imu_qz, imu_qw = yaw_to_quat(imu_yaw)
        imu.orientation.x = imu_qx
        imu.orientation.y = imu_qy
        imu.orientation.z = imu_qz
        imu.orientation.w = imu_qw
        imu.orientation_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, 0.05,
        ]
        imu.angular_velocity.x = math.radians(state['gyr'].x)
        imu.angular_velocity.y = math.radians(state['gyr'].y)
        imu.angular_velocity.z = math.radians(state['gyr'].z)
        imu.linear_acceleration.x = state['acc'].x
        imu.linear_acceleration.y = state['acc'].y
        imu.linear_acceleration.z = state['acc'].z
        cv = 0.02
        imu.angular_velocity_covariance = [cv, 0.0, 0.0, 0.0, cv, 0.0, 0.0, 0.0, cv]
        imu.linear_acceleration_covariance = [cv, 0.0, 0.0, 0.0, cv, 0.0, 0.0, 0.0, cv]
        self.imu_pub.publish(imu)

        arms = state.get('arms', [])
        for arm_id, (arm, publishers, topics) in enumerate(zip(arms, self.arm_publishers, self.arm_topics)):
            arrived = Bool()
            arrived.data = bool(arm.get('arrived', False))
            publishers['arrived'].publish(arrived)

            fault = Bool()
            fault.data = bool(arm.get('fault', False))
            publishers['fault'].publish(fault)

            fault_code = Int32()
            fault_code.data = int(arm.get('fault_code', 0))
            publishers['fault_code'].publish(fault_code)

            if arm_id == 0:
                self.arm_arrived_pub.publish(arrived)
                self.arm_fault_pub.publish(fault)
                self.arm_fault_code_pub.publish(fault_code)


        if state['battery_v'] != 0.0:
            bat = Float32()
            bat.data = state['battery_v']
            self.bat_pub.publish(bat)

    def destroy_node(self):
        """节点销毁：停止 RX 线程并关闭串口。"""
        self._stop_event.set()
        if self._rx_thread is not None and self._rx_thread.is_alive():
            try:
                self._rx_thread.join(timeout=1.0)
            except Exception:
                pass
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
            except Exception:
                pass
        super().destroy_node()

    def _proto_log(self, level: str, msg: str):
        if level == 'debug':
            self.get_logger().debug(msg)
        elif level == 'warn':
            self.get_logger().warn(msg)
        elif level == 'error':
            self.get_logger().error(msg)
        else:
            self.get_logger().info(msg)


def main():
    """console_scripts 入口。"""
    rclpy.init()
    node = ChassisDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # 可能已被信号处理/launch 调用 shutdown
            pass


if __name__ == '__main__':
    main()