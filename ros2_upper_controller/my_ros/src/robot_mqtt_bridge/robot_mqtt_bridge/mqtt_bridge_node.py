#!/usr/bin/env python3
"MQTT <-> ROS2 bridge for the mini program control channel."

import json
import ssl
import time
import uuid

import paho.mqtt.client as mqtt
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, String


def now_ms() -> int:
    return int(time.time() * 1000)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class RobotMqttBridge(Node):
    "Translate mini program MQTT commands into ROS2 messages."

    def __init__(self):
        super().__init__('robot_mqtt_bridge')

        self.declare_parameter('robot_id', 'digua-x5-001')
        self.declare_parameter('mqtt_host', '127.0.0.1')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_username', '')
        self.declare_parameter('mqtt_password', '')
        self.declare_parameter('mqtt_use_tls', False)
        self.declare_parameter('mqtt_keepalive_s', 30)
        self.declare_parameter('topic_prefix', '')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('robot_status_topic', '/robot_status')
        self.declare_parameter('robot_telemetry_topic', '/robot_telemetry')
        self.declare_parameter('battery_topic', '/battery_voltage')
        self.declare_parameter('max_linear_x', 0.3)
        self.declare_parameter('max_angular_z', 1.2)
        self.declare_parameter('status_period_s', 1.0)
        self.declare_parameter('manual_timeout_s', 0.8)
        self.declare_parameter('min_command_interval_s', 0.05)
        self.declare_parameter('clean_area_command_topic', '/robot_test/clean_area_command')

        self.robot_id = self.get_parameter('robot_id').value
        self.topic_prefix = self.get_parameter('topic_prefix').value or f'robot/{self.robot_id}'
        self.cmd_topic = f'{self.topic_prefix}/cmd'
        self.status_topic = f'{self.topic_prefix}/status'
        self.telemetry_topic = f'{self.topic_prefix}/telemetry'
        self.ack_topic = f'{self.topic_prefix}/ack'

        self.max_linear_x = float(self.get_parameter('max_linear_x').value)
        self.max_angular_z = float(self.get_parameter('max_angular_z').value)
        self.manual_timeout_s = float(self.get_parameter('manual_timeout_s').value)
        self.min_command_interval_s = float(self.get_parameter('min_command_interval_s').value)
        self._last_command_time = 0.0
        self._last_motion_command_time = 0.0
        self._started_monotonic = time.monotonic()
        self._battery_voltage = None
        self._connected = False

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10,
        )
        self.clean_area_pub = self.create_publisher(
            String,
            self.get_parameter('clean_area_command_topic').value,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter('robot_status_topic').value,
            self._on_ros_status,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter('robot_telemetry_topic').value,
            self._on_ros_telemetry,
            10,
        )
        self.create_subscription(
            Float32,
            self.get_parameter('battery_topic').value,
            self._on_battery,
            10,
        )

        self.mqtt_client = self._create_mqtt_client()
        self._connect_mqtt()

        self.create_timer(float(self.get_parameter('status_period_s').value), self._publish_status)
        self.create_timer(0.1, self._manual_deadman_check)

    def _create_mqtt_client(self):
        client_id = f'ros2-{self.robot_id}-{uuid.uuid4().hex[:8]}'
        try:
            client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
        except AttributeError:
            client = mqtt.Client(client_id=client_id)

        username = self.get_parameter('mqtt_username').value
        password = self.get_parameter('mqtt_password').value
        if username:
            client.username_pw_set(username, password=password or None)

        if bool(self.get_parameter('mqtt_use_tls').value):
            client.tls_set(tls_version=ssl.PROTOCOL_TLS_CLIENT)

        offline_payload = json.dumps({
            'robot_id': self.robot_id,
            'online': False,
            'status': 'offline',
            'ts': now_ms(),
        })
        client.will_set(self.status_topic, offline_payload, qos=1, retain=True)
        client.on_connect = self._on_mqtt_connect
        client.on_disconnect = self._on_mqtt_disconnect
        client.on_message = self._on_mqtt_message
        return client

    def _connect_mqtt(self):
        host = self.get_parameter('mqtt_host').value
        port = int(self.get_parameter('mqtt_port').value)
        keepalive = int(self.get_parameter('mqtt_keepalive_s').value)
        self.get_logger().info(f'Connecting MQTT broker {host}:{port}, cmd topic: {self.cmd_topic}')
        self.mqtt_client.connect_async(host, port, keepalive)
        self.mqtt_client.loop_start()

    def _on_mqtt_connect(self, client, userdata, flags, reason_code, properties=None):
        if not self._is_success_reason(reason_code):
            self._connected = False
            self.get_logger().warn(f'MQTT connection rejected: {reason_code}')
            return

        self._connected = True
        self.get_logger().info(f'MQTT connected: {reason_code}')
        client.subscribe(self.cmd_topic, qos=1)
        self._publish_status()

    def _on_mqtt_disconnect(self, client, userdata, *args):
        self._connected = False
        reason_code = args[-2] if len(args) >= 2 else args[-1] if args else 'unknown'
        self.get_logger().warn(f'MQTT disconnected: {reason_code}')
        self._publish_zero_cmd_vel()

    def _on_mqtt_message(self, client, userdata, msg):
        try:
            command = json.loads(msg.payload.decode('utf-8'))
        except Exception as exc:
            self.get_logger().warn(f'Invalid MQTT JSON on {msg.topic}: {exc}')
            self._publish_ack(None, 'rejected', f'invalid json: {exc}')
            return

        command_id = command.get('command_id')
        command_type = command.get('type')
        try:
            self._handle_command(command)
        except Exception as exc:
            self.get_logger().error(f'Command failed: {exc}')
            self._publish_ack(command_id, 'rejected', f'{command_type} failed: {exc}')

    def _handle_command(self, command: dict):
        command_id = command.get('command_id')
        command_type = command.get('type')

        if not command_id:
            self._publish_ack(None, 'rejected', 'missing command_id')
            return
        if not command_type:
            self._publish_ack(command_id, 'rejected', 'missing type')
            return

        expire_at = int(command.get('expire_at') or 0)
        if expire_at and expire_at < now_ms():
            self._publish_ack(command_id, 'rejected', 'command expired')
            return

        now = time.monotonic()
        if command_type != 'emergency_stop' and now - self._last_command_time < self.min_command_interval_s:
            self._publish_ack(command_id, 'rejected', 'command rate limited')
            return
        self._last_command_time = now

        if command_type == 'ping':
            self._publish_ack(command_id, 'accepted', 'pong', {'robot_time_ms': now_ms()})
        elif command_type == 'cmd_vel':
            self._handle_cmd_vel(command_id, command.get('payload') or {})
        elif command_type == 'stop':
            self._publish_zero_cmd_vel()
            self._publish_ack(command_id, 'accepted', 'stopped')
        elif command_type in ('clean_area', 'start_clean_task'):
            self._handle_clean_area(command_id, command_type, command.get('payload') or {})
        elif command_type == 'emergency_stop':
            self._publish_zero_cmd_vel()
            self._publish_ack(command_id, 'accepted', 'emergency stop accepted')
            self._publish_status(status='estop')
        else:
            self._publish_ack(command_id, 'rejected', f'unsupported command type: {command_type}')

    def _handle_clean_area(self, command_id: str, command_type: str, payload: dict):
        area = (
            payload.get('area')
            or payload.get('target_area')
            or payload.get('target')
            or payload.get('room')
            or ''
        )
        rooms = payload.get('rooms')
        if not area and isinstance(rooms, list) and rooms:
            area = rooms[0]

        clean_msg = String()
        clean_msg.data = json.dumps({
            'command_id': command_id,
            'type': command_type,
            'area': area,
            'payload': payload,
            'ts': now_ms(),
        }, ensure_ascii=False)
        self.clean_area_pub.publish(clean_msg)
        self._publish_ack(command_id, 'accepted', f'clean area task forwarded: {area or "unknown"}', {
            'area': area,
            'ros_topic': self.get_parameter('clean_area_command_topic').value,
        })

    def _handle_cmd_vel(self, command_id: str, payload: dict):
        linear_x = float(payload.get('linear_x', 0.0))
        angular_z = float(payload.get('angular_z', 0.0))
        clipped_linear_x = clamp(linear_x, -self.max_linear_x, self.max_linear_x)
        clipped_angular_z = clamp(angular_z, -self.max_angular_z, self.max_angular_z)

        twist = Twist()
        twist.linear.x = clipped_linear_x
        twist.angular.z = clipped_angular_z
        self.cmd_vel_pub.publish(twist)
        self._last_motion_command_time = time.monotonic()

        if clipped_linear_x != linear_x or clipped_angular_z != angular_z:
            message = 'cmd_vel published with safety clipping'
        else:
            message = 'cmd_vel published'
        self._publish_ack(command_id, 'accepted', message, {
            'linear_x': clipped_linear_x,
            'angular_z': clipped_angular_z,
        })

    def _publish_zero_cmd_vel(self):
        self.cmd_vel_pub.publish(Twist())
        self._last_motion_command_time = 0.0

    def _manual_deadman_check(self):
        if self._last_motion_command_time <= 0.0:
            return
        if time.monotonic() - self._last_motion_command_time > self.manual_timeout_s:
            self._publish_zero_cmd_vel()

    def _publish_status(self, status='online'):
        payload = {
            'robot_id': self.robot_id,
            'online': self._connected,
            'status': status,
            'ts': now_ms(),
        }
        uptime_seconds = max(0, int(time.monotonic() - self._started_monotonic))
        payload.setdefault('uptimeSeconds', uptime_seconds)
        payload.setdefault('uptimeMinutes', uptime_seconds // 60)
        if self._battery_voltage is not None:
            payload['battery_voltage'] = self._battery_voltage
        self._mqtt_publish(self.status_topic, payload, retain=True)

    def _on_ros_status(self, msg: String):
        payload = self._load_json_or_wrap(msg.data, 'status')
        payload.setdefault('robot_id', self.robot_id)
        payload.setdefault('ts', now_ms())
        uptime_seconds = max(0, int(time.monotonic() - self._started_monotonic))
        payload.setdefault('uptimeSeconds', uptime_seconds)
        payload.setdefault('uptimeMinutes', uptime_seconds // 60)
        self._mqtt_publish(self.status_topic, payload, retain=True)

    def _on_ros_telemetry(self, msg: String):
        payload = self._load_json_or_wrap(msg.data, 'telemetry')
        payload.setdefault('robot_id', self.robot_id)
        payload.setdefault('ts', now_ms())
        uptime_seconds = max(0, int(time.monotonic() - self._started_monotonic))
        payload.setdefault('uptimeSeconds', uptime_seconds)
        payload.setdefault('uptimeMinutes', uptime_seconds // 60)
        self._mqtt_publish(self.telemetry_topic, payload)

    def _on_battery(self, msg: Float32):
        self._battery_voltage = round(float(msg.data), 3)

    def _load_json_or_wrap(self, data: str, key: str):
        try:
            value = json.loads(data)
            if isinstance(value, dict):
                return value
        except Exception:
            pass
        return {key: data}

    def _publish_ack(self, command_id, status: str, message: str, extra=None):
        payload = {
            'command_id': command_id,
            'status': status,
            'message': message,
            'ts': now_ms(),
        }
        if extra:
            payload.update(extra)
        self._mqtt_publish(self.ack_topic, payload)

    def _mqtt_publish(self, topic: str, payload: dict, retain=False):
        try:
            self.mqtt_client.publish(
                topic,
                json.dumps(payload, ensure_ascii=False),
                qos=1,
                retain=retain,
            )
        except Exception as exc:
            self.get_logger().warn(f'MQTT publish failed on {topic}: {exc}')

    def _is_success_reason(self, reason_code):
        if reason_code == 0:
            return True
        return str(reason_code).lower() in ('success', 'normal disconnection')

    def destroy_node(self):
        try:
            if rclpy.ok():
                self._publish_zero_cmd_vel()
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotMqttBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
