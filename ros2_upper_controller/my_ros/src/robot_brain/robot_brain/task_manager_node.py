from __future__ import annotations

import json
from pathlib import Path

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from robot_msgs.action import PickObject


class TaskManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('task_manager_node')
        package_root = Path(__file__).resolve().parent.parent
        config_path = package_root / 'config' / 'available_actions.yaml'
        with config_path.open('r', encoding='utf-8') as file:
            self.available_actions = yaml.safe_load(file) or {}

        self.pick_client = ActionClient(self, PickObject, '/pick_object')
        self.plan_sub = self.create_subscription(String, '/task_manager/plan', self.on_plan, 10)
        self.status_pub = self.create_publisher(String, '/task/status', 10)

    def on_plan(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.publish_status('FAILED: invalid JSON plan')
            return

        plan = payload.get('plan', payload if isinstance(payload, list) else [])
        if not isinstance(plan, list) or not plan:
            self.publish_status('FAILED: empty plan')
            return

        first_step = plan[0]
        if first_step.get('action') != 'pick':
            self.publish_status('WAITING_USER: only pick is wired in the minimal skeleton')
            return

        goal = PickObject.Goal()
        goal.object_name = first_step.get('object_name', '')
        goal.arm = first_step.get('arm', 'auto')
        self.publish_status(f'EXECUTING: pick {goal.object_name}')
        if not self.pick_client.wait_for_server(timeout_sec=1.0):
            self.publish_status('FAILED: /pick_object server unavailable')
            return
        self.pick_client.send_goal_async(goal)

    def publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self.status_pub.publish(message)
        self.get_logger().info(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
