from __future__ import annotations

import asyncio

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from robot_msgs.action import PickObject


class MockPickServer(Node):
    def __init__(self) -> None:
        super().__init__('mock_pick_server')
        self.action_server = ActionServer(self, PickObject, '/pick_object', self.execute_callback)

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'Mock pick: {goal_handle.request.object_name}')
        await asyncio.sleep(3.0)
        goal_handle.succeed()
        result = PickObject.Result()
        result.success = True
        result.failed_reason = ''
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockPickServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
