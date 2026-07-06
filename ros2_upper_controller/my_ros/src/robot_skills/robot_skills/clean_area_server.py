from __future__ import annotations

import asyncio

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from robot_msgs.action import CleanArea


class CleanAreaServer(Node):
    def __init__(self) -> None:
        super().__init__('clean_area_server')
        self.action_server = ActionServer(self, CleanArea, '/clean_area', self.execute_callback)

    async def execute_callback(self, goal_handle):
        feedback = CleanArea.Feedback()
        feedback.current_phase = 'cleaning'
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)
        await asyncio.sleep(0.2)
        goal_handle.succeed()
        result = CleanArea.Result()
        result.success = True
        result.failed_reason = ''
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CleanAreaServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
