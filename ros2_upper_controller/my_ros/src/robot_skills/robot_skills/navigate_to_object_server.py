from __future__ import annotations

import asyncio

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer
from rclpy.node import Node

from robot_msgs.action import NavigateToObject


class NavigateToObjectServer(Node):
    def __init__(self) -> None:
        super().__init__('navigate_to_object_server')
        self.action_server = ActionServer(self, NavigateToObject, '/navigate_to_object', self.execute_callback)

    async def execute_callback(self, goal_handle):
        feedback = NavigateToObject.Feedback()
        feedback.current_phase = 'navigating'
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)
        await asyncio.sleep(0.2)
        goal_handle.succeed()
        result = NavigateToObject.Result()
        result.success = True
        result.failed_reason = ''
        result.goal_pose = PoseStamped()
        result.goal_pose.header.frame_id = 'map'
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigateToObjectServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
