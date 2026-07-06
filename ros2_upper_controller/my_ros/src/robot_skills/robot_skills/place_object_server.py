from __future__ import annotations

import asyncio

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer
from rclpy.node import Node

from robot_msgs.action import PlaceObject


class PlaceObjectServer(Node):
    def __init__(self) -> None:
        super().__init__('place_object_server')
        self.action_server = ActionServer(self, PlaceObject, '/place_object', self.execute_callback)

    async def execute_callback(self, goal_handle):
        feedback = PlaceObject.Feedback()
        feedback.current_phase = 'placing'
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)
        await asyncio.sleep(0.2)
        goal_handle.succeed()
        result = PlaceObject.Result()
        result.success = True
        result.failed_reason = ''
        result.final_pose = PoseStamped()
        result.final_pose.header.frame_id = 'base_link'
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlaceObjectServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
