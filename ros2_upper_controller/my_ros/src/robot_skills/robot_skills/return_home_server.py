from __future__ import annotations

import asyncio

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from my_arm_control.ik import load_arm_ik
from robot_msgs.action import ReturnHome


class ReturnHomeServer(Node):
    "System safety action: always publish the configured six-axis home pose."

    def __init__(self) -> None:
        super().__init__("return_home_server")
        ik = load_arm_ik()
        self.declare_parameter("trajectory_topic", "/arm/joint_trajectory")
        self.declare_parameter("joint_names", ik.joint_names)
        self.declare_parameter("home_pose", ik.home_pose_rad)
        self.declare_parameter("move_time_s", 2.0)
        self.trajectory_pub = self.create_publisher(JointTrajectory, self.get_parameter("trajectory_topic").value, 10)
        self.action_server = ActionServer(self, ReturnHome, "/return_home", self.execute_callback)

    async def execute_callback(self, goal_handle):
        feedback = ReturnHome.Feedback()
        feedback.current_phase = "publishing_home_pose"
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(self.get_parameter("joint_names").value)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in self.get_parameter("home_pose").value]
        move_time_s = float(self.get_parameter("move_time_s").value)
        point.time_from_start.sec = int(move_time_s)
        point.time_from_start.nanosec = int(round((move_time_s - int(move_time_s)) * 1e9))
        trajectory.points = [point]
        self.trajectory_pub.publish(trajectory)
        await asyncio.sleep(0.1)
        feedback.current_phase = "home_command_sent"
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        result = ReturnHome.Result()
        result.success = True
        result.failed_reason = ""
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReturnHomeServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
