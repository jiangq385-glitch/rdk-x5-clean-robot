from __future__ import annotations

import asyncio

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from my_arm_control.ik import load_arm_ik
from robot_msgs.action import CalibratePlaneHeight, PickObject, ReturnHome
from robot_msgs.msg import ObjectOnPlane, PlaneHeight


class PickObjectServer(Node):
    "Six-axis pick skill with plane-height check and return-home safety."

    def __init__(self) -> None:
        super().__init__("pick_object_server")
        self.ik = load_arm_ik()
        self.declare_parameter("trajectory_topic", "/arm/joint_trajectory")
        self.declare_parameter("object_on_plane_topic", "/perception/object_on_plane")
        self.declare_parameter("plane_height_topic", "/perception/plane_height")
        self.declare_parameter("joint_names", self.ik.joint_names)
        self.declare_parameter("home_pose", self.ik.home_pose_rad)
        self.declare_parameter("probe_pose", self.ik.probe_pose_rad)
        self.declare_parameter("wrist_roll_deg", 0.0)
        self.declare_parameter("gripper_deg", 20.0)
        self.declare_parameter("z_pre_grasp", 0.03)
        self.declare_parameter("z_grasp", 0.005)
        self.declare_parameter("z_lift", 0.06)
        self.declare_parameter("grasp_offset", 0.0)
        self.declare_parameter("wait_target_timeout_s", 3.0)
        self.declare_parameter("return_home_after_pick", True)
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.home_pose = [float(v) for v in self.get_parameter("home_pose").value]
        self.probe_pose = [float(v) for v in self.get_parameter("probe_pose").value]
        self.wrist_roll_deg = float(self.get_parameter("wrist_roll_deg").value)
        self.gripper_deg = float(self.get_parameter("gripper_deg").value)
        self.latest_target: ObjectOnPlane | None = None
        self.latest_plane_height = PlaneHeight()
        self.latest_plane_height.valid = False
        self.trajectory_pub = self.create_publisher(JointTrajectory, self.get_parameter("trajectory_topic").value, 10)
        self.create_subscription(ObjectOnPlane, self.get_parameter("object_on_plane_topic").value, self._on_target, 10)
        self.create_subscription(PlaneHeight, self.get_parameter("plane_height_topic").value, self._on_plane_height, 10)
        self.calibrate_client = ActionClient(self, CalibratePlaneHeight, "/calibrate_plane_height")
        self.return_home_client = ActionClient(self, ReturnHome, "/return_home")
        self.action_server = ActionServer(self, PickObject, "/pick_object", self.execute_callback)

    def _on_target(self, msg: ObjectOnPlane) -> None:
        self.latest_target = msg

    def _on_plane_height(self, msg: PlaneHeight) -> None:
        self.latest_plane_height = msg

    async def execute_callback(self, goal_handle):
        feedback = PickObject.Feedback()
        result = PickObject.Result()
        await self._publish_feedback(goal_handle, feedback, "return_home_before_pick", 0.05)
        await self._return_home()
        if not self.latest_plane_height.valid:
            await self._publish_feedback(goal_handle, feedback, "calibrate_plane_height", 0.2)
            await self._move_to_pose(self.probe_pose, 1.5)
            calibrated = await self._calibrate_plane_height()
            await self._return_home()
            if not calibrated:
                goal_handle.abort()
                result.success = False
                result.failed_reason = "plane height calibration failed"
                result.final_pose = PoseStamped()
                return result
        await self._publish_feedback(goal_handle, feedback, "wait_object_on_plane", 0.4)
        target = await self._wait_for_target(str(goal_handle.request.object_name))
        if target is None:
            goal_handle.abort()
            result.success = False
            result.failed_reason = "no object_on_plane target available"
            result.final_pose = PoseStamped()
            return result
        await self._publish_feedback(goal_handle, feedback, "publish_6axis_grasp", 0.7)
        trajectory = self._build_grasp_trajectory(target)
        if trajectory is None:
            goal_handle.abort()
            result.success = False
            result.failed_reason = "6-axis IK failed"
            result.final_pose = PoseStamped()
            return result
        self.trajectory_pub.publish(trajectory)
        if bool(self.get_parameter("return_home_after_pick").value):
            await self._publish_feedback(goal_handle, feedback, "return_home_after_pick", 0.9)
            await self._return_home()
        goal_handle.succeed()
        result.success = True
        result.failed_reason = ""
        result.final_pose = PoseStamped()
        result.final_pose.header.frame_id = target.header.frame_id or "base_link"
        result.final_pose.pose.position.x = float(target.x)
        result.final_pose.pose.position.y = float(target.y)
        result.final_pose.pose.position.z = float(self.latest_plane_height.z_plane_in_base)
        return result

    async def _publish_feedback(self, goal_handle, feedback, phase: str, progress: float) -> None:
        feedback.current_phase = phase
        feedback.progress = progress
        goal_handle.publish_feedback(feedback)
        await asyncio.sleep(0.05)

    async def _wait_for_target(self, object_name: str) -> ObjectOnPlane | None:
        timeout_s = float(self.get_parameter("wait_target_timeout_s").value)
        deadline = self.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while self.get_clock().now().nanoseconds < deadline:
            target = self.latest_target
            if target is not None and (not object_name or not target.class_name or target.class_name == object_name):
                return target
            await asyncio.sleep(0.05)
        return None

    async def _move_to_pose(self, positions: list[float], move_time_s: float) -> None:
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(self.joint_names)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start.sec = int(move_time_s)
        point.time_from_start.nanosec = int(round((move_time_s - int(move_time_s)) * 1e9))
        trajectory.points = [point]
        self.trajectory_pub.publish(trajectory)
        await asyncio.sleep(0.1)

    async def _return_home(self) -> None:
        if self.return_home_client.wait_for_server(timeout_sec=0.2):
            goal = ReturnHome.Goal()
            goal.immediate = False
            await self.return_home_client.send_goal_async(goal)
        else:
            await self._move_to_pose(self.home_pose, 2.0)

    async def _calibrate_plane_height(self) -> bool:
        if not self.calibrate_client.wait_for_server(timeout_sec=1.0):
            return False
        goal = CalibratePlaneHeight.Goal()
        goal.force_recalibrate = True
        goal_handle = await self.calibrate_client.send_goal_async(goal)
        if not goal_handle.accepted:
            return False
        result = await goal_handle.get_result_async()
        return bool(result.result.success)

    def _build_grasp_trajectory(self, target: ObjectOnPlane) -> JointTrajectory | None:
        z_plane = float(self.latest_plane_height.z_plane_in_base)
        grasp_offset = float(self.get_parameter("grasp_offset").value)
        heights = [
            float(self.get_parameter("z_pre_grasp").value),
            float(self.get_parameter("z_grasp").value),
            float(self.get_parameter("z_lift").value),
        ]
        durations = [2.0, 1.0, 1.0]
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(self.joint_names)
        total_time = 0.0
        for height, duration in zip(heights, durations):
            positions = self.ik.solve_radians(
                float(target.x),
                float(target.y),
                z_plane + grasp_offset + height,
                wrist_roll_deg=self.wrist_roll_deg,
                gripper_deg=self.gripper_deg,
            )
            if positions is None:
                return None
            point = JointTrajectoryPoint()
            point.positions = positions[:len(self.joint_names)]
            total_time += duration
            point.time_from_start.sec = int(total_time)
            point.time_from_start.nanosec = int(round((total_time - int(total_time)) * 1e9))
            trajectory.points.append(point)
        return trajectory


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickObjectServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
