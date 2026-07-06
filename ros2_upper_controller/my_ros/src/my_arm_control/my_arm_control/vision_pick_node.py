#!/usr/bin/env python3
"Build six-axis grasp trajectories from work-plane targets."

from __future__ import annotations

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from my_arm_control.ik import load_arm_ik
from robot_msgs.msg import ObjectOnPlane, PlaneHeight


class VisionPickNode(Node):
    "Listen for plane targets and publish six-axis grasp trajectories."

    def __init__(self):
        super().__init__("vision_pick_node")
        self.ik = load_arm_ik()
        self.declare_parameter("object_on_plane_topic", "/perception/object_on_plane")
        self.declare_parameter("plane_height_topic", "/perception/plane_height")
        self.declare_parameter("trajectory_topic", "/arm/joint_trajectory")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("joint_names", self.ik.joint_names)
        self.declare_parameter("home_pose", self.ik.home_pose_rad)
        self.declare_parameter("z_pre_grasp", 0.03)
        self.declare_parameter("z_grasp", 0.005)
        self.declare_parameter("z_lift", 0.06)
        self.declare_parameter("grasp_offset", 0.0)
        self.declare_parameter("wrist_roll_deg", 0.0)
        self.declare_parameter("gripper_deg", 20.0)
        self.declare_parameter("approach_time_s", 2.0)
        self.declare_parameter("grasp_time_s", 1.0)
        self.declare_parameter("lift_time_s", 1.0)
        self.declare_parameter("return_home_time_s", 2.0)
        self.declare_parameter("require_plane_height", True)
        self.declare_parameter("auto_execute", True)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.home_pose = [float(v) for v in self.get_parameter("home_pose").value]
        self.z_pre_grasp = float(self.get_parameter("z_pre_grasp").value)
        self.z_grasp = float(self.get_parameter("z_grasp").value)
        self.z_lift = float(self.get_parameter("z_lift").value)
        self.grasp_offset = float(self.get_parameter("grasp_offset").value)
        self.wrist_roll_deg = float(self.get_parameter("wrist_roll_deg").value)
        self.gripper_deg = float(self.get_parameter("gripper_deg").value)
        self.approach_time_s = float(self.get_parameter("approach_time_s").value)
        self.grasp_time_s = float(self.get_parameter("grasp_time_s").value)
        self.lift_time_s = float(self.get_parameter("lift_time_s").value)
        self.return_home_time_s = float(self.get_parameter("return_home_time_s").value)
        self.require_plane_height = bool(self.get_parameter("require_plane_height").value)
        self.auto_execute = bool(self.get_parameter("auto_execute").value)
        self.latest_plane_height = PlaneHeight()
        self.latest_plane_height.valid = False
        self.trajectory_pub = self.create_publisher(JointTrajectory, self.get_parameter("trajectory_topic").value, 10)
        self.status_pub = self.create_publisher(String, "/arm/vision_pick_status", 10)
        self.create_subscription(PlaneHeight, self.get_parameter("plane_height_topic").value, self._on_plane_height, 10)
        self.create_subscription(ObjectOnPlane, self.get_parameter("object_on_plane_topic").value, self._on_object_on_plane, 10)

    def _on_plane_height(self, msg: PlaneHeight):
        self.latest_plane_height = msg

    def _on_object_on_plane(self, msg: ObjectOnPlane):
        if msg.header.frame_id and msg.header.frame_id != self.target_frame:
            self._publish_status("ignored", f"unexpected frame: {msg.header.frame_id}")
            return
        if self.require_plane_height and not self.latest_plane_height.valid:
            self._publish_status("waiting_for_plane_height", "plane height is not calibrated")
            return
        z_plane = float(self.latest_plane_height.z_plane_in_base) if self.latest_plane_height.valid else 0.0
        phases = [
            ("pre_grasp", float(msg.x), float(msg.y), z_plane + self.grasp_offset + self.z_pre_grasp, self.approach_time_s),
            ("grasp", float(msg.x), float(msg.y), z_plane + self.grasp_offset + self.z_grasp, self.grasp_time_s),
            ("lift", float(msg.x), float(msg.y), z_plane + self.grasp_offset + self.z_lift, self.lift_time_s),
        ]
        trajectory = self._build_trajectory(phases)
        if trajectory is None:
            return
        self._append_point(trajectory, self.home_pose, self.return_home_time_s)
        if self.auto_execute:
            self.trajectory_pub.publish(trajectory)
            self._publish_status("published", f"published six-axis grasp for {msg.class_name or  target}")
        else:
            self._publish_status("planned", "planned six-axis grasp but auto_execute=false")

    def _build_trajectory(self, phases: list[tuple[str, float, float, float, float]]) -> JointTrajectory | None:
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(self.joint_names)
        total_time = 0.0
        for phase, x, y, z_axis, duration_s in phases:
            positions = self.ik.solve_radians(x, y, z_axis, wrist_roll_deg=self.wrist_roll_deg, gripper_deg=self.gripper_deg)
            if positions is None:
                self._publish_status("ik_failed", f"{phase} has no six-axis IK solution for ({x:.3f}, {y:.3f}, {z_axis:.3f})")
                return None
            total_time += duration_s
            self._append_point(trajectory, positions[:len(self.joint_names)], total_time, absolute=True)
        return trajectory

    def _append_point(self, trajectory: JointTrajectory, positions: list[float], duration_s: float, absolute: bool = False):
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        if len(point.positions) != len(trajectory.joint_names):
            raise ValueError("trajectory point length does not match joint_names")
        total_time = duration_s if absolute else self._last_time_s(trajectory) + duration_s
        point.time_from_start.sec = int(total_time)
        point.time_from_start.nanosec = int(round((total_time - int(total_time)) * 1e9))
        trajectory.points.append(point)

    def _last_time_s(self, trajectory: JointTrajectory) -> float:
        if not trajectory.points:
            return 0.0
        last = trajectory.points[-1].time_from_start
        return float(last.sec) + float(last.nanosec) * 1e-9

    def _publish_status(self, state: str, message: str):
        payload = {"state": state, "message": message, "timestamp_ns": self.get_clock().now().nanoseconds}
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = VisionPickNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
