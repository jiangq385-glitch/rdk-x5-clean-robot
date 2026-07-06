#!/usr/bin/env python3
"Simple CLI tool to publish a one-shot arm JointTrajectory command."

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


DEFAULT_JOINT_NAMES = [
    'arm_joint_1',
    'arm_joint_2',
    'arm_joint_3',
    'arm_joint_4',
    'arm_joint_5',
    'arm_joint_6',
]


def seconds_to_duration(seconds: float):
    seconds = max(0.0, float(seconds))
    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return sec, nanosec


def parse_args():
    parser = argparse.ArgumentParser(description='Publish a one-shot arm JointTrajectory command')
    parser.add_argument('--topic', default='/arm/joint_trajectory', help='JointTrajectory topic')
    parser.add_argument('--joint', help='Single joint name to move')
    parser.add_argument('--target', type=float, help='Target position in radians for --joint')
    parser.add_argument('--positions', help='Comma separated full joint position list')
    parser.add_argument('--joint-names', default=','.join(DEFAULT_JOINT_NAMES), help='Comma separated joint name list')
    parser.add_argument('--time', type=float, default=2.0, help='Move time in seconds')
    return parser.parse_args()


def build_positions(args, joint_names: list[str]) -> list[float]:
    if args.positions:
        positions = [float(value.strip()) for value in args.positions.split(',') if value.strip()]
        if len(positions) != len(joint_names):
            raise ValueError('positions length must match joint-names length')
        return positions

    if args.joint is None or args.target is None:
        raise ValueError('provide either --positions or both --joint and --target')

    positions = [0.0 for _ in joint_names]
    try:
        index = joint_names.index(args.joint)
    except ValueError as exc:
        raise ValueError(f'joint {args.joint} not in joint-names list') from exc
    positions[index] = float(args.target)
    return positions


def main(args=None):
    cli_args = parse_args()
    joint_names = [value.strip() for value in cli_args.joint_names.split(',') if value.strip()]
    positions = build_positions(cli_args, joint_names)

    rclpy.init(args=args)
    node = Node('joint_cli')
    publisher = node.create_publisher(JointTrajectory, cli_args.topic, 10)
    time.sleep(0.2)

    msg = JointTrajectory()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.joint_names = joint_names
    point = JointTrajectoryPoint()
    point.positions = positions
    sec, nanosec = seconds_to_duration(cli_args.time)
    point.time_from_start.sec = sec
    point.time_from_start.nanosec = nanosec
    msg.points = [point]
    publisher.publish(msg)
    node.get_logger().info(f'published joint trajectory to {cli_args.topic}: {positions}')

    time.sleep(0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
