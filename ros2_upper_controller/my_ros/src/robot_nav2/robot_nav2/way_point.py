#!/usr/bin/env python3
import argparse, math, sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

GOALS = {
    'big_table': (-2.60916, -1.17651, 3.04223),
    'trash_bin': (-2.61424, -0.782089, 2.96889),
    'small_table': (-1.85905, 0.254937, 2.3897),
}

def pose(node, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(float(yaw) / 2.0)
    p.pose.orientation.w = math.cos(float(yaw) / 2.0)
    return p

class Nav(Node):
    def __init__(self, name):
        super().__init__('waypoint_nav')
        self.name = name
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
    def start(self):
        x, y, yaw = GOALS[self.name]
        self.get_logger().info('Waiting for /navigate_to_pose...')
        self.client.wait_for_server()
        goal = NavigateToPose.Goal()
        goal.pose = pose(self, x, y, yaw)
        self.get_logger().info('Sending goal ' + self.name)
        fut = self.client.send_goal_async(goal)
        fut.add_done_callback(self.accepted)
    def accepted(self, fut):
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted')
        handle.get_result_async().add_done_callback(self.done)
    def done(self, fut):
        self.get_logger().info('Goal finished')
        rclpy.shutdown()

def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('goal', choices=sorted(GOALS))
    ns = parser.parse_args(sys.argv[1:] if args is None else args)
    rclpy.init(args=args)
    node = Nav(ns.goal)
    node.start()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
