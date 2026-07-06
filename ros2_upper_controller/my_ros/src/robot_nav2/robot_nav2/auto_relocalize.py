#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from std_srvs.srv import Empty


class AutoRelocalize(Node):
    def __init__(self):
        super().__init__('auto_relocalize')

        self.declare_parameter('use_initial_pose', False)
        self.declare_parameter('initial_pose_frame', 'map')
        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0)
        self.declare_parameter('initial_pose_yaw', 0.0)
        self.declare_parameter('initial_pose_cov_x', 0.25)
        self.declare_parameter('initial_pose_cov_y', 0.25)
        self.declare_parameter('initial_pose_cov_yaw', 0.25)
        self.declare_parameter('initial_pose_publish_count', 10)
        self.declare_parameter('initial_pose_publish_period', 0.2)
        self.declare_parameter('fallback_global_localization', True)
        self.declare_parameter('spin_speed', 0.35)
        self.declare_parameter('drive_speed', 0.06)
        self.declare_parameter('motion_duration', 60.0)
        self.declare_parameter('min_motion_before_converged', 18.0)
        self.declare_parameter('converged_required_count', 3)
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('retry_pause', 2.0)
        self.declare_parameter('max_wait_after_motion', 15.0)
        self.declare_parameter('covariance_xy_threshold', 0.50)
        self.declare_parameter('covariance_yaw_threshold', 2.4)

        self.use_initial_pose = bool(self.get_parameter('use_initial_pose').value)
        self.initial_pose_frame = str(self.get_parameter('initial_pose_frame').value)
        self.initial_pose_x = float(self.get_parameter('initial_pose_x').value)
        self.initial_pose_y = float(self.get_parameter('initial_pose_y').value)
        self.initial_pose_yaw = float(self.get_parameter('initial_pose_yaw').value)
        self.initial_pose_cov_x = float(self.get_parameter('initial_pose_cov_x').value)
        self.initial_pose_cov_y = float(self.get_parameter('initial_pose_cov_y').value)
        self.initial_pose_cov_yaw = float(self.get_parameter('initial_pose_cov_yaw').value)
        self.initial_pose_publish_count = int(self.get_parameter('initial_pose_publish_count').value)
        self.initial_pose_publish_period = float(self.get_parameter('initial_pose_publish_period').value)
        self.fallback_global_localization = bool(self.get_parameter('fallback_global_localization').value)
        self.spin_speed = float(self.get_parameter('spin_speed').value)
        self.drive_speed = float(self.get_parameter('drive_speed').value)
        self.min_motion_before_converged = float(self.get_parameter('min_motion_before_converged').value)
        self.converged_required_count = int(self.get_parameter('converged_required_count').value)
        self.motion_duration = float(self.get_parameter('motion_duration').value)
        self.max_retries = int(self.get_parameter('max_retries').value)
        self.retry_pause = float(self.get_parameter('retry_pause').value)
        self.max_wait_after_motion = float(self.get_parameter('max_wait_after_motion').value)
        self.cov_xy_threshold = float(self.get_parameter('covariance_xy_threshold').value)
        self.cov_yaw_threshold = float(self.get_parameter('covariance_yaw_threshold').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._on_amcl_pose,
            10,
        )
        self.global_loc_client = self.create_client(Empty, '/reinitialize_global_localization')

        self.latest_pose = None
        self.latest_pose_receive_time = None
        self.node_start = self.get_clock().now()
        self.request_start = None
        self.motion_start = None
        self.wait_start = None
        self.retry_start = None
        self.reinit_future = None
        self.last_service_log = self.node_start
        self.last_covariance_log = self.node_start
        self.last_initial_pose_publish = None
        self.initial_pose_publish_left = max(1, self.initial_pose_publish_count)
        self.converged_count = 0
        self.retry_count = 0
        self.used_initial_pose = False

        self.state = 'publishing_initial_pose' if self.use_initial_pose else 'waiting_service'
        self.timer = self.create_timer(0.1, self._tick)

    def _on_amcl_pose(self, msg):
        self.latest_pose = msg
        self.latest_pose_receive_time = self.get_clock().now()

    def _tick(self):
        if self.state in ('done', 'failed'):
            return

        if self._elapsed_since(self.node_start) < 2.0:
            return

        if self.state == 'publishing_initial_pose':
            self._publish_initial_pose_until_done()
        elif self.state == 'waiting_service':
            self._start_global_localization_when_ready()
        elif self.state == 'waiting_response':
            self._check_global_localization_timeout()
        elif self.state == 'moving':
            self._continue_motion()
        elif self.state == 'waiting_convergence':
            if self._check_convergence(allow_stale_warning=True):
                self._stop_robot()
                self.state = 'done'
        elif self.state == 'retry_pause':
            if self._elapsed_since(self.retry_start) >= self.retry_pause:
                self.state = 'waiting_service'

    def _publish_initial_pose_until_done(self):
        if (
            self.last_initial_pose_publish is not None
            and self._elapsed_since(self.last_initial_pose_publish) < self.initial_pose_publish_period
        ):
            return

        self.initial_pose_pub.publish(self._make_initial_pose_msg())
        self.last_initial_pose_publish = self.get_clock().now()
        self.initial_pose_publish_left -= 1

        if self.initial_pose_publish_left == max(0, self.initial_pose_publish_count - 1):
            self.get_logger().info(
                'publishing /initialpose '
                f'x={self.initial_pose_x:.3f}, y={self.initial_pose_y:.3f}, '
                f'yaw={self.initial_pose_yaw:.3f} rad in {self.initial_pose_frame}'
            )

        if self.initial_pose_publish_left > 0:
            return

        self.used_initial_pose = True
        self.wait_start = self.get_clock().now()
        self.last_covariance_log = self.wait_start
        self.get_logger().info('initial pose published; waiting for AMCL convergence')
        self.state = 'waiting_convergence'

    def _make_initial_pose_msg(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.initial_pose_frame
        msg.pose.pose.position.x = self.initial_pose_x
        msg.pose.pose.position.y = self.initial_pose_y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(self.initial_pose_yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.initial_pose_yaw / 2.0)
        msg.pose.covariance[0] = self.initial_pose_cov_x
        msg.pose.covariance[7] = self.initial_pose_cov_y
        msg.pose.covariance[35] = self.initial_pose_cov_yaw
        return msg

    def _start_global_localization_when_ready(self):
        if not self.global_loc_client.service_is_ready():
            if self._elapsed_since(self.last_service_log) >= 5.0:
                self.get_logger().info('waiting for /reinitialize_global_localization ...')
                self.last_service_log = self.get_clock().now()
            return

        self.retry_count += 1
        self.get_logger().info(
            f'calling /reinitialize_global_localization, attempt {self.retry_count}/{self.max_retries}'
        )
        self.reinit_future = self.global_loc_client.call_async(Empty.Request())
        self.reinit_future.add_done_callback(self._on_global_localization_done)
        self.request_start = self.get_clock().now()
        self.state = 'waiting_response'

    def _check_global_localization_timeout(self):
        if self.reinit_future is None or self.reinit_future.done():
            return

        if self._elapsed_since(self.request_start) >= 10.0:
            self.get_logger().error('timed out calling /reinitialize_global_localization')
            self._stop_robot()
            self._schedule_retry_or_fail()

    def _on_global_localization_done(self, future):
        if self.state != 'waiting_response':
            return

        try:
            future.result()
        except Exception as exc:
            self.get_logger().error(
                f'failed to call /reinitialize_global_localization: {exc}'
            )
            self._stop_robot()
            self._schedule_retry_or_fail()
            return

        self.get_logger().info('moving gently and checking AMCL convergence')
        self.motion_start = self.get_clock().now()
        self.converged_count = 0
        self.last_covariance_log = self.motion_start
        self.state = 'moving'

    def _continue_motion(self):

        elapsed = self._elapsed_since(self.motion_start)
        if elapsed >= self.motion_duration:
            self._stop_robot()
            self.get_logger().info('motion finished; waiting briefly for final AMCL update')
            self.wait_start = self.get_clock().now()
            self.last_covariance_log = self.wait_start
            self.state = 'waiting_convergence'
            return
        if elapsed >= self.min_motion_before_converged and self._check_convergence(allow_stale_warning=False):
            self.converged_count += 1
            if self.converged_count >= self.converged_required_count:
                self._stop_robot()
                self.state = 'done'
                return
        else:
            self.converged_count = 0


        self.cmd_pub.publish(self._motion_cmd(elapsed))

    def _motion_cmd(self, elapsed):
        twist = Twist()
        phase = int(elapsed // 6.0) % 6
        if phase == 0:
            twist.angular.z = self.spin_speed
        elif phase == 1:
            twist.linear.x = self.drive_speed
            twist.angular.z = self.spin_speed * 0.45
        elif phase == 2:
            twist.angular.z = -self.spin_speed
        elif phase == 3:
            twist.linear.x = -self.drive_speed * 0.6
            twist.angular.z = -self.spin_speed * 0.45
        elif phase == 4:
            twist.linear.x = self.drive_speed
        else:
            twist.angular.z = self.spin_speed * 0.7
        return twist

    def _check_convergence(self, allow_stale_warning):
        if self.latest_pose is not None:
            cov = self.latest_pose.pose.covariance
            cov_x = cov[0]
            cov_y = cov[7]
            cov_yaw = cov[35]
            pose_age = self._elapsed_since(self.latest_pose_receive_time)

            if self._elapsed_since(self.last_covariance_log) >= 1.0:
                self.get_logger().info(
                    f'AMCL covariance x={cov_x:.4f}, y={cov_y:.4f}, yaw={cov_yaw:.4f}, pose_age={pose_age:.1f}s'
                )
                self.last_covariance_log = self.get_clock().now()

            if (
                pose_age <= 2.0
                and cov_x < self.cov_xy_threshold
                and cov_y < self.cov_xy_threshold
                and cov_yaw < self.cov_yaw_threshold
            ):
                self.get_logger().info('AMCL covariance is below threshold')
                return True

            if allow_stale_warning and pose_age > 2.0 and self._elapsed_since(self.last_covariance_log) >= 2.0:
                self.get_logger().warn(
                    f'/amcl_pose has not updated for {pose_age:.1f}s; check AMCL scan/TF matching'
                )
        elif self._elapsed_since(self.last_covariance_log) >= 1.0:
            self.get_logger().warn('waiting for first /amcl_pose message')
            self.last_covariance_log = self.get_clock().now()

        if self.state == 'waiting_convergence' and self._elapsed_since(self.wait_start) >= self.max_wait_after_motion:
            self._stop_robot()
            if self.used_initial_pose and self.fallback_global_localization:
                self.get_logger().warn('initial pose did not converge; falling back to global localization')
                self.used_initial_pose = False
                self.state = 'waiting_service'
                return False

            self.get_logger().warn('AMCL covariance did not converge before timeout')
            self._schedule_retry_or_fail()

        return False

    def _schedule_retry_or_fail(self):
        self._stop_robot()
        if self.retry_count < self.max_retries:
            self.get_logger().warn('retrying global localization')
            self.retry_start = self.get_clock().now()
            self.state = 'retry_pause'
        else:
            self.get_logger().error('global localization failed after all attempts')
            self.state = 'failed'

    def _elapsed_since(self, start_time):
        if start_time is None:
            return 0.0
        return (self.get_clock().now() - start_time).nanoseconds / 1e9

    def _stop_robot(self):
        stop = Twist()
        for _ in range(10):
            self.cmd_pub.publish(stop)


def main(args=None):
    rclpy.init(args=args)
    node = AutoRelocalize()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

