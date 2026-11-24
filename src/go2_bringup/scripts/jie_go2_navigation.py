#!/usr/bin/env python3
"""Bridge RViz goals and jie_3d_nav path tracking into the Go2 command mux."""

import copy

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Empty
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener


class JieGo2Navigation(Node):
    def __init__(self):
        super().__init__('jie_go2_navigation')
        self.world_frame = self.declare_parameter('world_frame', 'world').value
        self.base_frame = self.declare_parameter('base_frame', 'base').value
        self.goal_input_topic = self.declare_parameter('goal_input_topic', '/goal_pose').value
        self.clicked_point_topic = self.declare_parameter(
            'clicked_point_topic', '/clicked_point').value
        self.plan_start_topic = self.declare_parameter('plan_start_topic', '/jie/start_point').value
        self.plan_goal_topic = self.declare_parameter('plan_goal_topic', '/jie/goal_point').value
        self.plan_goal_pose_topic = self.declare_parameter(
            'plan_goal_pose_topic', '/jie/goal_pose').value
        self.planner_reset_topic = self.declare_parameter(
            'planner_reset_topic', '/jie/reset').value
        self.path_topic = self.declare_parameter(
            'path_topic', '/jie/planned_path').value
        self.controller_active_topic = self.declare_parameter(
            'controller_active_topic', '/jie/controller_active').value
        self.start_navigation_topic = self.declare_parameter(
            'start_navigation_topic', '/jie/start_navigation').value
        self.stop_navigation_topic = self.declare_parameter(
            'stop_navigation_topic', '/jie/stop_navigation').value
        self.navigation_mode_service = self.declare_parameter(
            'navigation_mode_service', '/go2/set_navigation').value

        if not self.world_frame or not self.base_frame:
            raise ValueError('world_frame and base_frame must not be empty')

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.start_pub = self.create_publisher(PointStamped, self.plan_start_topic, latched)
        self.goal_pub = self.create_publisher(PointStamped, self.plan_goal_topic, latched)
        self.goal_pose_pub = self.create_publisher(
            PoseStamped, self.plan_goal_pose_topic, latched)
        self.reset_pub = self.create_publisher(Empty, self.planner_reset_topic, 10)
        self.start_navigation_pub = self.create_publisher(
            Bool, self.start_navigation_topic, 10)
        self.stop_navigation_pub = self.create_publisher(
            Bool, self.stop_navigation_topic, 10)
        self.create_subscription(
            PoseStamped, self.goal_input_topic, self.on_goal, 10)
        self.create_subscription(
            PointStamped, self.clicked_point_topic, self.on_clicked_point, 10)
        self.create_subscription(
            Path, self.path_topic, self.on_path, latched)
        self.create_subscription(
            Bool, self.controller_active_topic, self.on_controller_active, latched)
        self.mode_client = self.create_client(SetBool, self.navigation_mode_service)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.generation = 0
        self.awaiting_path = False
        self.navigation_mode = False
        self.tracker_active = False
        self.mode_future = None
        self.pending_mode = None
        self.pending_goal_timer = None
        self.reset_timer = None
        self.pending_goal_data = None
        self.create_timer(0.1, self.retry_mode_request)
        self.get_logger().info(
            f'Go2 jie navigation ready: goal={self.goal_input_topic}, '
            f'path={self.path_topic}, base={self.base_frame}, frame={self.world_frame}')

    def on_clicked_point(self, msg):
        goal = PoseStamped()
        goal.header = msg.header
        goal.pose.position = copy.deepcopy(msg.point)
        goal.pose.orientation.w = 1.0
        self.on_goal(goal)

    def on_goal(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.world_frame:
            self.get_logger().error(
                f'Goal frame {msg.header.frame_id!r} must be {self.world_frame!r}')
            return

        self.generation += 1
        generation = self.generation
        self.awaiting_path = True
        self.tracker_active = False
        self.stop_navigation_pub.publish(Bool(data=True))
        self.start_navigation_pub.publish(Bool(data=False))

        if self.navigation_mode or self.mode_future or self.pending_mode:
            self.request_mode(False, lambda: self.dispatch_goal(msg, generation))
        else:
            self.dispatch_goal(msg, generation)

    def dispatch_goal(self, goal, generation):
        if generation != self.generation:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame, self.base_frame, Time(),
                timeout=Duration(seconds=0.5))
        except TransformException as exc:
            self.awaiting_path = False
            self.get_logger().error(f'Cannot read Go2 pose for planning: {exc}')
            return

        stamp = self.get_clock().now().to_msg()
        goal_pose = copy.deepcopy(goal)
        goal_pose.header.frame_id = self.world_frame
        goal_pose.header.stamp = stamp

        start = PointStamped()
        start.header.frame_id = self.world_frame
        start.header.stamp = stamp
        start.point.x = transform.transform.translation.x
        start.point.y = transform.transform.translation.y
        start.point.z = transform.transform.translation.z

        target = PointStamped()
        target.header = start.header
        target.point = copy.deepcopy(goal_pose.pose.position)

        # Clear the planner's previous start and goal before sending a new request.
        self.pending_goal_data = (goal_pose, start, target, generation)
        self.reset_pub.publish(Empty())
        if self.pending_goal_timer is not None:
            self.pending_goal_timer.cancel()
            self.destroy_timer(self.pending_goal_timer)
            self.pending_goal_timer = None
        if self.reset_timer is not None:
            self.reset_timer.cancel()
            self.destroy_timer(self.reset_timer)
        self.reset_timer = self.create_timer(0.1, self.publish_goal_pose_and_start)
        self.get_logger().info(
            f'Planning from ({start.point.x:.2f}, {start.point.y:.2f}, '
            f'{start.point.z:.2f}) to ({target.point.x:.2f}, '
            f'{target.point.y:.2f}, {target.point.z:.2f})')

    def publish_goal_pose_and_start(self):
        if self.reset_timer is not None:
            self.reset_timer.cancel()
            self.destroy_timer(self.reset_timer)
            self.reset_timer = None
        data = self.pending_goal_data
        self.pending_goal_data = None
        if data is None:
            return
        goal_pose, start, target, generation = data
        if generation != self.generation or not self.awaiting_path:
            return
        # Set terminal orientation before the point callback starts A*.
        self.goal_pose_pub.publish(goal_pose)
        self.start_pub.publish(start)
        self.pending_goal_timer = self.create_timer(
            0.15, lambda: self.publish_goal_point_once(target, generation))

    def publish_goal_point_once(self, target, generation):
        if self.pending_goal_timer is not None:
            self.pending_goal_timer.cancel()
            self.destroy_timer(self.pending_goal_timer)
            self.pending_goal_timer = None
        if generation == self.generation and self.awaiting_path:
            self.goal_pub.publish(target)

    def on_path(self, msg):
        if not msg.poses:
            if self.awaiting_path or self.navigation_mode:
                self.stop_navigation_pub.publish(Bool(data=True))
            if self.navigation_mode and not self.awaiting_path:
                self.request_mode(False)
            return
        if not self.awaiting_path:
            return

        generation = self.generation
        self.awaiting_path = False
        self.request_mode(
            True,
            lambda: self.start_path_tracking(generation))

    def start_path_tracking(self, generation):
        if generation != self.generation:
            return
        self.start_navigation_pub.publish(Bool(data=True))
        self.get_logger().info('Go2 path tracking enabled')

    def on_controller_active(self, msg):
        was_active = self.tracker_active
        self.tracker_active = msg.data
        if was_active and not msg.data and self.navigation_mode and not self.awaiting_path:
            self.request_mode(False)
            self.get_logger().info('Path tracker stopped; Go2 command mux returned to teleop')

    def request_mode(self, enabled, callback=None):
        if (self.mode_future is None and self.pending_mode is None
                and self.navigation_mode == enabled):
            if callback is not None:
                callback()
            return
        self.pending_mode = (enabled, callback)
        self.retry_mode_request()

    def retry_mode_request(self):
        if self.mode_future is not None or self.pending_mode is None:
            return
        if not self.mode_client.service_is_ready():
            return

        enabled, callback = self.pending_mode
        self.pending_mode = None
        request = SetBool.Request()
        request.data = enabled
        self.mode_future = self.mode_client.call_async(request)
        self.mode_future.add_done_callback(
            lambda future: self.mode_request_done(future, enabled, callback))

    def mode_request_done(self, future, enabled, callback):
        self.mode_future = None
        try:
            response = future.result()
        except Exception as exc:  # rclpy surfaces service errors through the future.
            self.get_logger().error(f'Go2 navigation mode request failed: {exc}')
            return
        if not response.success:
            self.get_logger().error(
                f'Go2 rejected navigation mode {enabled}: {response.message}')
            return

        self.navigation_mode = enabled
        if callback is not None:
            callback()
        self.retry_mode_request()


def main():
    rclpy.init()
    node = JieGo2Navigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_navigation_pub.publish(Bool(data=True))
        if node.navigation_mode and node.mode_client.service_is_ready():
            request = SetBool.Request()
            request.data = False
            node.mode_client.call_async(request)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
