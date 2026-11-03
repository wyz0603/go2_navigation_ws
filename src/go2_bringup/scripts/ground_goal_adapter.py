#!/usr/bin/env python3
"""Convert a world-frame surface goal to the planner's body-height goal."""
import math
import copy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool


def body_goal(message, clearance, frame):
    if message.header.frame_id != frame:
        raise ValueError(f'Goal frame must be {frame}')
    p, q = message.pose.position, message.pose.orientation
    if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w, clearance)):
        raise ValueError('Goal contains non-finite values')
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if norm < 1e-9:
        raise ValueError('Goal quaternion must be nonzero')
    result = copy.deepcopy(message)
    result.pose.position.z += clearance
    q = result.pose.orientation
    q.x, q.y, q.z, q.w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    return result


class GroundGoalAdapter(Node):
    def __init__(self):
        super().__init__('ground_goal_adapter')
        self.clearance = self.declare_parameter('ground_clearance', 0.35).value
        self.frame = self.declare_parameter('world_frame', 'world').value
        self.select_navigation_mode = self.create_client(SetBool, '/go2/set_navigation')
        self.publisher = self.create_publisher(PoseStamped, '/ugv_nav4d_ros2/goal_pose', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.receive, 10)

    def receive(self, message):
        try:
            goal = body_goal(message, self.clearance, self.frame)
            if self.select_navigation_mode.service_is_ready():
                request = SetBool.Request()
                request.data = True
                future = self.select_navigation_mode.call_async(request)
                future.add_done_callback(lambda result: self.publish_after_mode(goal, result))
            else:
                self.get_logger().error('Navigation mode service is unavailable; goal was not forwarded')
        except ValueError as error:
            self.get_logger().error(str(error))

    def publish_after_mode(self, goal, future):
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f'Failed to select navigation mode: {error}')
            return
        if not response.success:
            self.get_logger().error(f'Failed to select navigation mode: {response.message}')
            return
        self.publisher.publish(goal)


def main():
    rclpy.init()
    node = GroundGoalAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
