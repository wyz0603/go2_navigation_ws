#!/usr/bin/env python3
"""Drive simulation goals and verify arrival using odometry, including floor height.

Start accessible.launch.py first and source install/setup.bash in this terminal.
The default sequence drives to the ground floor, second floor, then third floor.
"""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from ugv_nav4d_ros2.msg import MissionStatus


class NavigationVerification(Node):
    def __init__(self, args):
        super().__init__('navigation_verification')
        self.args = args
        self.odom = self.command = self.planner = self.execution = None
        self.started = None
        self.target = None
        self.seen_execution = False
        self.results = []
        args.output.mkdir(parents=True, exist_ok=True)
        self.trace = (args.output / 'odometry.jsonl').open('w')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.stop_client = self.create_client(Trigger, '/ugv_nav4d_ros2/stop_execution')
        self.create_subscription(Odometry, '/odom', self.receive_odometry, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.receive_command, 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(MissionStatus, '/ugv_nav4d_ros2/planner_status', self.receive_planner, qos)
        self.create_subscription(MissionStatus, '/ugv_nav4d_ros2/execution_status', self.receive_execution, qos)

    def receive_odometry(self, message):
        self.odom = message

    def receive_command(self, message):
        self.command = message

    def receive_planner(self, message):
        self.planner = message

    def receive_execution(self, message):
        changed = self.execution is None or (message.state, message.current_segment) != (
            self.execution.state, self.execution.current_segment)
        self.execution = message
        if self.started is not None and message.state == MissionStatus.EXECUTING:
            self.seen_execution = True
        if changed:
            print(f'{message.state_name}: {message.summary} {message.failure_reason}', flush=True)

    def snapshot(self):
        position = self.odom.pose.pose.position
        return {
            'elapsed': round(time.monotonic() - self.started, 2),
            'sim_time': self.odom.header.stamp.sec + self.odom.header.stamp.nanosec * 1e-9,
            'position': [position.x, position.y, position.z],
            'target_surface': list(self.target),
            'state': self.execution.state_name if self.execution else '',
            'segment': self.execution.current_segment if self.execution else 0,
            'command': ([self.command.linear.x, self.command.linear.y, self.command.angular.z]
                        if self.command else None),
        }

    def wait_ready(self):
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.1)
            # The planner retains EXECUTING after handing a route to the
            # follower. Its completed task must not block the next verification.
            idle = self.execution and self.execution.state in (
                MissionStatus.READY, MissionStatus.COMPLETED, MissionStatus.ABORTED)
            ready = self.planner and (
                self.planner.state == MissionStatus.READY or
                (self.planner.state == MissionStatus.EXECUTING and self.planner.route_valid))
            if (self.odom and ready and idle and self.publisher.get_subscription_count()):
                return
        raise RuntimeError('Planner, idle follower, odometry and goal adapter did not become ready within 90 seconds')

    def run_goal(self, target):
        self.target = target
        self.started = time.monotonic()
        self.seen_execution = False
        message = PoseStamped()
        message.header.frame_id = 'world'
        message.pose.position.x, message.pose.position.y, message.pose.position.z = target
        message.pose.orientation.w = 1.
        self.publisher.publish(message)
        print(f'Goal surface: {target}', flush=True)
        next_sample = 0.
        while time.monotonic() - self.started < self.args.timeout:
            rclpy.spin_once(self, timeout_sec=.1)
            now = time.monotonic()
            if now >= next_sample:
                sample = self.snapshot()
                print(json.dumps(sample), flush=True)
                self.trace.write(json.dumps(sample) + '\n')
                self.trace.flush()
                next_sample = now + 10
            if not self.seen_execution:
                if now - self.started > 45:
                    raise RuntimeError('Goal did not start: ' + self.planner.summary)
                continue
            if self.execution.state in (MissionStatus.PAUSED, MissionStatus.FAILED, MissionStatus.ABORTED):
                raise RuntimeError(self.execution.summary + ': ' + self.execution.failure_reason)
            if self.execution.state != MissionStatus.COMPLETED:
                continue
            position = self.odom.pose.pose.position
            xy_error = math.hypot(position.x - target[0], position.y - target[1])
            height_error = abs(position.z - target[2] - self.args.ground_clearance)
            result = dict(self.snapshot(), xy_error=xy_error, height_error=height_error)
            self.results.append(result)
            if xy_error > .25 or height_error > .4:
                raise RuntimeError(f'Controller reported completion at the wrong physical position: {result}')
            print('PASS ' + json.dumps(result), flush=True)
            return
        raise RuntimeError(f'Goal timed out: {self.snapshot()}')

    def stop_after_failure(self):
        if self.started is None or not self.stop_client.wait_for_service(timeout_sec=1):
            return
        future = self.stop_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 3
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal', type=float, nargs=3, action='append', metavar=('X', 'Y', 'GROUND_Z'))
    parser.add_argument('--timeout', type=float, default=1200, help='Wall seconds allowed per goal')
    parser.add_argument('--ground-clearance', type=float, default=.35)
    parser.add_argument('--output', type=Path, default=Path('/tmp/go2_navigation_verification'))
    args = parser.parse_args()
    goals = args.goal or [(1., 0., 0.), (.839, 1.423, 3.), (.839, 1.423, 6.)]
    if (not all(math.isfinite(v) for goal in goals for v in goal)
            or not math.isfinite(args.timeout) or args.timeout <= 0
            or not math.isfinite(args.ground_clearance) or args.ground_clearance < 0):
        parser.error('Require finite goals, positive timeout, and nonnegative ground clearance')
    rclpy.init()
    node = NavigationVerification(args)
    error = None
    try:
        node.wait_ready()
        for goal in goals:
            node.run_goal(goal)
    except (Exception, KeyboardInterrupt) as exception:
        error = str(exception) or 'Interrupted'
        print('FAIL: ' + error, flush=True)
        node.stop_after_failure()
    finally:
        (args.output / 'report.json').write_text(json.dumps(
            {'success': error is None, 'failure': error, 'results': node.results}, indent=2) + '\n')
        node.trace.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if error else 0


if __name__ == '__main__':
    raise SystemExit(main())
