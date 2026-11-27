#!/usr/bin/env python3
"""Exercise SCAN and D1 together in an isolated ROS_DOMAIN_ID."""
import math
import subprocess
import time
from pathlib import Path as FilePath

from ament_index_python.packages import get_package_prefix
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header
from tf2_ros import TransformBroadcaster


def main():
    rclpy.init()
    node = Node('scan_navigation_integration_test')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    path_pub = node.create_publisher(Path, '/jie/planned_path', latched)
    cloud_pub = node.create_publisher(PointCloud2, '/go2/obstacle_points', qos_profile_sensor_data)
    odom_pub = node.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
    start_pub = node.create_publisher(Bool, '/jie/start_navigation', 10)
    stop_pub = node.create_publisher(Bool, '/jie/stop_navigation', 10)
    broadcaster = TransformBroadcaster(node)
    paths, commands, states = [], [], []
    node.create_subscription(Path, '/scan/local_path', paths.append, 1)
    node.create_subscription(Twist, '/cmd_vel_nav', commands.append, 10)
    node.create_subscription(Bool, '/jie/controller_active', lambda m: states.append(m.data), latched)
    processes = []
    logs = []
    for package, executable, params in [
        ('scan_planner', 'scan_local_planner', {}),
        ('octo_planner', 'go2_path_follower', {
            'path_topic': '/jie/planned_path', 'local_path_topic': '/scan/local_path',
            'start_navigation_topic': '/jie/start_navigation',
            'stop_navigation_topic': '/jie/stop_navigation',
            'controller_active_topic': '/jie/controller_active',
            'cmd_vel_topic': '/cmd_vel_nav', 'map_frame': 'world', 'base_frame': 'base',
            'robot_center_offset_x': '0.0', 'enable_tracking_debug_view': 'false',
            'enable_lateral_motion': 'false', 'goal_position_tolerance': '0.12',
            'goal_yaw_tolerance': '0.2', 'max_linear_speed': '0.25',
        }),
    ]:
        binary = FilePath(get_package_prefix(package)) / 'lib' / package / executable
        args = [str(binary), '--ros-args']
        for key, value in params.items():
            args += ['-p', f'{key}:={value}']
        log = open(f'/tmp/{executable}-integration.log', 'w')
        logs.append(log)
        processes.append(subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT))

    def spin(seconds, obstacles=(), publish_cloud=True):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            for process in processes:
                assert process.poll() is None, f'node exited {process.args[0]}'
            stamp = node.get_clock().now().to_msg()
            transform = TransformStamped()
            transform.header = Header(stamp=stamp, frame_id='world')
            transform.child_frame_id = 'base'
            transform.transform.translation.z = 0.35
            transform.transform.rotation.w = 1.0
            broadcaster.sendTransform(transform)
            odom = Odometry(header=transform.header, child_frame_id='base')
            odom.pose.pose.position.z = 0.35
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            if publish_cloud:
                cloud_pub.publish(point_cloud2.create_cloud_xyz32(transform.header, obstacles))
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.02)

    def global_path(goal_x):
        path = Path()
        path.header = Header(stamp=node.get_clock().now().to_msg(), frame_id='world')
        for i in range(1 if goal_x == 0.0 else 26):
            point = PoseStamped(header=path.header)
            point.pose.position.x = goal_x * i / 25
            point.pose.position.z = 0.35
            point.pose.orientation.w = 1.0
            path.poses.append(point)
        path_pub.publish(path)
        spin(0.5)
        start_pub.publish(Bool(data=True))

    def stopped():
        assert commands, 'no controller output'
        assert all(abs(m.linear.x) + abs(m.linear.y) + abs(m.angular.z) < 1e-6
                   for m in commands[-5:]), 'controller moved without a valid local path'

    try:
        spin(2.0)
        global_path(4.0)
        spin(2.0)
        assert any(m.poses for m in paths), 'no clear SCAN trajectory'
        assert any(m.linear.x > 0.05 for m in commands), 'D1 did not follow SCAN'
        print('PASS clear route produces local paths and forward commands', flush=True)

        paths.clear()
        obstacles = [(1.5 + dx, y, z) for dx in (-0.15, 0.0, 0.15)
                     for y in (-0.15, 0.0, 0.15) for z in (0.2, 0.35, 0.5)]
        spin(4.0, obstacles)
        candidates = [m for m in paths[-5:] if m.poses]
        assert candidates, 'SCAN did not find a route around a finite obstacle'
        for path in candidates:
            for pose in path.poses:
                p = pose.pose.position
                assert min(math.hypot(p.x-x, p.y-y) for x,y,z in obstacles) >= 0.24
                assert abs(p.z - 0.35) < 0.15, 'trajectory left the ground level'
        assert max(abs(p.pose.position.y) for m in candidates for p in m.poses) > 0.25
        print('PASS obstacle causes a collision-free lateral detour', flush=True)

        spin(2.0, [(0.0, 0.0, 0.35)])
        stopped()
        print('PASS blocked footprint stops D1', flush=True)
        commands.clear()
        spin(2.0)
        assert any(m.linear.x > 0.05 for m in commands), 'did not recover after obstacle removal'
        print('PASS obstacle removal resumes tracking', flush=True)
        spin(2.0, publish_cloud=False)
        stopped()
        print('PASS stale sensor data stops D1', flush=True)
        spin(1.0)
        stop_pub.publish(Bool(data=True))
        spin(0.5)
        global_path(0.0)
        spin(1.5)
        assert states[-1] is False, 'final goal did not finish navigation'
        stopped()
        print('PASS exact final goal completes without a degenerate solve', flush=True)
        global_path(4.0)
        spin(1.0)
        assert states[-1] is True, 'replacement goal was not activated'
        processes[0].terminate()
        processes[0].wait(timeout=5)
        processes.pop(0)
        spin(1.5)
        stopped()
        print('PASS stopped SCAN process triggers local-path timeout', flush=True)
        stop_pub.publish(Bool(data=True))
        spin(0.5)
        assert states[-1] is False
        stopped()
        print('PASS navigation cancellation clears controller state', flush=True)
    finally:
        for process in processes:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
