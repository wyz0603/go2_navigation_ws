"""Explicit command-source selection with a wall-clock watchdog."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_srvs.srv import SetBool


class CommandGate:
    def __init__(self, timeout=0.3):
        self.timeout = timeout
        self.navigation = False
        self.command = (0.0, 0.0, 0.0)
        self.received = float('-inf')

    def select(self, navigation):
        self.navigation = navigation
        self.command = (0.0, 0.0, 0.0)
        self.received = float('-inf')

    def accept(self, navigation, values, now):
        if navigation != self.navigation:
            return
        if not all(math.isfinite(v) for v in values):
            self.command = (0.0, 0.0, 0.0)
            self.received = float('-inf')
            return
        limits = (0.5, 0.3, 0.8)
        self.command = tuple(max(-limit, min(limit, value))
                             for value, limit in zip(values, limits))
        self.received = now

    def output(self, now):
        return self.command if 0 <= now - self.received < self.timeout else (0.0, 0.0, 0.0)


class CommandMux(Node):
    def __init__(self):
        super().__init__('command_mux')
        self.gate = CommandGate()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel_teleop', lambda m: self.receive(False, m), 10)
        self.create_subscription(Twist, '/cmd_vel_nav', lambda m: self.receive(True, m), 10)
        self.create_service(SetBool, '/go2/set_navigation', self.select)
        # Watchdog must keep running even if simulation time stops.
        self.create_timer(0.05, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, navigation, message):
        self.gate.accept(navigation, (message.linear.x, message.linear.y, message.angular.z),
                         time.monotonic())

    def select(self, request, response):
        self.gate.select(request.data)
        self.publish()
        response.success = True
        response.message = 'navigation' if request.data else 'teleop'
        return response

    def publish(self):
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = self.gate.output(time.monotonic())
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = CommandMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.gate.select(False)
            node.publish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
