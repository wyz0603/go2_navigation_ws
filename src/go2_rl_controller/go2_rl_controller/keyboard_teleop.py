import curses
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.duration import Duration


COMMAND_STEPS = {
    "w": (0, 0.15),
    "s": (0, -0.15),
    "a": (1, 0.1),
    "d": (1, -0.1),
    "q": (2, 0.15),
    "e": (2, -0.15),
}


def update_command(
    command: tuple[float, float, float], key: int | str
) -> tuple[float, float, float]:
    if isinstance(key, int):
        if key < 0:
            return command
        try:
            key = chr(key)
        except ValueError:
            return command
    if key == " ":
        return 0.0, 0.0, 0.0

    step = COMMAND_STEPS.get(key.lower())
    if step is None:
        return command

    axis, delta = step
    updated = list(command)
    updated[axis] += delta
    return tuple(updated)


def main() -> None:
    rclpy.init()
    node = Node("keyboard_teleop")
    topic = node.declare_parameter("topic", "/cmd_vel_teleop").value
    publisher = node.create_publisher(Twist, topic, 10)

    if not sys.stdin.isatty() or not sys.stdout.isatty():
        node.get_logger().error("Keyboard teleop requires an interactive terminal.")
        node.destroy_node()
        rclpy.shutdown()
        return

    command = (0.0, 0.0, 0.0)

    def run(screen) -> None:
        nonlocal command
        screen.timeout(50)
        try:
            curses.curs_set(0)
        except curses.error:
            pass

        while rclpy.ok():
            screen.erase()
            screen.addstr(0, 0, "Go2 keyboard control (focus this terminal)")
            screen.addstr(2, 0, "W/S: forward/back   A/D: left/right   Q/E: yaw")
            screen.addstr(3, 0, "Keys change speed in steps; Space: stop   Esc: quit")
            screen.addstr(
                5,
                0,
                f"vx={command[0]:+.2f} m/s  vy={command[1]:+.2f} m/s  "
                f"yaw={command[2]:+.2f} rad/s",
            )
            screen.refresh()
            key = screen.getch()
            if key == 27:
                break
            command = update_command(command, key)

            message = Twist()
            message.linear.x, message.linear.y, message.angular.z = command
            publisher.publish(message)

    try:
        curses.wrapper(run)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                publisher.publish(Twist())
                publisher.wait_for_all_acked(Duration(seconds=0.5))
            except RCLError:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
