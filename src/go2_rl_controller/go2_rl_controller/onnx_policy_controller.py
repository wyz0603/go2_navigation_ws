"""Run the exported 450-input Go2 velocity policy in Gazebo Sim.

The node deliberately keeps the observation construction explicit.  The layout
matches the Go2 RL deployment contract: angular velocity, projected gravity,
body-frame command, joint position error, joint velocity, and previous action.
"""

from collections import deque
import csv
from datetime import datetime
from enum import IntEnum
import math
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from go2_rl_interfaces.msg import PolicyStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray


JOINT_NAMES = (
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
)

FOOT_ORDER = ("FL", "FR", "RL", "RR")


def clip_torque_speed(
    effort,
    joint_velocity,
    use_torque_speed_curve,
    speed_x1,
    speed_x2,
    effort_y1,
    effort_y2,
):
    """Apply the Go2HV direction-dependent torque-speed envelope.

    This pure NumPy helper mirrors ``go2_actuator.py::_clip_effort`` and the
    MuJoCo deployment helper, making the actuator boundary independently
    testable without constructing a ROS node.
    """
    effort = np.asarray(effort, dtype=np.float64)
    joint_velocity = np.asarray(joint_velocity, dtype=np.float64)
    if not use_torque_speed_curve:
        return effort
    if speed_x2 <= speed_x1:
        raise ValueError("motor_x2 must be greater than motor_x1")
    same_direction = joint_velocity * effort > 0.0
    maximum_effort = np.where(same_direction, effort_y1, effort_y2)
    high_speed_effort = maximum_effort * (
        1.0 - (np.abs(joint_velocity) - speed_x1) / (speed_x2 - speed_x1)
    )
    maximum_effort = np.where(
        np.abs(joint_velocity) < speed_x1,
        maximum_effort,
        np.maximum(high_speed_effort, 0.0),
    )
    return np.clip(effort, -maximum_effort, maximum_effort)


class State(IntEnum):
    WAITING_FOR_SENSORS = 0
    STAND_TRANSITION = 1
    POLICY_WARMUP = 2
    ACTIVE = 3
    FAULT = 4


class Go2OnnxPolicyController(Node):
    def __init__(self):
        super().__init__("onnx_policy_controller")
        self.policy_path = self.declare_parameter("policy_path", "").value
        self.control_rate = float(self.declare_parameter("control_rate", 200.0).value)
        self.policy_decimation = int(self.declare_parameter("policy_decimation", 4).value)
        self.history_length = int(self.declare_parameter("history_length", 10).value)
        self.single_obs_size = int(self.declare_parameter("single_observation_size", 45).value)
        self.action_size = int(self.declare_parameter("action_size", 12).value)
        self.action_scale = float(self.declare_parameter("action_scale", 0.25).value)
        self.action_clip = float(self.declare_parameter("action_clip", 100.0).value)
        self.observation_clip = float(self.declare_parameter("observation_clip", 100.0).value)
        self.ang_vel_scale = float(self.declare_parameter("ang_vel_scale", 0.25).value)
        self.dof_pos_scale = float(self.declare_parameter("dof_pos_scale", 1.0).value)
        self.dof_vel_scale = float(self.declare_parameter("dof_vel_scale", 0.05).value)
        self.command_scale = np.asarray(
            self.declare_parameter("command_scale", [0.5, 1.0, 1.0]).value, dtype=np.float32
        )
        self.default_angles = np.asarray(self.declare_parameter(
            "default_angles",
            [0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1.0, -1.5, -0.1, 1.0, -1.5],
        ).value, dtype=np.float64)
        self.kp = float(self.declare_parameter("kp", 25.0).value)
        self.kd = float(self.declare_parameter("kd", 0.0).value)
        self.motor_use_torque_speed_curve = bool(
            self.declare_parameter("motor_use_torque_speed_curve", True).value
        )
        self.motor_x1 = float(self.declare_parameter("motor_x1", 13.5).value)
        self.motor_x2 = float(self.declare_parameter("motor_x2", 30.0).value)
        self.motor_y1 = float(self.declare_parameter("motor_y1", 20.2).value)
        self.motor_y2 = float(self.declare_parameter("motor_y2", 23.4).value)
        if self.motor_x2 <= self.motor_x1:
            raise ValueError("motor_x2 must be greater than motor_x1")
        self.command_timeout = float(self.declare_parameter("command_timeout", 0.3).value)
        self.sensor_timeout = float(self.declare_parameter("sensor_timeout", 0.2).value)
        self.transition_duration = float(
            self.declare_parameter("stand_transition_duration", 2.0).value
        )
        self.warmup_duration = float(self.declare_parameter("policy_warmup_duration", 1.0).value)
        self.hold_stand = bool(self.declare_parameter("hold_stand_without_command", False).value)
        self.front_thigh_action_gain = float(
            self.declare_parameter("front_thigh_action_gain", 1.0).value
        )
        self.front_calf_offset = float(self.declare_parameter("front_calf_offset", 0.0).value)
        self.heading_hold = bool(self.declare_parameter("heading_hold", False).value)
        self.heading_kp = float(self.declare_parameter("heading_kp", 1.0).value)
        self.heading_max_yaw_rate = float(
            self.declare_parameter("heading_max_yaw_rate", 0.25).value
        )
        self.joint_topic = self.declare_parameter("joint_state_topic", "/joint_states").value
        self.imu_topic = self.declare_parameter("imu_topic", "/go2/imu").value
        self.cmd_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.effort_topic = self.declare_parameter(
            "effort_topic", "/go2_effort_controller/commands"
        ).value

        # --- CSV logging / instrumentation parameters ---
        self.log_enabled = bool(self.declare_parameter("log_enabled", False).value)
        self.log_csv_path = self.declare_parameter("log_csv_path", "").value
        self.log_every_control_tick = bool(self.declare_parameter("log_every_control_tick", False).value)
        self.log_flush_every = max(1, int(self.declare_parameter("log_flush_every", 50).value))
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.foot_contact_topic = self.declare_parameter(
            "foot_contact_topic", "/go2/foot_contacts"
        ).value

        if (self.single_obs_size != 45 or self.action_size != 12
                or self.history_length != 10):
            raise ValueError("This policy requires 45 observations, 12 actions and 10 history frames")
        if (self.control_rate <= 0 or self.policy_decimation < 1
                or not math.isclose(self.policy_decimation / self.control_rate, 0.02)):
            raise ValueError("Policy period must match training: decimation / control_rate = 0.02 s")
        for name, values, size in (("default_angles", self.default_angles, 12),
                                   ("command_scale", self.command_scale, 3)):
            if values.shape != (size,) or not np.all(np.isfinite(values)):
                raise ValueError(f"{name} must contain {size} finite values")
        if self.transition_duration <= 0 or self.sensor_timeout <= 0 or self.command_timeout <= 0:
            raise ValueError("Transition and timeout durations must be positive")

        self.state = State.WAITING_FOR_SENSORS
        self.state_started = self.now_seconds()
        self.have_joints = False
        self.have_imu = False
        self.last_joint_time = 0.0
        self.last_imu_time = 0.0
        self.last_command_time = 0.0
        self.command = np.zeros(3, dtype=np.float64)
        self.yaw = 0.0
        self.heading_target = None
        self.joint_positions = np.zeros(12, dtype=np.float64)
        self.joint_velocities = np.zeros(12, dtype=np.float64)
        self.angular_velocity = np.zeros(3, dtype=np.float64)
        self.projected_gravity = np.array([0.0, 0.0, -1.0], dtype=np.float64)
        self.transition_start = self.default_angles.copy()
        self.target_positions = self.default_angles.copy()
        self.previous_action = np.zeros(12, dtype=np.float64)
        self.last_policy_action = np.zeros(12, dtype=np.float64)
        self.smoothed_command = np.zeros(3, dtype=np.float64)
        self.policy_tick = 0
        self.history = deque(maxlen=self.history_length)
        self.policy = None
        self.policy_input_name = None
        self.policy_loaded = False
        self.fault_message = ""

        # Telemetry for logging
        self.body_height = float("nan")
        self.odom_linear = np.full(3, float("nan"), dtype=np.float64)
        self.odom_angular = np.full(3, float("nan"), dtype=np.float64)
        self.odom_orientation_rpy = np.full(3, float("nan"), dtype=np.float64)
        self.odom_child_frame = ""
        self.odom_parent_frame = ""
        self.last_odom_receive_time = float("nan")
        self.last_odom_stamp = float("nan")
        self.last_csv_sim_time = float("nan")
        self.csv_rows_written = 0
        self.foot_contacts = np.full(4, float("nan"), dtype=np.float64)

        # CSV handle
        self.csv_file = None
        self.csv_writer = None

        self.effort_pub = self.create_publisher(Float64MultiArray, self.effort_topic, 10)
        self.status_pub = self.create_publisher(PolicyStatus, "/go2/rl/status", 10)
        # Keep strong references: rclpy subscriptions can be garbage-collected
        # when the returned handles are not retained by the node.
        self.joint_subscription = self.create_subscription(
            JointState, self.joint_topic, self.on_joint_state,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.imu_subscription = self.create_subscription(
            Imu, self.imu_topic, self.on_imu,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.command_subscription = self.create_subscription(
            Twist, self.cmd_topic, self.on_command, 10
        )
        self.odom_subscription = self.create_subscription(
            Odometry, self.odom_topic, self.on_odom, 10
        )
        self.foot_contact_subscription = self.create_subscription(
            Float64MultiArray, self.foot_contact_topic, self.on_foot_contact, 10
        )

        self.last_status_time = float("-inf")
        self.open_csv_log()
        self.load_policy()
        self.timer = self.create_timer(1.0 / self.control_rate, self.update)
        self.get_logger().info(
            f"Go2 RL controller configured: {self.policy_path or 'no policy path'}, "
            f"input={self.single_obs_size}, history={self.history_length}, actions={self.action_size}",
        )

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    # ------------------------------------------------------------------ #
    # CSV logging helpers
    # ------------------------------------------------------------------ #
    def open_csv_log(self):
        if not self.log_enabled:
            self.get_logger().info("CSV logging disabled via log_enabled=false")
            return
        try:
            if self.log_csv_path:
                path = Path(self.log_csv_path)
            else:
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                path = Path.cwd() / f"policy_log_{stamp}.csv"
            path.parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = path.open("w", newline="")
            self.csv_writer = csv.writer(self.csv_file)

            header = [
                "sim_time", "control_dt_observed", "controller_state", "policy_inference",
                "cmd_vx", "cmd_vy", "cmd_wz", "body_height",
                "base_vx", "base_vy", "base_vz",
                "base_wx", "base_wy", "base_wz",
                "base_roll", "base_pitch", "base_yaw",
                "odom_child_frame", "odom_parent_frame", "odom_age_s", "odom_header_stamp_s",
                "joint_age_s", "imu_age_s", "command_age_s", "policy_tick",
            ]
            header += [f"action_{i}" for i in range(self.action_size)]
            header += [f"q_target_{i}" for i in range(12)]
            header += [f"q_{i}" for i in range(12)]
            header += [f"dq_{i}" for i in range(12)]
            header += [f"tau_{i}" for i in range(12)]
            header += [f"contact_{name}" for name in FOOT_ORDER]
            self.csv_writer.writerow(header)
            self.csv_file.flush()
            self.get_logger().info(f"Logging policy data to {path}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"failed to open CSV log: {exc}")
            self.csv_file = None
            self.csv_writer = None

    def close_csv_log(self):
        if self.csv_file is not None:
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except Exception:  # noqa: BLE001
                pass
            self.csv_file = None
            self.csv_writer = None

    def compute_torque(self, target):
        """Compute commanded effort and apply the Go2HV envelope.

        The Isaac Lab training actuator selects Y1/Y2 from the relative
        direction of joint speed and effort, then linearly reduces the limit
        from X1 to X2.  Keep this calculation on the final effort, immediately
        before publishing to ros2_control. Gazebo's joint damping is outside
        this limit, so this remains an approximation of the training actuator.
        """
        effort = self.kp * (target - self.joint_positions) - self.kd * self.joint_velocities
        if not self.motor_use_torque_speed_curve:
            return effort

        return clip_torque_speed(
            effort,
            self.joint_velocities,
            self.motor_use_torque_speed_curve,
            self.motor_x1,
            self.motor_x2,
            self.motor_y1,
            self.motor_y2,
        )

    def log_control_step(self, command, policy_inference=False):
        """Log 200 Hz controller ticks (or policy ticks if configured).

        base_v* is odometry.twist.twist.linear expressed in child_frame_id,
        which must be a robot body frame before comparing with /cmd_vel.
        """
        if self.csv_writer is None:
            return
        if not self.log_every_control_tick and not policy_inference:
            return
        now = self.now_seconds()
        dt = now - self.last_csv_sim_time if math.isfinite(self.last_csv_sim_time) else float("nan")
        self.last_csv_sim_time = now
        def age(last):
            return now - last if math.isfinite(last) and last > 0 else float("nan")

        torque = self.compute_torque(self.target_positions)
        row = [
            now, dt, self.state.name, int(policy_inference),
            float(command[0]), float(command[1]), float(command[2]),
            float(self.body_height),
            *self.odom_linear.tolist(),
            *self.odom_angular.tolist(),
            *self.odom_orientation_rpy.tolist(),
            self.odom_child_frame, self.odom_parent_frame,
            age(self.last_odom_receive_time), self.last_odom_stamp,
            age(self.last_joint_time), age(self.last_imu_time),
            age(self.last_command_time), self.policy_tick,
        ]
        row += [float(x) for x in self.last_policy_action]
        row += [float(x) for x in self.target_positions]
        row += [float(x) for x in self.joint_positions]
        row += [float(x) for x in self.joint_velocities]
        row += [float(x) for x in torque]
        row += [float(x) for x in self.foot_contacts]
        try:
            self.csv_writer.writerow(row)
            self.csv_rows_written += 1
            if self.csv_rows_written % self.log_flush_every == 0:
                self.csv_file.flush()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"failed to write CSV row: {exc}")

    # ------------------------------------------------------------------ #
    # Telemetry callbacks
    # ------------------------------------------------------------------ #
    def on_odom(self, message):
        self.last_odom_receive_time = self.now_seconds()
        self.last_odom_stamp = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9
        self.odom_parent_frame = message.header.frame_id
        self.odom_child_frame = message.child_frame_id
        z = float(message.pose.pose.position.z)
        if math.isfinite(z):
            self.body_height = z

        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        lv = np.array([linear.x, linear.y, linear.z], dtype=np.float64)
        av = np.array([angular.x, angular.y, angular.z], dtype=np.float64)
        self.odom_linear[:] = lv if np.all(np.isfinite(lv)) else float("nan")
        self.odom_angular[:] = av if np.all(np.isfinite(av)) else float("nan")

        q = message.pose.pose.orientation
        x, y, zq, w = float(q.x), float(q.y), float(q.z), float(q.w)
        qnorm = math.sqrt(x*x + y*y + zq*zq + w*w)
        if math.isfinite(qnorm) and qnorm > 1e-9:
            x, y, zq, w = x/qnorm, y/qnorm, zq/qnorm, w/qnorm
            roll = math.atan2(2*(w*x + y*zq), 1 - 2*(x*x + y*y))
            sin_pitch = max(-1.0, min(1.0, 2*(w*y - zq*x)))
            pitch = math.asin(sin_pitch)
            yaw = math.atan2(2*(w*zq + x*y), 1 - 2*(y*y + zq*zq))
            self.odom_orientation_rpy[:] = [roll, pitch, yaw]

    def on_foot_contact(self, message):
        if len(message.data) < 4:
            return
        values = np.asarray(message.data[:4], dtype=np.float64)
        if np.all(np.isfinite(values)):
            self.foot_contacts = values

    def on_command(self, message):
        self.command[:] = [message.linear.x, message.linear.y, message.angular.z]
        if not np.all(np.isfinite(self.command)):
            self.command.fill(0.0)
        self.last_command_time = self.now_seconds()

    def on_joint_state(self, message):
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        if any(name not in positions or name not in velocities for name in JOINT_NAMES):
            return
        values = np.asarray([positions[name] for name in JOINT_NAMES], dtype=np.float64)
        speeds = np.asarray([velocities[name] for name in JOINT_NAMES], dtype=np.float64)
        if not np.all(np.isfinite(values)) or not np.all(np.isfinite(speeds)):
            return
        self.joint_positions = values
        self.joint_velocities = speeds
        self.last_joint_time = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.have_joints = True

    def on_imu(self, message):
        q = message.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if not math.isfinite(norm) or norm < 1.0e-9:
            return
        x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
        angular_velocity = np.asarray(
            [message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(angular_velocity)):
            return
        self.angular_velocity = angular_velocity
        self.yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        self.projected_gravity = np.asarray(
            [2.0 * (-z * x + w * y), -2.0 * (z * y + w * x), 1.0 - 2.0 * (w * w + z * z)],
            dtype=np.float64,
        )
        self.last_imu_time = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.have_imu = True

    def sensors_fresh(self, now):
        return (
            self.have_joints and self.have_imu
            and 0.0 <= now - self.last_joint_time <= self.sensor_timeout
            and 0.0 <= now - self.last_imu_time <= self.sensor_timeout
        )

    def active_command(self, now):
        if now - self.last_command_time > self.command_timeout:
            self.heading_target = None
            return np.zeros(3, dtype=np.float64)
        command = self.command.copy()
        # Hold the starting heading only while translating without an explicit
        # turn command. Keep the reference across commanded stops so resuming
        # after a landing does not adopt accumulated yaw drift as a new heading.
        # Stops still command zero yaw; explicit turns and timeouts release it.
        if not self.heading_hold or abs(command[2]) > 1.0e-4:
            self.heading_target = None
        elif np.linalg.norm(command[:2]) >= 0.02:
            if self.heading_target is None:
                self.heading_target = self.yaw
            error = math.atan2(
                math.sin(self.heading_target - self.yaw),
                math.cos(self.heading_target - self.yaw),
            )
            limit = self.heading_max_yaw_rate
            command[2] = np.clip(self.heading_kp * error, -limit, limit)
        return command

    # ------------------------------------------------------------------ #
    # Policy loading / inference
    # ------------------------------------------------------------------ #
    def load_policy(self):
        if not self.policy_path:
            self.set_fault("policy_path is empty")
            return
        if not Path(self.policy_path).is_file():
            self.set_fault(f"policy file does not exist: {self.policy_path}")
            return
        try:
            import onnxruntime as ort
            options = ort.SessionOptions()
            options.intra_op_num_threads = 1
            options.inter_op_num_threads = 1
            self.policy = ort.InferenceSession(
                str(self.policy_path), options, providers=["CPUExecutionProvider"]
            )
            inputs = self.policy.get_inputs()
            outputs = self.policy.get_outputs()
            if len(inputs) != 1 or len(outputs) < 1:
                raise RuntimeError("policy must have one input and at least one output")
            self.policy_input_name = inputs[0].name
            shape = inputs[0].shape
            expected = self.single_obs_size * self.history_length
            if shape[-1] not in (expected, "batch", None):
                raise RuntimeError(f"policy input shape {shape} does not end in {expected}")
            self.policy_loaded = True
            self.get_logger().info(
                f"Loaded ONNX policy input={inputs[0].name} shape={shape} output={outputs[0].shape}"
            )
        except ImportError:
            self.set_fault("onnxruntime is not installed; install the Python ONNX Runtime package")
        except Exception as exc:  # noqa: BLE001 - convert model errors to a safe controller fault
            self.set_fault(f"failed to load ONNX policy: {exc}")

    def set_fault(self, message):
        self.fault_message = str(message)
        self.state = State.FAULT
        self.get_logger().error(self.fault_message)

    def make_observation(self, command):
        obs = np.zeros(self.single_obs_size, dtype=np.float32)
        obs[0:3] = self.angular_velocity.astype(np.float32) * self.ang_vel_scale
        obs[3:6] = self.projected_gravity.astype(np.float32)
        obs[6:9] = command.astype(np.float32) * self.command_scale
        obs[9:21] = ((self.joint_positions - self.default_angles) * self.dof_pos_scale).astype(np.float32)
        obs[21:33] = (self.joint_velocities * self.dof_vel_scale).astype(np.float32)
        obs[33:45] = self.previous_action.astype(np.float32)
        return np.clip(obs, -self.observation_clip, self.observation_clip)

    def infer(self, observation, apply_action):
        if not self.policy_loaded:
            return False
        if not self.history:
            self.history.extend([observation.copy() for _ in range(self.history_length)])
        else:
            self.history.append(observation.copy())
        # The MoE-CTS ONNX exporter stores history term-major, not frame-major:
        # angular_velocity[10], gravity[10], command[10], dof_pos[10],
        # dof_vel[10], previous_action[10].
        history_array = np.stack(tuple(self.history), axis=0)
        feature_dimensions = (3, 3, 3, 12, 12, 12)
        chunks = []
        offset = 0
        for dimension in feature_dimensions:
            end = offset + dimension
            chunks.append(history_array[:, offset:end].reshape(-1))
            offset = end
        model_input = np.concatenate(chunks, axis=0).reshape(1, -1).astype(np.float32)
        try:
            output = self.policy.run(None, {self.policy_input_name: model_input})[0]
            action = np.asarray(output, dtype=np.float32).reshape(-1)
            if action.size != self.action_size or not np.all(np.isfinite(action)):
                raise RuntimeError(f"invalid policy output size={action.size}")
            if apply_action:
                clipped = np.clip(action, -self.action_clip, self.action_clip).astype(np.float64)
                self.last_policy_action = clipped
                self.previous_action = clipped
                self.target_positions = self.default_angles + self.action_scale * self.previous_action
                # Small deployment corrections for the Gazebo model; keep the
                # policy's own action history unchanged.
                self.target_positions[[1, 4]] += (
                    (self.front_thigh_action_gain - 1.0)
                    * self.action_scale * self.previous_action[[1, 4]]
                )
                self.target_positions[[2, 5]] += self.front_calf_offset
            return True
        except Exception as exc:  # noqa: BLE001 - fail safe on inference errors
            self.set_fault(f"policy inference failed: {exc}")
            return False

    # ------------------------------------------------------------------ #
    # Low-level actuation / status
    # ------------------------------------------------------------------ #
    def publish_effort(self, target):
        torque = self.compute_torque(target)
        message = Float64MultiArray()
        message.data = torque.astype(float).tolist()
        self.effort_pub.publish(message)

    def publish_status(self):
        now = self.now_seconds()
        if 0.0 <= now - self.last_status_time < 0.1:
            return
        self.last_status_time = now
        message = PolicyStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.state = int(self.state)
        message.policy_loaded = self.policy_loaded
        message.observation_size = self.single_obs_size * self.history_length
        message.action_size = self.action_size
        message.message = self.fault_message if self.state == State.FAULT else self.state.name
        self.status_pub.publish(message)

    # ------------------------------------------------------------------ #
    # Main update
    # ------------------------------------------------------------------ #
    def update(self):
        now = self.now_seconds()
        if self.state == State.FAULT:
            self.effort_pub.publish(Float64MultiArray(data=[0.0] * 12))
            self.publish_status()
            return
        if not self.have_joints:
            self.publish_status()
            return
        if self.state == State.WAITING_FOR_SENSORS:
            if not self.sensors_fresh(now):
                self.publish_status()
                return
            self.transition_start = self.joint_positions.copy()
            self.state_started = now
            self.state = State.STAND_TRANSITION
        if self.state == State.STAND_TRANSITION:
            blend = min(1.0, max(0.0, (now - self.state_started) / self.transition_duration))
            self.target_positions = self.transition_start + blend * (self.default_angles - self.transition_start)
            self.publish_effort(self.target_positions)
            if blend >= 1.0:
                self.history.clear()
                self.policy_tick = 0
                self.state_started = now
                self.state = State.POLICY_WARMUP
            self.publish_status()
            return
        if not self.sensors_fresh(now):
            # The default-pose fallback does not apply the policy action.  Do
            # not reuse stale observations or report an old action on recovery.
            self.history.clear()
            self.heading_target = None
            self.previous_action.fill(0.0)
            self.last_policy_action.fill(0.0)
            self.target_positions = self.default_angles.copy()
            self.publish_effort(self.target_positions)
            self.publish_status()
            return
        command = self.active_command(now)
        observation = self.make_observation(command)
        if self.state == State.POLICY_WARMUP:
            if self.policy_tick % self.policy_decimation == 0 and not self.infer(observation, False):
                return
            self.policy_tick += 1
            self.publish_effort(self.default_angles)
            if now - self.state_started >= self.warmup_duration:
                self.state = State.ACTIVE
            self.publish_status()
            return
        if self.state == State.ACTIVE:
            policy_updated = False
            if self.hold_stand and np.allclose(command, 0.0, atol=1.0e-4):
                self.target_positions = self.default_angles
                self.previous_action.fill(0.0)
                self.last_policy_action.fill(0.0)
                if self.policy_tick % self.policy_decimation == 0:
                    observation = self.make_observation(command)
                    if not self.infer(observation, False):
                        return
                    policy_updated = True
            elif self.policy_tick % self.policy_decimation == 0:
                if not self.infer(observation, True):
                    return
                policy_updated = True
            self.policy_tick += 1
            self.publish_effort(self.target_positions)
            self.log_control_step(command, policy_inference=policy_updated)
            self.publish_status()

    def destroy_node(self):
        self.close_csv_log()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Go2OnnxPolicyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
