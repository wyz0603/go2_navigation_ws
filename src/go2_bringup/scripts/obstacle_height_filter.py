#!/usr/bin/env python3
"""Filter obstacle points by world-vertical height relative to the robot base."""
import math
import time
import numpy as np
from scipy import ndimage
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener, TransformException


MAX_CLOUD_AGE = 0.5
MAX_TRANSFORM_WAIT = 0.5


def height_mask(points, rotation, translation, base_z, minimum, maximum):
    """World vertical remains vertical when the robot / lidar pitches or rolls."""
    finite = np.isfinite(points).all(axis=1)
    relative_z = points[finite] @ rotation[2, :] + translation[2] - base_z
    keep = np.zeros(len(points), dtype=bool)
    keep[finite] = (relative_z >= minimum) & (relative_z <= maximum)
    return keep


def transform_arrays(transform):
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    t = transform.translation
    return rotation, np.array([t.x, t.y, t.z])


def cloud_is_expired(age, waited,
                     max_age=MAX_CLOUD_AGE,
                     max_wait=MAX_TRANSFORM_WAIT):
    """Reject stale clouds, but briefly retain a cloud stamped in the future."""
    if not all(math.isfinite(value) for value in (age, waited, max_age, max_wait)):
        return True
    return age > max_age or waited > max_wait


class TerrainSurfaceGrid:
    """Remove mapped support surfaces from the local obstacle cloud."""

    def __init__(self, resolution, points, availability_radius=1.0):
        self.resolution = float(resolution)
        self.points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        self.availability_radius = float(availability_radius)
        finite = np.isfinite(self.points).all(axis=1)
        self.points = self.points[finite]
        self.origin = np.zeros(2, dtype=np.float64)
        self.width = 0
        self.height = 0
        self.cells = {}
        self.indices = np.empty((0, 2), dtype=np.int64)
        if (not len(self.points) or not np.isfinite(resolution)
                or resolution <= 0.0 or not np.isfinite(availability_radius)
                or availability_radius < 0.0):
            return
        self.origin = self.points[:, :2].min(axis=0)
        indices = np.floor(
            (self.points[:, :2] - self.origin) / self.resolution
        ).astype(np.int64)
        self.indices = indices
        self.width, self.height = indices.max(axis=0) + 1
        for (x, y), point in zip(indices, self.points):
            key = (int(x), int(y))
            self.cells.setdefault(key, []).append(point[2])

    def ground_mask(self, points, base_z, clearance, max_step, tolerance):
        ground = np.zeros(len(points), dtype=bool)
        if not self.cells or not len(points):
            return ground
        indices = np.floor(
            (points[:, :2] - self.origin) / self.resolution
        ).astype(np.int64)
        radius = int(math.ceil(self.availability_radius / self.resolution))
        target_z = base_z - clearance
        shape = (int(self.width + 2 * radius), int(self.height + 2 * radius))
        # Unusually sparse maps can span too much memory; preserve the original
        # sparse lookup in that case rather than allocating an enormous grid.
        if shape[0] * shape[1] <= 4_000_000:
            below = np.full(shape, -np.inf)
            above = np.full(shape, np.inf)
            terrain_xy = self.indices + radius
            heights = self.points[:, 2]
            lower = heights <= target_z
            np.maximum.at(below, (terrain_xy[lower, 0], terrain_xy[lower, 1]), heights[lower])
            np.minimum.at(above, (terrain_xy[~lower, 0], terrain_xy[~lower, 1]), heights[~lower])
            window = 2 * radius + 1
            below = ndimage.maximum_filter(below, size=window, mode='constant', cval=-np.inf)
            above = ndimage.minimum_filter(above, size=window, mode='constant', cval=np.inf)
            query = indices + radius
            valid = ((query[:, 0] >= 0) & (query[:, 0] < shape[0]) &
                     (query[:, 1] >= 0) & (query[:, 1] < shape[1]))
            slots = np.flatnonzero(valid)
            if len(slots):
                x, y = query[valid, 0], query[valid, 1]
                low, high = below[x, y], above[x, y]
                height = np.where(target_z - low <= high - target_z, low, high)
                ground[slots] = (np.isfinite(height) & (np.abs(height - target_z) <= max_step) &
                                 (np.abs(points[valid, 2] - height) <= tolerance))
            return ground
        for slot, (point, (x, y)) in enumerate(zip(points, indices)):
            nearby = []
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    nearby.extend(self.cells.get((int(x + dx), int(y + dy)), ()))
            if not nearby:
                continue
            height = min(nearby, key=lambda z: abs(z - target_z))
            if (abs(height - target_z) <= max_step
                    and abs(point[2] - height) <= tolerance):
                ground[slot] = True
        return ground


class ObstacleHeightFilter(Node):
    def __init__(self):
        super().__init__('obstacle_height_filter')
        self.world = self.declare_parameter('world_frame', 'world').value
        self.base = self.declare_parameter('base_frame', 'base').value
        self.minimum = float(self.declare_parameter('min_relative_height', -0.25).value)
        self.maximum = float(self.declare_parameter('max_relative_height', 0.45).value)
        self.clearance = float(self.declare_parameter('ground_clearance', 0.35).value)
        self.max_step = float(self.declare_parameter('max_step_height', 0.45).value)
        self.terrain_tolerance = float(self.declare_parameter('terrain_tolerance', 0.08).value)
        self.terrain_radius = float(self.declare_parameter('terrain_radius', 0.15).value)
        if not np.isfinite([self.minimum, self.maximum]).all() or self.minimum >= self.maximum:
            raise ValueError('Require finite min_relative_height < max_relative_height')
        if (not np.isfinite([self.clearance, self.max_step, self.terrain_tolerance]).all()
                or not np.isfinite(self.terrain_radius)
                or self.clearance < 0 or self.max_step <= 0
                or self.terrain_tolerance <= 0 or self.terrain_radius < 0):
            raise ValueError('Require finite nonnegative clearance/radius and positive step/tolerance')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(PointCloud2, '/go2/obstacle_points', qos_profile_sensor_data)
        self.terrain = None
        self.create_subscription(
            Marker, '/jie/traversable_cells_markers', self.receive_terrain,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pending = None
        self.newest_pending = None
        self.pending_since = 0.0
        self.create_subscription(PointCloud2, '/go2/points', self.receive, qos_profile_sensor_data)
        # Retry the active cloud until its exact TF arrives. While it waits, retain
        # one newer cloud rather than replacing the active frame every callback.
        self.create_timer(0.02, self.process)

    def receive(self, message):
        if self.pending is None:
            self.pending = message
            self.pending_since = time.monotonic()
        else:
            self.newest_pending = message

    def release_pending(self):
        self.pending = self.newest_pending
        self.newest_pending = None
        self.pending_since = time.monotonic() if self.pending is not None else 0.0

    def receive_terrain(self, message):
        if message.header.frame_id != self.world:
            self.get_logger().warning('Ignoring traversability map in a different frame')
            return
        if message.action in (Marker.DELETE, Marker.DELETEALL):
            self.terrain = None
            return
        if message.type != Marker.CUBE_LIST:
            return
        points = np.array([(p.x, p.y, p.z - message.scale.z / 2)
                           for p in message.points], dtype=float).reshape(-1, 3)
        if not len(points):
            self.get_logger().warning('Ignoring empty traversability map')
            return
        self.terrain = TerrainSurfaceGrid(
            max(message.scale.x, 0.01), points, self.terrain_radius
        )

    def process(self):
        if self.pending is None:
            self.release_pending()
        message = self.pending
        if message is None:
            return
        stamp = rclpy.time.Time.from_msg(message.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        waited = time.monotonic() - self.pending_since
        if cloud_is_expired(age, waited):
            self.get_logger().warning(
                f'Dropping cloud after {waited:.2f} s without usable TF '
                f'(stamp age {age:.2f} s)',
                throttle_duration_sec=5.0,
            )
            self.release_pending()
            return
        try:
            cloud_tf = self.buffer.lookup_transform(self.world, message.header.frame_id, stamp).transform
            base_tf = self.buffer.lookup_transform(self.world, self.base, stamp).transform
        except TransformException as error:
            self.get_logger().warning(f'Waiting for cloud timestamp TF: {error}', throttle_duration_sec=5.0)
            return
        self.release_pending()
        points = point_cloud2.read_points_numpy(message, field_names=('x', 'y', 'z'), skip_nans=False).reshape(-1, 3)
        rotation, translation = transform_arrays(cloud_tf)
        keep = height_mask(points, rotation, translation, base_tf.translation.z, self.minimum, self.maximum)
        if self.terrain is not None:
            slots = np.flatnonzero(keep)
            world_points = points[keep] @ rotation.T + translation
            keep[slots] &= ~self.terrain.ground_mask(
                world_points, base_tf.translation.z, self.clearance,
                self.max_step, self.terrain_tolerance)
        # Preserve measurement time and sensor frame for SCAN transformation.
        self.publisher.publish(point_cloud2.create_cloud_xyz32(message.header, points[keep]))


def main():
    rclpy.init()
    node = ObstacleHeightFilter()
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
