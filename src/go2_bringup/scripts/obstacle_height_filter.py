#!/usr/bin/env python3
"""Filter obstacle points by world-vertical height relative to the robot base."""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener, TransformException
from ugv_nav4d_ros2.msg import TravMap


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


class TerrainSurfaceGrid:
    """Vectorized lookup of the reachable floor surfaces in each map cell."""

    def __init__(self, resolution, patches):
        self.resolution = float(resolution)
        patches = [p for p in patches
                   if p.type_name in ('TRAVERSABLE', 'PARTIALLY_TRAVERSABLE')
                   and np.isfinite([p.position.x, p.position.y, p.position.z,
                                    p.a, p.b, p.c]).all() and abs(p.c) > 1e-6]
        self.layers = []
        if not patches or not np.isfinite(resolution) or resolution <= 0:
            return
        centers = np.array([[p.position.x, p.position.y] for p in patches])
        self.origin = centers.min(axis=0) - self.resolution / 2
        indices = np.rint((centers - self.origin) / self.resolution - .5).astype(int)
        self.width, self.height = indices.max(axis=0) + 1
        counts = {}
        for p, (x, y) in zip(patches, indices):
            key = y * self.width + x
            layer = counts.get(key, 0)
            if layer == len(self.layers):
                self.layers.append(np.full((self.width * self.height, 5), np.nan))
            self.layers[layer][key] = (p.position.x, p.position.y, p.position.z,
                                      p.a / p.c, p.b / p.c)
            counts[key] = layer + 1

    def ground_mask(self, points, base_z, clearance, max_step, tolerance):
        ground = np.zeros(len(points), dtype=bool)
        if not self.layers or not len(points):
            return ground
        indices = np.floor((points[:, :2] - self.origin) / self.resolution).astype(int)
        valid = ((indices[:, 0] >= 0) & (indices[:, 0] < self.width) &
                 (indices[:, 1] >= 0) & (indices[:, 1] < self.height))
        slots = np.flatnonzero(valid)
        keys = indices[valid, 1] * self.width + indices[valid, 0]
        samples = points[valid]
        for layer in self.layers:
            surfaces = layer[keys]
            height = (surfaces[:, 2] - surfaces[:, 3] * (samples[:, 0] - surfaces[:, 0])
                      - surfaces[:, 4] * (samples[:, 1] - surfaces[:, 1]))
            # Only a nearby reachable level can be ground. An upper floor's
            # underside must continue to count as an overhead obstacle.
            reachable = abs(height - (base_z - clearance)) <= max_step
            ground[slots] |= reachable & (abs(samples[:, 2] - height) <= tolerance)
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
        self.terrain_tolerance = float(self.declare_parameter('terrain_tolerance', 0.10).value)
        if not np.isfinite([self.minimum, self.maximum]).all() or self.minimum >= self.maximum:
            raise ValueError('Require finite min_relative_height < max_relative_height')
        if (not np.isfinite([self.clearance, self.max_step, self.terrain_tolerance]).all()
                or self.clearance < 0 or self.max_step <= 0 or self.terrain_tolerance <= 0):
            raise ValueError('Require finite ground_clearance >= 0 and positive step height and terrain tolerance')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(PointCloud2, '/go2/obstacle_points', qos_profile_sensor_data)
        self.clearing_publisher = self.create_publisher(PointCloud2, '/go2/clearing_points', qos_profile_sensor_data)
        self.terrain = None
        self.create_subscription(
            TravMap, '/ugv_nav4d_ros2/trav_map', self.receive_terrain,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pending = None
        self.create_subscription(PointCloud2, '/go2/points', self.receive, qos_profile_sensor_data)
        # A small retry queue lets TF arrive after its matching cloud; never use a
        # newer transform to project an older measurement. Only keep the newest cloud.
        self.create_timer(0.02, self.process)

    def receive(self, message):
        self.pending = message

    def receive_terrain(self, message):
        if message.header.frame_id != self.world:
            self.get_logger().warning('Ignoring traversability map in a different frame')
            return
        self.terrain = TerrainSurfaceGrid(message.resolution, message.patches)

    def process(self):
        message = self.pending
        if message is None:
            return
        stamp = rclpy.time.Time.from_msg(message.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        if age > 0.5 or age < -0.1:
            self.pending = None
            return
        try:
            cloud_tf = self.buffer.lookup_transform(self.world, message.header.frame_id, stamp).transform
            base_tf = self.buffer.lookup_transform(self.world, self.base, stamp).transform
        except TransformException as error:
            self.get_logger().warning(f'Waiting for cloud timestamp TF: {error}', throttle_duration_sec=5.0)
            return
        self.pending = None
        points = point_cloud2.read_points_numpy(message, field_names=('x', 'y', 'z'), skip_nans=False).reshape(-1, 3)
        rotation, translation = transform_arrays(cloud_tf)
        keep = height_mask(points, rotation, translation, base_tf.translation.z, self.minimum, self.maximum)
        clearing = height_mask(points, rotation, translation, base_tf.translation.z,
                               -self.clearance - self.max_step, self.maximum)
        if self.terrain is not None:
            slots = np.flatnonzero(keep)
            world_points = points[keep] @ rotation.T + translation
            keep[slots] &= ~self.terrain.ground_mask(
                world_points, base_tf.translation.z, self.clearance,
                self.max_step, self.terrain_tolerance)
        # Keep sensor frame and timestamp so Nav2 raytraces from the real sensor origin.
        self.publisher.publish(point_cloud2.create_cloud_xyz32(message.header, points[keep]))
        # Ground returns still clear old markings even when excluded above.
        self.clearing_publisher.publish(point_cloud2.create_cloud_xyz32(message.header, points[clearing]))


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
