import sys
from pathlib import Path
import unittest
import numpy as np
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/go2_bringup/scripts'))
from obstacle_height_filter import height_mask, TerrainSurfaceGrid


class HeightFilterTest(unittest.TestCase):
    def test_elevated_robot_same_obstacles(self):
        points = np.array([[0, 0, -.35], [1, 0, 0], [1, 0, 1], [np.nan, 0, 0], [np.inf, 0, 0]])
        for floor in (0., 3., -2.):
            mask = height_mask(points, np.eye(3), np.array([0, 0, floor+.35]), floor+.35, -.25, .45)
            np.testing.assert_array_equal(mask, [False, True, False, False, False])

    def test_pitched_sensor_uses_world_vertical(self):
        rotation = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
        # Sensor X is world -Z; points with identical sensor Z differ in height.
        points = np.array([[0, 0, 1], [-1, 0, 1], [1, 0, 1]])
        mask = height_mask(points, rotation, np.array([0., 0., .5]), .35, -.25, .45)
        np.testing.assert_array_equal(mask, [True, False, False])

    def test_ramp_is_ground_but_object_on_ramp_is_an_obstacle(self):
        patch = SimpleNamespace(type_name='TRAVERSABLE',
                                position=SimpleNamespace(x=1., y=0., z=.2),
                                a=-.2, b=0., c=1.)
        terrain = TerrainSurfaceGrid(.4, [patch])
        points = np.array([[1.1, 0., .22], [1.1, 0., .52], [2., 0., .4]])
        np.testing.assert_array_equal(
            terrain.ground_mask(points, .35, .35, .45, .1),
            [True, False, False])

    def test_overlapping_floors_do_not_hide_overhead_obstacles(self):
        patches = [SimpleNamespace(type_name='TRAVERSABLE',
                                   position=SimpleNamespace(x=0., y=0., z=z),
                                   a=0., b=0., c=1.) for z in (0., 3., 6.)]
        terrain = TerrainSurfaceGrid(.2, patches)
        points = np.array([[0., 0., 3.], [0., 0., 3.3]])
        np.testing.assert_array_equal(
            terrain.ground_mask(points, 3.35, .35, .45, .1), [True, False])
        np.testing.assert_array_equal(
            terrain.ground_mask(points, 2.65, .35, .45, .1), [False, False])

    def test_unknown_or_obstacle_cells_are_not_removed(self):
        patch = SimpleNamespace(type_name='OBSTACLE',
                                position=SimpleNamespace(x=0., y=0., z=.2),
                                a=0., b=0., c=1.)
        terrain = TerrainSurfaceGrid(.2, [patch])
        np.testing.assert_array_equal(
            terrain.ground_mask(np.array([[0., 0., .2]]), .35, .35, .45, .1),
            [False])
