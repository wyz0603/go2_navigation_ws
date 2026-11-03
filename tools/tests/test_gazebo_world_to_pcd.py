import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gazebo_world_to_pcd import Converter, write_pcd, np, trimesh


class CollisionConversionTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def sdf(self, text, name='world.sdf'):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
        return path

    def test_include_override_nested_pose_and_visual_exclusion(self):
        self.sdf('''<sdf version="1.7"><model name="source"><pose>99 0 0 0 0 0</pose>
          <link name="link"><pose>2 0 0 0 0 0</pose>
          <collision name="c"><pose>1 0 0 0 0 0</pose><geometry><box><size>2 2 2</size></box></geometry></collision>
          <visual name="ignored"><geometry><mesh><uri>does_not_exist.dae</uri></mesh></geometry></visual>
          </link></model></sdf>''', 'asset/model.sdf')
        world = self.sdf('''<sdf version="1.7"><world name="w"><model name="parent">
          <pose>10 0 0 0 0 1.5707963267948966</pose>
          <include><uri>model://asset</uri><pose>1 0 0 0 0 0</pose></include>
          </model></world></sdf>''')
        converter = Converter([self.root])
        converter.read(world)
        self.assertEqual(len(converter.collisions), 1)
        np.testing.assert_allclose(converter.collisions[0].bounds, [[9, 3, -1], [11, 5, 1]], atol=1e-8)

    def test_mesh_scale_and_collision_offset(self):
        trimesh.creation.box(extents=[1, 1, 1]).export(self.root / 'collision.stl')
        world = self.sdf('''<sdf><model name="m"><link name="l"><collision name="c">
          <pose>3 0 0 0 0 0</pose><geometry><mesh><uri>collision.stl</uri>
          <scale>2 4 6</scale></mesh></geometry></collision></link></model></sdf>''')
        converter = Converter()
        converter.read(world)
        np.testing.assert_allclose(converter.collisions[0].bounds, [[2, -2, -3], [4, 2, 3]])

    def test_plane_bounds_binary_ascii_and_reproducibility(self):
        world = self.sdf('''<sdf><world name="w"><model name="m"><link name="l"><collision name="c">
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
          </collision></link></model></world></sdf>''')
        with self.assertRaisesRegex(ValueError, 'infinite'):
            Converter().read(world)
        converter = Converter(bounds=[-2, -3, -1, 2, 3, 1])
        converter.read(world)
        points = converter.sample(0.1)
        np.testing.assert_array_equal(points, converter.sample(0.1))
        self.assertTrue(np.all(points[:, 2] == 0))
        self.assertGreater(points[:, 0].max(), 1.9)  # Infinite plane, not visual 1x1 size.
        self.assertTrue(np.all(abs(points[:, 1]) <= 3))
        with self.assertRaisesRegex(ValueError, 'samples required'):
            converter.sample(0.001, max_samples=100)
        for ascii_output in [False, True]:
            output = self.root / 'result.pcd'
            write_pcd(output, points, ascii_output)
            raw = output.read_bytes()
            marker = b'DATA ascii\n' if ascii_output else b'DATA binary\n'
            header, body = raw.split(marker, 1)
            self.assertIn(f'POINTS {len(points)}'.encode(), header)
            if ascii_output:
                import io
                restored = np.loadtxt(io.BytesIO(body))
            else:
                restored = np.frombuffer(body, dtype='<f4').reshape(-1, 3)
            np.testing.assert_allclose(points, restored, atol=1e-7)

    def test_reject_unsupported_and_include_cycle(self):
        path = self.sdf('''<sdf><model name="m"><pose relative_to="other">0 0 0 0 0 0</pose></model></sdf>''')
        with self.assertRaisesRegex(ValueError, 'relative_to'):
            Converter().read(path)
        path = self.sdf('''<sdf><model name="m"><include><uri>world.sdf</uri></include></model></sdf>''')
        with self.assertRaisesRegex(ValueError, 'Cyclic'):
            Converter().read(path)


if __name__ == '__main__':
    unittest.main()
