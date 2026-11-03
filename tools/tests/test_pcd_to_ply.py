import shutil
import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pcd_to_ply import convert


@unittest.skipUnless(shutil.which('pcl_pcd2ply'), 'requires pcl-tools')
class ConversionTest(unittest.TestCase):
    def test_ascii_and_binary_inputs_and_outputs(self):
        header = ('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                  'WIDTH 2\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 2\n')
        values = (-1.25, 2.5, 0.0, 3.0, -4.0, 5.5)
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / 'cloud.pcd'
            for binary in (False, True):
                source.write_bytes((header + ('DATA binary\n' if binary else 'DATA ascii\n')).encode()
                                   + (struct.pack('<6f', *values) if binary else b'-1.25 2.5 0\n3 -4 5.5\n'))
                for ascii_output in (False, True):
                    path, count = convert(source, ascii_output=ascii_output)
                    self.assertEqual(count, 2)
                    metadata, payload = path.read_bytes().split(b'end_header\n', 1)
                    self.assertIn(b'element vertex 2', metadata)
                    if ascii_output:
                        recovered = tuple(float(v) for v in payload.split())
                    else:
                        recovered = struct.unpack('<6f', payload[:24])
                    self.assertEqual(recovered, values)

    def test_failure_preserves_existing_output(self):
        with tempfile.TemporaryDirectory() as folder:
            source, target = Path(folder) / 'bad.pcd', Path(folder) / 'cloud.ply'
            source.write_text('not a PCD')
            target.write_bytes(b'previous output')
            with self.assertRaises(ValueError):
                convert(source, target)
            self.assertEqual(target.read_bytes(), b'previous output')


if __name__ == '__main__':
    unittest.main()
