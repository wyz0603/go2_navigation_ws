#!/usr/bin/env python3
"""Convert PCD to PLY using PCL, without resampling or changing coordinates."""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile


def convert(source, output=None, ascii_output=False):
    source = Path(source).expanduser().resolve()
    output = Path(output).expanduser().resolve() if output else source.with_suffix('.ply')
    if not source.is_file():
        raise ValueError(f'Input file does not exist: {source}')
    if source.suffix.lower() != '.pcd' or output.suffix.lower() != '.ply':
        raise ValueError('Input must be .pcd and output must be .ply')
    if source == output or (output.exists() and source.samefile(output)):
        raise ValueError('Input and output must be different files')
    executable = shutil.which('pcl_pcd2ply')
    if not executable:
        raise ValueError('pcl_pcd2ply not found. Install it with: sudo apt-get install pcl-tools')
    output.parent.mkdir(parents=True, exist_ok=True)
    # Conversion errors must not replace an existing output file.
    with tempfile.TemporaryDirectory(prefix='.pcd-to-ply-', dir=output.parent) as folder:
        temporary = Path(folder) / 'converted.ply'
        result = subprocess.run(
            [executable, '-format', '0' if ascii_output else '1', '-use_camera', '0',
             str(source), str(temporary)], capture_output=True, text=True)
        if result.returncode != 0 or not temporary.is_file():
            raise ValueError(f'PCL conversion failed:\n{result.stdout}{result.stderr}')
        count = None
        with temporary.open('rb') as stream:
            if stream.readline().strip() != b'ply':
                raise ValueError('PCL did not produce a PLY file')
            for _ in range(1000):
                line = stream.readline().strip()
                if line.startswith(b'element vertex '):
                    count = int(line.split()[-1])
                if line == b'end_header':
                    break
            else:
                raise ValueError('Invalid PLY header')
        if count is None or count <= 0:
            raise ValueError('Converted PLY contains no vertices')
        temporary.replace(output)
    return output, count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path, help='Input PCD (ASCII, binary or binary_compressed)')
    parser.add_argument('output', type=Path, nargs='?', help='Default: input filename with .ply suffix')
    parser.add_argument('--ascii', action='store_true', help='Write ASCII PLY instead of binary PLY')
    args = parser.parse_args()
    try:
        output, count = convert(args.input, args.output, args.ascii)
    except (ValueError, OSError) as error:
        parser.exit(1, f'Conversion failed: {error}\n')
    print(f'Converted {count:,} points -> {output}')


if __name__ == '__main__':
    main()
