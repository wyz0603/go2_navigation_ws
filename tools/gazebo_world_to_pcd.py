#!/usr/bin/env python3
"""Sample SDF collision surfaces into a world-coordinate XYZ PCD, offline."""
import argparse
import itertools
import math
import os
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET
from urllib.parse import unquote

# Optional isolated dependencies; no changes to the ROS Python installation.
sys.path.insert(0, str(Path(__file__).resolve().parent / '.deps'))
import numpy as np
import trimesh


def numbers(text, count):
    values = np.array([float(x) for x in text.split()])
    if len(values) != count or not np.isfinite(values).all():
        raise ValueError(f'Expected {count} finite values: {text!r}')
    return values


def pose(element):
    transform = np.eye(4)
    if element is None:
        return transform
    if element.get('relative_to'):
        raise ValueError('pose relative_to is not supported; export poses relative to their parent first')
    if element.get('rotation_format', 'euler_rpy') != 'euler_rpy':
        raise ValueError('Only euler_rpy poses are supported')
    values = numbers(element.text or '', 6)
    angles = values[3:]
    if element.get('degrees', 'false').lower() in ('true', '1'):
        angles = np.deg2rad(angles)
    transform = trimesh.transformations.euler_matrix(*angles, axes='sxyz')
    transform[:3, 3] = values[:3]
    return transform


class Converter:
    def __init__(self, resource_paths=(), bounds=None):
        self.paths = [Path(p).expanduser().resolve() for p in resource_paths if p]
        self.bounds = None if bounds is None else np.asarray(bounds).reshape(2, 3)
        self.collisions = []
        self.stack = []

    def resolve(self, uri, source):
        uri = unquote(uri.strip())
        if uri.startswith('model://'):
            tail = uri[len('model://'):]
            candidates = [p / tail for p in self.paths]
        elif uri.startswith('package://'):
            tail = uri[len('package://'):]
            candidates = [Path(p) / 'share' / tail for p in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep) if p]
            candidates += [p / tail for p in self.paths]
        elif uri.startswith('file://'):
            candidates = [Path(uri[len('file://'):])]
        elif '://' in uri:
            raise ValueError(f'Remote URI not supported; download model locally: {uri}')
        else:
            candidates = [source.parent / uri] + [p / uri for p in self.paths]
        for candidate in candidates:
            if candidate.exists():
                return candidate.resolve()
        raise ValueError(f'Cannot resolve {uri!r} from {source}; add --resource-path')

    def read(self, filename, parent=None, include=None):
        filename = Path(filename).resolve()
        if filename.is_dir():
            config = filename / 'model.config'
            sdf = ET.parse(config).getroot().find('sdf') if config.exists() else None
            filename = filename / (sdf.text.strip() if sdf is not None else 'model.sdf')
        if filename in self.stack:
            raise ValueError(f'Cyclic include: {filename}')
        self.stack.append(filename)
        try:
            root = ET.parse(filename).getroot()
            if root.tag != 'sdf':
                raise ValueError(f'Expected SDF document: {filename}')
            containers = list(root.findall('world')) + list(root.findall('model'))
            if not containers:
                raise ValueError(f'No world or model in {filename}')
            if include is not None and (len(containers) != 1 or containers[0].tag != 'model'):
                raise ValueError('Includes must contain exactly one model')
            for element in containers:
                self.walk(element, np.eye(4) if parent is None else parent, filename, include)
        finally:
            self.stack.pop()

    def walk(self, element, parent, source, include=None):
        if element.get('placement_frame') or element.find('placement_frame') is not None:
            raise ValueError('placement_frame is not supported')
        local_pose = element.find('pose')
        if include is not None and include.find('pose') is not None:
            local_pose = include.find('pose')  # SDF include pose replaces model pose.
        transform = parent @ pose(local_pose)
        for child in element:
            if child.tag in ('model', 'link'):
                self.walk(child, transform, source)
            elif child.tag == 'include':
                if child.get('merge', 'false') in ('true', '1') or child.find('placement_frame') is not None:
                    raise ValueError('Merged includes / placement_frame are not supported')
                self.read(self.resolve(child.findtext('uri', ''), source), transform, child)
            elif child.tag == 'collision':
                geometry = child.find('geometry')
                if geometry is None or len(geometry) != 1:
                    raise ValueError('Collision must have exactly one geometry')
                mesh = self.geometry(geometry[0], source, transform @ pose(child.find('pose')))
                if not np.isfinite(mesh.vertices).all() or len(mesh.faces) == 0:
                    raise ValueError(f'Invalid or empty collision mesh in {source}')
                self.collisions.append(mesh)

    def geometry(self, shape, source, transform):
        tag = shape.tag
        if tag == 'mesh':
            if shape.find('submesh') is not None:
                raise ValueError('Mesh submesh selection is not supported')
            filename = self.resolve(shape.findtext('uri', ''), source)
            loaded = trimesh.load(str(filename), force='scene')
            mesh = loaded.to_mesh()
            scale = numbers(shape.findtext('scale', '1 1 1'), 3)
            if np.any(scale == 0):
                raise ValueError('Mesh scale must be nonzero')
            mesh.apply_scale(scale)
        elif tag == 'box':
            size = numbers(shape.findtext('size', ''), 3)
            if np.any(size <= 0):
                raise ValueError('Box dimensions must be positive')
            mesh = trimesh.creation.box(extents=size)
        elif tag in ('sphere', 'cylinder'):
            radius = float(shape.findtext('radius', 'nan'))
            if not math.isfinite(radius) or radius <= 0:
                raise ValueError('Radius must be positive and finite')
            if tag == 'sphere':
                mesh = trimesh.creation.icosphere(subdivisions=4, radius=radius)
            else:
                length = float(shape.findtext('length', 'nan'))
                if not math.isfinite(length) or length <= 0:
                    raise ValueError('Cylinder length must be positive and finite')
                mesh = trimesh.creation.cylinder(radius=radius, height=length, sections=128)
        elif tag == 'plane':
            if self.bounds is None:
                raise ValueError('SDF collision planes are infinite; supply --bounds xmin ymin zmin xmax ymax zmax')
            normal = numbers(shape.findtext('normal', '0 0 1'), 3)
            if np.linalg.norm(normal) == 0:
                raise ValueError('Plane normal must be nonzero')
            normal /= np.linalg.norm(normal)
            tangent = np.cross(normal, np.eye(3)[np.argmin(abs(normal))])
            tangent /= np.linalg.norm(tangent)
            other = np.cross(normal, tangent)
            corners = np.array(list(itertools.product(*zip(*self.bounds))))
            local = trimesh.transform_points(corners, np.linalg.inv(transform))
            uv = local @ np.column_stack((tangent, other))
            low, high = uv.min(axis=0), uv.max(axis=0)
            vertices = np.array([u * tangent + v * other for u, v in
                                 [(low[0], low[1]), (high[0], low[1]),
                                  (high[0], high[1]), (low[0], high[1])]])
            mesh = trimesh.Trimesh(vertices=vertices, faces=[[0, 1, 2], [0, 2, 3]], process=False)
        else:
            raise ValueError(f'Unsupported collision geometry: {tag}')
        mesh.apply_transform(transform)
        return mesh

    def sample(self, spacing, seed=0, max_samples=5000000):
        if not self.collisions:
            raise ValueError('No collision surfaces found')
        counts = [max(1, math.ceil(mesh.area / spacing ** 2)) for mesh in self.collisions]
        total = sum(counts)
        if total > max_samples:
            raise ValueError(f'{total:,} samples required; increase --spacing or --max-samples')
        rng = np.random.default_rng(seed)
        chunks = []
        for mesh, count in zip(self.collisions, counts):
            points, _ = trimesh.sample.sample_surface(mesh, count, seed=rng)
            if self.bounds is not None:
                points = points[np.all((points >= self.bounds[0]) & (points <= self.bounds[1]), axis=1)]
            chunks.append(points)
        points = np.concatenate(chunks)
        if len(points) == 0:
            raise ValueError('No points inside export bounds')
        # Density is approximate; keep one sampled point in each spacing-sized voxel.
        _, indices = np.unique(np.floor(points / spacing).astype(np.int64), axis=0, return_index=True)
        return points[np.sort(indices)].astype('<f4')


def write_pcd(filename, points, ascii_output=False):
    filename = Path(filename)
    filename.parent.mkdir(parents=True, exist_ok=True)
    header = ('# .PCD v0.7 - collision surfaces in SDF world coordinates\n'
              'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\n'
              f'DATA {"ascii" if ascii_output else "binary"}\n')
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=filename.parent, delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(header.encode('ascii'))
            if ascii_output:
                np.savetxt(stream, points, fmt='%.8g')
            else:
                stream.write(points.astype('<f4').tobytes())
        os.replace(temporary, filename)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('world', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--resource-path', action='append', default=[])
    parser.add_argument('--bounds', type=float, nargs=6, metavar=('XMIN', 'YMIN', 'ZMIN', 'XMAX', 'YMAX', 'ZMAX'))
    parser.add_argument('--spacing', type=float, default=0.05, help='Approximate sampling / voxel spacing in metres')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--max-samples', type=int, default=5000000)
    parser.add_argument('--ascii', action='store_true')
    args = parser.parse_args()
    try:
        if not math.isfinite(args.spacing) or args.spacing <= 0 or args.max_samples <= 0:
            raise ValueError('spacing and max-samples must be positive')
        if args.bounds is not None:
            b = np.asarray(args.bounds).reshape(2, 3)
            if not np.isfinite(b).all() or np.any(b[0] >= b[1]):
                raise ValueError('Bounds must be finite and min < max on every axis')
        if args.world.resolve() == args.output.resolve():
            raise ValueError('Output must not overwrite the input world')
        paths = args.resource_path + [str(args.world.resolve().parent),
                str(args.world.resolve().parent.parent / 'models'),
                str(Path(__file__).resolve().parents[1] / 'src')]
        for variable in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH', 'GAZEBO_MODEL_PATH'):
            paths.extend(os.environ.get(variable, '').split(os.pathsep))
        converter = Converter(paths, args.bounds)
        converter.read(args.world)
        points = converter.sample(args.spacing, args.seed, args.max_samples)
        write_pcd(args.output, points, args.ascii)
        print(f'{len(converter.collisions)} collisions -> {len(points):,} points: {args.output}')
        print(f'World XYZ bounds: {points.min(axis=0)} .. {points.max(axis=0)}')
    except (ValueError, OSError, ET.ParseError, ImportError) as error:
        parser.exit(1, f'Conversion failed: {error}\n')


if __name__ == '__main__':
    main()
