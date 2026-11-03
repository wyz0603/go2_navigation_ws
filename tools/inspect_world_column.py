#!/usr/bin/env python3
"""Report collision-surface heights along a world-coordinate vertical column."""
import argparse
import numpy as np
from gazebo_world_to_pcd import Converter


def column_hits(mesh, x, y):
    triangles = mesh.triangles
    origin = triangles[:, 0]
    u = triangles[:, 1] - origin
    v = triangles[:, 2] - origin
    determinant = u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0]
    valid = np.abs(determinant) > 1e-10
    origin, u, v, determinant = origin[valid], u[valid], v[valid], determinant[valid]
    delta = np.array([x, y]) - origin[:, :2]
    alpha = (delta[:, 0] * v[:, 1] - delta[:, 1] * v[:, 0]) / determinant
    beta = (u[:, 0] * delta[:, 1] - u[:, 1] * delta[:, 0]) / determinant
    inside = (alpha >= -1e-8) & (beta >= -1e-8) & (alpha + beta <= 1 + 1e-8)
    return np.unique(np.round((origin[:, 2] + alpha * u[:, 2] + beta * v[:, 2])[inside], 6))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('world')
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('--resource-path', action='append', default=[])
    args = parser.parse_args()
    from pathlib import Path
    world = Path(args.world).resolve()
    converter = Converter(args.resource_path + [str(world.parent.parent / 'models')],
                          [args.x - 1, args.y - 1, -100, args.x + 1, args.y + 1, 100])
    converter.read(world)
    for index, mesh in enumerate(converter.collisions):
        heights = column_hits(mesh, args.x, args.y)
        if len(heights):
            print(f'collision[{index}]: ' + ', '.join(f'{z:.6f}' for z in heights))


if __name__ == '__main__':
    main()
