"""NumPy/SciPy-backed PCD point cloud utilities compatible with Open3D PointCloud API."""
from __future__ import annotations
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components


class PointCloud:
    def __init__(self, points: np.ndarray | None = None) -> None:
        if points is None:
            self.points = np.empty((0, 3), dtype=np.float32)
        else:
            arr = np.asarray(points, dtype=np.float32)
            self.points = arr.reshape((-1, 3)) if arr.size else np.empty((0, 3), dtype=np.float32)

    def has_points(self) -> bool:
        return self.points.shape[0] > 0

    def select_by_index(self, indices, invert: bool = False) -> "PointCloud":
        idx = np.asarray(indices, dtype=np.int64)
        if invert:
            mask = np.ones(self.points.shape[0], dtype=bool)
            if idx.size:
                mask[idx] = False
            return PointCloud(self.points[mask])
        if idx.size == 0:
            return PointCloud()
        return PointCloud(self.points[idx])

    def voxel_down_sample(self, voxel_size: float) -> "PointCloud":
        if voxel_size <= 0.0 or not self.has_points():
            return PointCloud(self.points.copy())
        keys = np.floor(self.points / float(voxel_size)).astype(np.int64)
        _, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
        sums = np.zeros((counts.shape[0], 3), dtype=np.float64)
        np.add.at(sums, inv, self.points)
        down = (sums / counts[:, None]).astype(np.float32)
        return PointCloud(down)

    def remove_statistical_outlier(self, nb_neighbors: int = 20, std_ratio: float = 2.0):
        n = self.points.shape[0]
        if n <= nb_neighbors:
            idx = np.arange(n, dtype=np.int64)
            return PointCloud(self.points.copy()), idx.tolist()
        tree = cKDTree(self.points)
        dists, _ = tree.query(self.points, k=min(nb_neighbors + 1, n))
        mean_d = np.mean(dists[:, 1:], axis=1)
        thresh = float(np.mean(mean_d) + std_ratio * np.std(mean_d))
        inliers = np.where(mean_d <= thresh)[0]
        return self.select_by_index(inliers), inliers.tolist()

    def remove_radius_outlier(self, nb_points: int = 12, radius: float = 0.2):
        n = self.points.shape[0]
        if n == 0:
            return PointCloud(), []
        tree = cKDTree(self.points)
        counts = tree.query_ball_point(self.points, r=float(radius), return_length=True)
        inliers = np.where(np.asarray(counts) - 1 >= nb_points)[0]
        return self.select_by_index(inliers), inliers.tolist()

    def cluster_dbscan(self, eps: float = 0.2, min_points: int = 20, print_progress: bool = False):
        n = self.points.shape[0]
        if n == 0:
            return np.empty((0,), dtype=np.int32)
        tree = cKDTree(self.points)
        neighbors = tree.query_ball_point(self.points, r=float(eps))
        counts = np.array([len(nb) for nb in neighbors], dtype=np.int32)
        core_mask = counts >= min_points
        labels = np.full(n, -1, dtype=np.int32)
        if not np.any(core_mask):
            return labels
        pairs = tree.query_pairs(r=float(eps), output_type="ndarray")
        if pairs.size == 0:
            core_idx = np.where(core_mask)[0]
            labels[core_idx] = np.arange(core_idx.size, dtype=np.int32)
            return labels
        data = np.ones(pairs.shape[0] * 2, dtype=np.int8)
        row = np.concatenate([pairs[:, 0], pairs[:, 1]])
        col = np.concatenate([pairs[:, 1], pairs[:, 0]])
        adj = csr_matrix((data, (row, col)), shape=(n, n))
        _, comp = connected_components(adj, directed=False)
        valid_comps = np.unique(comp[core_mask])
        remap = {int(c): idx for idx, c in enumerate(valid_comps)}
        for i in range(n):
            c = int(comp[i])
            if c in remap:
                labels[i] = remap[c]
        return labels

    def segment_plane(self, distance_threshold: float = 0.05, ransac_n: int = 3, num_iterations: int = 1000):
        pts = self.points
        n = pts.shape[0]
        if n < max(3, ransac_n):
            return [0.0, 0.0, 1.0, 0.0], []
        rng = np.random.default_rng(42)
        best_inliers = np.empty((0,), dtype=np.int64)
        best_model = [0.0, 0.0, 1.0, 0.0]
        sample_pts = pts if n <= 50000 else pts[rng.choice(n, 50000, replace=False)]
        sn = sample_pts.shape[0]
        for _ in range(min(num_iterations, 250)):
            idx = rng.choice(sn, 3, replace=False)
            p0, p1, p2 = sample_pts[idx[0]], sample_pts[idx[1]], sample_pts[idx[2]]
            normal = np.cross(p1 - p0, p2 - p0)
            norm = float(np.linalg.norm(normal))
            if norm < 1e-8:
                continue
            normal = normal / norm
            d = -float(np.dot(normal, p0))
            dist = np.abs(sample_pts @ normal + d)
            inlier_count = int(np.count_nonzero(dist <= distance_threshold))
            if inlier_count > best_inliers.size:
                full_dist = np.abs(pts @ normal + d)
                best_inliers = np.where(full_dist <= distance_threshold)[0]
                best_model = [float(normal[0]), float(normal[1]), float(normal[2]), d]
        return best_model, best_inliers.tolist()


class KDTreeFlann:
    def __init__(self, cloud: PointCloud) -> None:
        self._tree = cKDTree(cloud.points)

    def search_knn_vector_3d(self, query_pt, k: int):
        dists, idxs = self._tree.query(np.asarray(query_pt, dtype=np.float32), k=k)
        dists = np.atleast_1d(dists)
        idxs = np.atleast_1d(idxs)
        sq = (dists ** 2).tolist()
        return len(sq), idxs.tolist(), sq


class _IO:
    @staticmethod
    def read_point_cloud(path: str) -> PointCloud:
        p = Path(path)
        raw = p.read_bytes()
        pos = 0
        fields: list[str] = []
        sizes: list[int] = []
        types: list[str] = []
        counts: list[int] = []
        points_count = 0
        width = 0
        height = 1
        data_mode = ""
        while pos < len(raw):
            nl = raw.find(b"\n", pos)
            if nl < 0:
                break
            line = raw[pos:nl].decode("utf-8", errors="ignore").strip()
            pos = nl + 1
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            key = parts[0].upper()
            if key == "FIELDS":
                fields = parts[1:]
            elif key == "SIZE":
                sizes = [int(x) for x in parts[1:]]
            elif key == "TYPE":
                types = parts[1:]
            elif key == "COUNT":
                counts = [int(x) for x in parts[1:]]
            elif key == "WIDTH":
                width = int(parts[1])
            elif key == "HEIGHT":
                height = int(parts[1])
            elif key == "POINTS":
                points_count = int(parts[1])
            elif key == "DATA":
                data_mode = parts[1].lower()
                break
        if points_count == 0:
            points_count = width * height
        if not counts:
            counts = [1] * len(fields)
        if data_mode == "ascii":
            body = raw[pos:].decode("utf-8", errors="ignore")
            arr = np.fromstring(body, sep=" ", dtype=np.float32)
            cols = sum(counts)
            if cols > 0 and arr.size >= cols:
                arr = arr[: (arr.size // cols) * cols].reshape((-1, cols))
                xi, yi, zi = fields.index("x"), fields.index("y"), fields.index("z")
                pts = np.stack([arr[:, xi], arr[:, yi], arr[:, zi]], axis=1)
                pts = pts[np.all(np.isfinite(pts), axis=1)]
                return PointCloud(pts)
            return PointCloud()
        if data_mode == "binary":
            dtype_fields = []
            for f, s, t, c in zip(fields, sizes, types, counts):
                code = f"{'<' if s > 1 else ''}{t.lower()}{s}"
                dtype_fields.append((f, code) if c == 1 else (f, code, (c,)))
            dt = np.dtype(dtype_fields)
            max_items = min(points_count, (len(raw) - pos) // dt.itemsize)
            rec = np.frombuffer(raw, dtype=dt, count=max_items, offset=pos)
            pts = np.stack(
                [rec["x"].astype(np.float32), rec["y"].astype(np.float32), rec["z"].astype(np.float32)],
                axis=1,
            )
            pts = pts[np.all(np.isfinite(pts), axis=1)]
            return PointCloud(pts)
        raise ValueError(f"Unsupported PCD DATA mode: {data_mode}")

    @staticmethod
    def write_point_cloud(path: str, cloud: PointCloud, write_ascii: bool = False) -> bool:
        pts = np.asarray(cloud.points, dtype=np.float32).reshape((-1, 3))
        n = pts.shape[0]
        mode = "ascii" if write_ascii else "binary"
        header = (
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            f"WIDTH {n}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {n}\n"
            f"DATA {mode}\n"
        )
        p = Path(path)
        with p.open("wb") as f:
            f.write(header.encode("ascii"))
            if write_ascii:
                np.savetxt(f, pts, fmt="%.6f %.6f %.6f")
            else:
                f.write(np.ascontiguousarray(pts, dtype="<f4").tobytes())
        return True


class _Geometry:
    PointCloud = PointCloud
    KDTreeFlann = KDTreeFlann


io = _IO()
geometry = _Geometry()
