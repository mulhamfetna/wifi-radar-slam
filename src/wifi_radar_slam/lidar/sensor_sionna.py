"""LiDAR model B: Sionna ray-traced optical-return proxy (monostatic + diffuse).

Pure helpers here are NumPy-only and test locally. The Sionna PathSolver machinery
(SionnaLidarSensor) lazily imports sionna.rt/mitsuba inside its methods, so this
module imports without Sionna and only *running* the sensor needs the amd server.
"""
from __future__ import annotations
import numpy as np
from ..geometry import RX_HEIGHT_M
from .pointcloud import Scan


def _voxel_downsample(pts: np.ndarray, voxel: float) -> np.ndarray:
    """Keep one point per `voxel`-sized xy cell (caps point density)."""
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    if pts.shape[0] == 0:
        return pts
    seen: dict[tuple[int, int], np.ndarray] = {}
    for p in pts:
        key = (int(round(p[0] / voxel)), int(round(p[1] / voxel)))
        seen.setdefault(key, p)
    return np.array(list(seen.values()))


def vertices_to_scan(world_hits, pose, cfg, rng, scan_voxel: float = 0.2) -> Scan:
    """Convert world-frame hit points to a sensor-local Scan.

    Filters by [min_range, max_range], adds radial Gaussian range noise
    (cfg.range_sigma_m), rotates world->local by -yaw, and voxel-downsamples.
    """
    world_hits = np.asarray(world_hits, dtype=float).reshape(-1, 2)
    px, py = float(pose[0]), float(pose[1])
    yaw = float(pose[2]) if len(pose) > 2 else 0.0
    if world_hits.shape[0] == 0:
        return Scan.empty()
    rel = world_hits - np.array([px, py])
    r = np.linalg.norm(rel, axis=1)
    keep = (r >= cfg.min_range_m) & (r <= cfg.max_range_m)
    rel, r = rel[keep], r[keep]
    if rel.shape[0] == 0:
        return Scan.empty()
    if cfg.range_sigma_m > 0:
        u = rel / np.maximum(r[:, None], 1e-9)
        rel = rel + u * rng.normal(0, cfg.range_sigma_m, size=r.shape)[:, None]
    c, s = np.cos(-yaw), np.sin(-yaw)          # world -> local: rotate by -yaw
    R = np.array([[c, -s], [s, c]])
    local = rel @ R.T
    return Scan(_voxel_downsample(local, scan_voxel))


class SionnaLidarSensor:
    """Model B: monostatic Sionna LiDAR. A TX co-located with the vehicle RX plus
    diffuse material backscatter; single-scatter interaction vertices are returns.
    """

    def __init__(self, built, cfg, rng, scattering: float = 0.7,
                 max_depth: int = 2, scan_voxel: float = 0.2):
        import sionna.rt as rt          # lazy: server only
        self.built, self.cfg, self.rng = built, cfg, rng
        self.max_depth, self.scan_voxel = max_depth, scan_voxel
        self.scene = built.scene
        if "lidar_tx" not in self.scene.transmitters:
            self.scene.add(rt.Transmitter("lidar_tx",
                                          position=[0.0, 0.0, RX_HEIGHT_M]))
        for m in self.scene.radio_materials.values():
            try:
                m.scattering_coefficient = scattering    # enable diffuse backscatter
            except Exception:
                pass
        self.solver = rt.PathSolver()
        self.lidx = list(self.scene.transmitters.keys()).index("lidar_tx")
        self.rx = self.scene.receivers["veh"]
        self.floor_ids = {o.object_id for n, o in self.scene.objects.items()
                          if "floor" in n.lower()}

    def __call__(self, pose):
        import os
        import mitsuba as mi
        px, py = float(pose[0]), float(pose[1])
        self.scene.transmitters["lidar_tx"].position = mi.Point3f(px, py, RX_HEIGHT_M)
        self.rx.position = mi.Point3f(px, py, RX_HEIGHT_M)
        ns = int(os.environ.get("WRS_NUM_SAMPLES", "1000000"))
        paths = self.solver(self.scene, max_depth=self.max_depth, samples_per_src=ns,
                            diffuse_reflection=True,
                            seed=int(self.rng.integers(1, 2**31 - 1)))
        inter = np.asarray(paths.interactions.numpy())[:, 0][:, self.lidx]   # (depth,n_paths)
        valid = np.asarray(paths.valid.numpy())[0][self.lidx]               # (n_paths,)
        objs = np.asarray(paths.objects.numpy())[0, 0][self.lidx]           # depth-0 obj id
        verts = np.asarray(paths.vertices.numpy())[:, 0, self.lidx]         # (depth,n_paths,3)
        ss = (np.count_nonzero(inter, axis=0) == 1) & valid
        hits = []
        for p in np.where(ss)[0]:
            if int(objs[p]) in self.floor_ids:            # drop ground bounces
                continue
            d = int(np.argmax(inter[:, p] != 0))          # the single interaction depth
            hits.append(verts[d, p, :2])
        world = np.array(hits) if hits else np.empty((0, 2))
        return vertices_to_scan(world, pose, self.cfg, self.rng, self.scan_voxel)


def sionna_lidar_sensor(built, cfg, rng) -> "SionnaLidarSensor":
    """make_sensor factory for model B (monostatic Sionna optical-ray LiDAR)."""
    return SionnaLidarSensor(built, cfg, rng)
