# Paper 2 LiDAR Harness (branch 0) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the shared LiDAR SLAM + metrics substrate (`src/wifi_radar_slam/lidar/`) that the three LiDAR sensor models (branches 1–3) plug into, so WiFi and LiDAR are measured apples-to-apples on the same 2D BEV ground truth.

**Architecture:** A new additive package `lidar/` with: a datasheet-pinned `LidarConfig`, a `Scan` point-cloud container (2D sensor-local points ↔ world), a point-to-point `icp_align`, a scan-to-map `run_lidar_slam` returning `(trajectory, map)`, a built-in `ReferenceSensor` (analytic 2D ray-cast over the footprint GT, used to test the substrate and as the reusable sensor seam), and a `run_lidar` runner that emits the **same six metrics** as the WiFi runner. Sensor models A/B/C (later branches) supply their own `make_sensor` factory against the same seam.

**Tech Stack:** Python 3, NumPy only (no SciPy). `dataclasses`, `pytest`. Reuses `wifi_radar_slam.eval.metrics` and `wifi_radar_slam.geometry.velocity_from_poses`.

## Global Constraints

- **Branch:** all work on `paper2-lidar-harness`, cut from `paper2-wifi-vs-lidar`; merge back on completion. Do NOT commit to `main` or any `paper1-*` ref.
- **NumPy only** — no SciPy, no sklearn, no new third-party deps.
- **2D BEV comparison plane** — scans, trajectories, and maps are all `xy`; LiDAR reuses `eval/metrics.py` unchanged (ate, rpe, chamfer, map_accuracy, map_completeness, occupancy_iou).
- **Pure-Python tests run in the default suite** — no Sionna/Mitsuba import anywhere in `lidar/` (models A/B add those later, gated). The whole `lidar/` package and its tests must import and run without Sionna installed.
- **RNG is always passed in** (`np.random.default_rng`), never created inside library code — matches the existing pipeline for determinism.
- **Pose convention:** a pose is `(x, y, yaw)`; a sensor→world transform rotates local points by `yaw` then translates by `(x, y)`. `Scan.to_world`, `icp_align`, and `run_lidar_slam` all use this one convention.
- **Frozen dataclasses** for config, matching `config.py`.

---

### Task 1: `LidarConfig` + package scaffold

**Files:**
- Create: `src/wifi_radar_slam/lidar/__init__.py`
- Create: `src/wifi_radar_slam/lidar/config.py`
- Test: `tests/test_lidar_config.py`

**Interfaces:**
- Consumes: nothing.
- Produces: `LidarConfig(angular_res_deg, fov_deg, max_range_m, min_range_m, range_sigma_m)` (frozen dataclass) with `@property n_beams -> int` and `bearings() -> np.ndarray` (radians, length `n_beams`, spanning `fov_deg` centred on 0 = local +x/forward). Module constant `OUSTER_OS1: LidarConfig` (datasheet-pinned preset).

- [ ] **Step 1: Write the failing test**

```python
# tests/test_lidar_config.py
import numpy as np
from wifi_radar_slam.lidar.config import LidarConfig, OUSTER_OS1


def test_bearings_span_and_count():
    cfg = LidarConfig(angular_res_deg=2.0, fov_deg=180.0, max_range_m=100.0,
                      min_range_m=0.5, range_sigma_m=0.03)
    b = cfg.bearings()
    assert cfg.n_beams == 90
    assert b.shape == (90,)
    # centred on 0, within +/- half-FOV
    assert np.isclose(b.mean(), 0.0, atol=1e-9)
    assert b.min() >= np.deg2rad(-90.0) - 1e-9
    assert b.max() <= np.deg2rad(90.0) + 1e-9


def test_ouster_preset_is_datasheet_pinned():
    assert OUSTER_OS1.max_range_m == 120.0
    assert OUSTER_OS1.range_sigma_m == 0.03
    assert OUSTER_OS1.fov_deg == 360.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_lidar_config.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'wifi_radar_slam.lidar'`

- [ ] **Step 3: Write minimal implementation**

```python
# src/wifi_radar_slam/lidar/__init__.py
"""LiDAR baseline substrate (paper 2): config, scans, ICP SLAM, runner.

Additive to the shared wifi_radar_slam pipeline; imports no Sionna/Mitsuba so it
runs in the default (fast) test suite. Sensor models A/B/C are added on later
branches against the `make_sensor` seam consumed by `runner.run_lidar`.
"""
```

```python
# src/wifi_radar_slam/lidar/config.py
from __future__ import annotations
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class LidarConfig:
    """2D horizontal-ring LiDAR model parameters (BEV comparison plane).

    A 3D automotive LiDAR is reduced to a single horizontal ring so its output is
    directly comparable to the WiFi pipeline's xy trajectories and footprint maps.
    """
    angular_res_deg: float   # bearing step between adjacent beams
    fov_deg: float           # total horizontal field of view (360 = full ring)
    max_range_m: float       # beyond this a beam returns no hit
    min_range_m: float       # closer than this is discarded (self-return / blind zone)
    range_sigma_m: float     # per-return Gaussian range noise (std)

    @property
    def n_beams(self) -> int:
        return int(round(self.fov_deg / self.angular_res_deg))

    def bearings(self) -> np.ndarray:
        """Beam bearings in radians, centred on 0 (local +x = forward)."""
        half = np.deg2rad(self.fov_deg) / 2.0
        # n_beams samples across the FOV, symmetric about 0
        step = np.deg2rad(self.fov_deg) / self.n_beams
        return (np.arange(self.n_beams) - (self.n_beams - 1) / 2.0) * step


# Preset pinned to the Ouster OS1 datasheet (automotive/mid-range spinning LiDAR):
# range up to ~120 m, range precision ~+/-3 cm, full 360-deg horizontal FOV,
# horizontal angular resolution ~0.35 deg (mid setting). Used for real runs;
# tests may construct coarser LidarConfigs directly.
OUSTER_OS1 = LidarConfig(angular_res_deg=0.35, fov_deg=360.0, max_range_m=120.0,
                         min_range_m=0.5, range_sigma_m=0.03)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_lidar_config.py -v`
Expected: PASS (2 tests)

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/lidar/__init__.py src/wifi_radar_slam/lidar/config.py tests/test_lidar_config.py
git commit -m "paper2(lidar): LidarConfig + Ouster OS1 preset (2D BEV ring)"
```

---

### Task 2: `Scan` point-cloud container

**Files:**
- Create: `src/wifi_radar_slam/lidar/pointcloud.py`
- Test: `tests/test_lidar_pointcloud.py`

**Interfaces:**
- Consumes: pose convention `(x, y, yaw)`.
- Produces: `Scan(points: np.ndarray)` where `points` is `(N, 2)` in the sensor-local frame (+x forward). Methods: `to_world(pose) -> np.ndarray` `(N,2)`; `__len__`; classmethod `from_ranges(bearings, ranges) -> Scan` (drops non-finite ranges); staticmethod `empty() -> Scan`.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_lidar_pointcloud.py
import numpy as np
from wifi_radar_slam.lidar.pointcloud import Scan


def test_to_world_rotates_then_translates():
    # a single local point 2 m straight ahead (+x local)
    scan = Scan(np.array([[2.0, 0.0]]))
    # pose at (1,1) facing +y (yaw=90deg): forward maps to +y in world
    w = scan.to_world((1.0, 1.0, np.pi / 2))
    assert np.allclose(w, [[1.0, 3.0]], atol=1e-9)


def test_from_ranges_drops_non_finite():
    bearings = np.array([0.0, np.pi / 2])
    ranges = np.array([np.inf, 4.0])          # first beam is a miss
    scan = Scan.from_ranges(bearings, ranges)
    assert len(scan) == 1
    assert np.allclose(scan.points, [[0.0, 4.0]], atol=1e-9)


def test_empty_scan():
    assert len(Scan.empty()) == 0
    assert Scan.empty().points.shape == (0, 2)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_lidar_pointcloud.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'wifi_radar_slam.lidar.pointcloud'`

- [ ] **Step 3: Write minimal implementation**

```python
# src/wifi_radar_slam/lidar/pointcloud.py
from __future__ import annotations
from dataclasses import dataclass
import numpy as np


@dataclass
class Scan:
    """A single 2D LiDAR scan: points in the sensor-local frame (+x forward)."""
    points: np.ndarray        # (N, 2)

    def __len__(self) -> int:
        return int(self.points.shape[0])

    def to_world(self, pose) -> np.ndarray:
        """Transform local points to world via pose (x, y, yaw): rotate then translate."""
        x, y = float(pose[0]), float(pose[1])
        yaw = float(pose[2]) if len(pose) > 2 else 0.0
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s], [s, c]])
        if len(self) == 0:
            return np.empty((0, 2))
        return self.points @ R.T + np.array([x, y])

    @classmethod
    def from_ranges(cls, bearings: np.ndarray, ranges: np.ndarray) -> "Scan":
        """Build a scan from per-beam bearings (rad) and ranges; drop non-finite."""
        bearings = np.asarray(bearings, dtype=float)
        ranges = np.asarray(ranges, dtype=float)
        ok = np.isfinite(ranges)
        b, r = bearings[ok], ranges[ok]
        pts = np.stack([r * np.cos(b), r * np.sin(b)], axis=1) if r.size else np.empty((0, 2))
        return cls(pts)

    @staticmethod
    def empty() -> "Scan":
        return Scan(np.empty((0, 2)))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_lidar_pointcloud.py -v`
Expected: PASS (3 tests)

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/lidar/pointcloud.py tests/test_lidar_pointcloud.py
git commit -m "paper2(lidar): Scan container (local<->world, from_ranges)"
```

---

### Task 3: `icp_align` — point-to-point 2D ICP

**Files:**
- Create: `src/wifi_radar_slam/lidar/slam_icp.py`
- Test: `tests/test_lidar_icp.py`

**Interfaces:**
- Consumes: pose convention `(x, y, yaw)`.
- Produces: `icp_align(source, target, init=(0.0,0.0,0.0), max_iter=30, tol=1e-5) -> tuple[float,float,float]` returning the pose `(x, y, yaw)` that maps `source` (N,2, sensor-local) onto `target` (M,2, world). Helper `_apply(pts, x, y, yaw) -> np.ndarray`. Both used by Task 4.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_lidar_icp.py
import numpy as np
from wifi_radar_slam.lidar.slam_icp import icp_align, _apply


def test_icp_recovers_known_transform():
    rng = np.random.default_rng(0)
    # an L-shaped point set (rotation is observable, not degenerate)
    source = np.array([[0, 0], [1, 0], [2, 0], [0, 1], [0, 2]], dtype=float)
    # small transform -> safely inside the point-to-point ICP basin from identity
    # (in SLAM, ICP is always seeded with the motion-predicted pose, i.e. close)
    gx, gy, gyaw = 0.4, -0.3, 0.15
    target = _apply(source, gx, gy, gyaw) + rng.normal(0, 1e-4, source.shape)
    x, y, yaw = icp_align(source, target, init=(0.0, 0.0, 0.0))
    assert np.isclose(x, gx, atol=1e-2)
    assert np.isclose(y, gy, atol=1e-2)
    assert np.isclose(yaw, gyaw, atol=1e-2)


def test_apply_identity():
    pts = np.array([[1.0, 2.0], [3.0, 4.0]])
    assert np.allclose(_apply(pts, 0.0, 0.0, 0.0), pts)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_lidar_icp.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'wifi_radar_slam.lidar.slam_icp'`

- [ ] **Step 3: Write minimal implementation**

```python
# src/wifi_radar_slam/lidar/slam_icp.py
from __future__ import annotations
import numpy as np


def _apply(pts: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    """Apply pose (x, y, yaw) to local points: rotate by yaw, then translate."""
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    return pts @ R.T + np.array([x, y])


def _rigid_2d(src: np.ndarray, dst: np.ndarray) -> tuple[float, float, float]:
    """Closed-form least-squares rigid transform mapping matched src -> dst (2D Kabsch)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    H = (src - mu_s).T @ (dst - mu_d)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:                 # reflect -> proper rotation
        Vt = Vt.copy()
        Vt[-1] *= -1
        R = Vt.T @ U.T
    t = mu_d - R @ mu_s
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return float(t[0]), float(t[1]), float(yaw)


def _nn_idx(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Index into b of the nearest point to each row of a (brute force)."""
    d = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)
    return np.argmin(d, axis=1)


def icp_align(source: np.ndarray, target: np.ndarray,
              init=(0.0, 0.0, 0.0), max_iter: int = 30, tol: float = 1e-5) -> tuple[float, float, float]:
    """Point-to-point ICP: pose (x,y,yaw) mapping source (local) onto target (world)."""
    x, y, yaw = float(init[0]), float(init[1]), float(init[2])
    for _ in range(max_iter):
        src_w = _apply(source, x, y, yaw)
        idx = _nn_idx(src_w, target)
        nx, ny, nyaw = _rigid_2d(source, target[idx])   # absolute source -> matched target
        if abs(nx - x) + abs(ny - y) + abs(nyaw - yaw) < tol:
            x, y, yaw = nx, ny, nyaw
            break
        x, y, yaw = nx, ny, nyaw
    return x, y, yaw
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_lidar_icp.py -v`
Expected: PASS (2 tests)

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/lidar/slam_icp.py tests/test_lidar_icp.py
git commit -m "paper2(lidar): point-to-point 2D ICP (icp_align)"
```

---

### Task 4: `run_lidar_slam` — scan-to-map SLAM

**Files:**
- Modify: `src/wifi_radar_slam/lidar/slam_icp.py` (append `run_lidar_slam`)
- Test: `tests/test_lidar_slam.py`

**Interfaces:**
- Consumes: `Scan` (Task 2), `icp_align`/`_apply` (Task 3).
- Produces: `run_lidar_slam(scans, velocity, timestep_s, rng, init_pose=None, voxel=0.5) -> tuple[np.ndarray, np.ndarray]` returning `est_traj` `(n,3)` and `est_map` `(M,2)`. `scans` is a list of `Scan`; `velocity` is `(n,2)`.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_lidar_slam.py
import numpy as np
from wifi_radar_slam.lidar.pointcloud import Scan
from wifi_radar_slam.lidar.slam_icp import run_lidar_slam


def test_recovers_straight_trajectory_from_scans():
    # world: a wall of points at x = 6, y in [-4, 4]; vehicle drives +x from origin
    rng = np.random.default_rng(0)
    wall = np.array([[6.0, y] for y in np.linspace(-4, 4, 40)])
    n, dt, speed = 12, 0.1, 3.0
    gt = np.array([[speed * dt * f, 0.0, 0.0] for f in range(n)])
    velocity = np.tile([speed, 0.0], (n, 1))
    # each scan = wall expressed in the sensor-local frame at the GT pose
    scans = []
    for f in range(n):
        rel = wall - gt[f, :2]                          # yaw=0 -> local == world-shifted
        scans.append(Scan(rel + rng.normal(0, 1e-3, rel.shape)))
    est_traj, est_map = run_lidar_slam(scans, velocity, dt, rng, init_pose=gt[0])
    ate = np.sqrt(np.mean(np.sum((est_traj[:, :2] - gt[:, :2]) ** 2, axis=1)))
    assert ate < 0.5                                    # tracks the straight drive
    assert est_map.shape[0] > 0
    # mapped points sit on the wall (x ~ 6)
    assert np.abs(est_map[:, 0].mean() - 6.0) < 0.5
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_lidar_slam.py -v`
Expected: FAIL with `ImportError: cannot import name 'run_lidar_slam'`

- [ ] **Step 3: Write minimal implementation** (append to `slam_icp.py`)

```python
def run_lidar_slam(scans, velocity, timestep_s: float, rng,
                   init_pose=None, voxel: float = 0.5):
    """Scan-to-map ICP SLAM: constant-velocity prediction corrected by ICP against
    a voxel-downsampled accumulated map. Returns (est_traj (n,3), est_map (M,2))."""
    n = len(scans)
    est = np.zeros((n, 3))
    if init_pose is not None:
        est[0, 0], est[0, 1] = float(init_pose[0]), float(init_pose[1])
        est[0, 2] = float(init_pose[2]) if len(init_pose) > 2 else 0.0

    map_cells: dict[tuple[int, int], np.ndarray] = {}

    def _accumulate(world_pts: np.ndarray) -> None:
        for p in world_pts:
            key = (int(round(p[0] / voxel)), int(round(p[1] / voxel)))
            map_cells.setdefault(key, p)                # first point wins the cell

    _accumulate(scans[0].to_world(est[0]))
    for f in range(1, n):
        vx, vy = velocity[f]
        pred = (est[f - 1, 0] + vx * timestep_s,
                est[f - 1, 1] + vy * timestep_s,
                est[f - 1, 2])
        target = np.array(list(map_cells.values()))
        src = scans[f].points
        if len(src) >= 3 and target.shape[0] >= 3:
            est[f] = icp_align(src, target, init=pred)
        else:                                           # too sparse -> dead-reckon
            est[f] = pred
        _accumulate(scans[f].to_world(est[f]))

    est_map = np.array(list(map_cells.values())) if map_cells else np.empty((0, 2))
    return est, est_map
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_lidar_slam.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/lidar/slam_icp.py tests/test_lidar_slam.py
git commit -m "paper2(lidar): scan-to-map ICP SLAM (run_lidar_slam)"
```

---

### Task 5: `ReferenceSensor` — analytic 2D ray-cast over footprint GT

**Files:**
- Create: `src/wifi_radar_slam/lidar/sensor_ref.py`
- Test: `tests/test_lidar_sensor_ref.py`

**Interfaces:**
- Consumes: `LidarConfig` (Task 1), `Scan` (Task 2), pose `(x,y,yaw)`.
- Produces: `ReferenceSensor(gt_xy, cfg, rng, gate=0.6)` with `__call__(pose) -> Scan`; factory `reference_sensor(built, cfg, rng) -> ReferenceSensor` (reads `built.ground_truth_map[:, :2]`). This is the `make_sensor` seam: signature `make_sensor(built, cfg, rng) -> Callable[[pose], Scan]`.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_lidar_sensor_ref.py
import numpy as np
from wifi_radar_slam.lidar.config import LidarConfig
from wifi_radar_slam.lidar.sensor_ref import ReferenceSensor, reference_sensor


def _cfg():
    return LidarConfig(angular_res_deg=2.0, fov_deg=360.0, max_range_m=100.0,
                       min_range_m=0.5, range_sigma_m=0.0)


def test_scan_sees_a_wall_at_correct_range():
    # dense wall of points at x = 5, y in [-2, 2]; sensor at origin facing +x
    wall = np.array([[5.0, y] for y in np.linspace(-2, 2, 41)])
    sensor = ReferenceSensor(wall, _cfg(), np.random.default_rng(0))
    scan = sensor((0.0, 0.0, 0.0))
    assert len(scan) > 0
    world = scan.to_world((0.0, 0.0, 0.0))
    # every returned point lies on the wall (x ~ 5) within the perpendicular gate
    assert np.all(np.abs(world[:, 0] - 5.0) < 0.6)


def test_factory_reads_ground_truth_map():
    class _Built:
        ground_truth_map = np.array([[5.0, 0.0, 0.0], [5.0, 1.0, 0.0]])
    s = reference_sensor(_Built(), _cfg(), np.random.default_rng(0))
    assert isinstance(s, ReferenceSensor)
    assert s(( -1.0, 0.0, 0.0)) is not None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_lidar_sensor_ref.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'wifi_radar_slam.lidar.sensor_ref'`

- [ ] **Step 3: Write minimal implementation**

```python
# src/wifi_radar_slam/lidar/sensor_ref.py
from __future__ import annotations
import numpy as np
from .pointcloud import Scan


class ReferenceSensor:
    """Analytic 2D LiDAR over a footprint point set (harness reference / test seam).

    For each beam bearing, the nearest GT point within a perpendicular `gate` of the
    beam line and within [min_range, max_range] is returned, with Gaussian range
    noise. This is the substrate's built-in sensor; models A (mesh ray-cast) and B
    (Sionna optical) replace it on later branches via the same call signature.
    """

    def __init__(self, gt_xy: np.ndarray, cfg, rng, gate: float = 0.6):
        self.gt = np.asarray(gt_xy, dtype=float)[:, :2]
        self.cfg = cfg
        self.rng = rng
        self.gate = gate

    def __call__(self, pose) -> Scan:
        px, py = float(pose[0]), float(pose[1])
        yaw = float(pose[2]) if len(pose) > 2 else 0.0
        rel = self.gt - np.array([px, py])
        rng_m = np.linalg.norm(rel, axis=1)
        ang = np.arctan2(rel[:, 1], rel[:, 0]) - yaw          # bearing in sensor frame
        in_band = (rng_m >= self.cfg.min_range_m) & (rng_m <= self.cfg.max_range_m)
        out_b, out_r = [], []
        for b in self.cfg.bearings():
            dang = np.arctan2(np.sin(ang - b), np.cos(ang - b))
            perp = rng_m * np.abs(np.sin(dang))                # dist from beam line
            cand = in_band & (np.cos(dang) > 0) & (perp < self.gate)
            if not cand.any():
                continue
            i = np.where(cand)[0][np.argmin(rng_m[cand])]      # nearest hit along beam
            out_b.append(b)
            out_r.append(rng_m[i] + self.rng.normal(0, self.cfg.range_sigma_m))
        if not out_b:
            return Scan.empty()
        return Scan.from_ranges(np.array(out_b), np.array(out_r))


def reference_sensor(built, cfg, rng) -> "ReferenceSensor":
    """make_sensor factory: build a ReferenceSensor from a BuiltScene's footprint GT."""
    return ReferenceSensor(built.ground_truth_map[:, :2], cfg, rng)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_lidar_sensor_ref.py -v`
Expected: PASS (2 tests)

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/lidar/sensor_ref.py tests/test_lidar_sensor_ref.py
git commit -m "paper2(lidar): ReferenceSensor (analytic 2D ray-cast) + make_sensor seam"
```

---

### Task 6: `run_lidar` runner — same six metrics as WiFi

**Files:**
- Create: `src/wifi_radar_slam/lidar/runner.py`
- Test: `tests/test_lidar_runner.py`

**Interfaces:**
- Consumes: `run_lidar_slam` (Task 4), a `make_sensor(built, cfg, rng)` factory (Task 5), `velocity_from_poses` and `eval.metrics`.
- Produces: `run_lidar(built, cfg, make_sensor, rng, timestep_s) -> dict` with keys exactly `{"ate","rpe","chamfer","map_accuracy","map_completeness","iou"}` (same shape as `runner.run_phase_a`).

- [ ] **Step 1: Write the failing test**

```python
# tests/test_lidar_runner.py
import numpy as np
from wifi_radar_slam.lidar.config import LidarConfig
from wifi_radar_slam.lidar.sensor_ref import reference_sensor
from wifi_radar_slam.lidar.runner import run_lidar


class _Built:
    def __init__(self):
        # box of wall points around a straight +x drive
        walls = []
        for y in (-4.0, 4.0):
            walls += [[x, y, 0.0] for x in np.linspace(0, 10, 40)]
        self.ground_truth_map = np.array(walls)
        n, dt, speed = 12, 0.1, 3.0
        self.trajectory = np.array([[speed * dt * f, 0.0, 0.0] for f in range(n)])


def test_run_lidar_emits_six_metrics_and_tracks():
    cfg = LidarConfig(angular_res_deg=2.0, fov_deg=360.0, max_range_m=100.0,
                      min_range_m=0.5, range_sigma_m=0.02)
    built = _Built()
    m = run_lidar(built, cfg, reference_sensor, np.random.default_rng(0), timestep_s=0.1)
    assert set(m) == {"ate", "rpe", "chamfer", "map_accuracy", "map_completeness", "iou"}
    assert np.isfinite(m["ate"]) and m["ate"] < 1.0
    assert np.isfinite(m["chamfer"])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_lidar_runner.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'wifi_radar_slam.lidar.runner'`

- [ ] **Step 3: Write minimal implementation**

```python
# src/wifi_radar_slam/lidar/runner.py
from __future__ import annotations
from ..geometry import velocity_from_poses
from ..eval.metrics import (ate, rpe, chamfer, occupancy_iou,
                            map_accuracy, map_completeness)
from .slam_icp import run_lidar_slam


def run_lidar(built, cfg, make_sensor, rng, timestep_s: float) -> dict:
    """Run the LiDAR baseline on a BuiltScene and return the six comparison metrics.

    `make_sensor(built, cfg, rng) -> (pose -> Scan)` is the seam that selects the
    LiDAR model (reference / A / B). Metrics match `runner.run_phase_a` exactly so
    WiFi and LiDAR rows share one comparison table.
    """
    traj = built.trajectory
    sensor = make_sensor(built, cfg, rng)
    scans = [sensor(traj[f]) for f in range(len(traj))]
    velocity = velocity_from_poses(traj, timestep_s)
    est_traj, est_map = run_lidar_slam(scans, velocity, timestep_s, rng,
                                       init_pose=traj[0])
    gt_xy = built.ground_truth_map[:, :2]
    return {
        "ate": ate(est_traj, traj),
        "rpe": rpe(est_traj, traj),
        "chamfer": chamfer(est_map, gt_xy),
        "map_accuracy": map_accuracy(est_map, gt_xy),
        "map_completeness": map_completeness(est_map, gt_xy),
        "iou": occupancy_iou(est_map, gt_xy, cell=1.0),
    }
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_lidar_runner.py -v`
Expected: PASS

- [ ] **Step 5: Run the full suite and commit**

Run: `pytest -q` (expect all pre-existing tests + the 6 new LiDAR test files to pass)

```bash
git add src/wifi_radar_slam/lidar/runner.py tests/test_lidar_runner.py
git commit -m "paper2(lidar): run_lidar runner emitting the six WiFi-comparable metrics"
```

---

## After branch 0

Merge `paper2-lidar-harness` into `paper2-wifi-vs-lidar`, record the substrate in
`papers/2-wifi-vs-lidar/DOSSIER.md`, then start branch 1 (`paper2-lidar-geo`, model A)
with its own plan against the `make_sensor(built, cfg, rng) -> (pose -> Scan)` seam and
the `run_lidar` runner defined here.
