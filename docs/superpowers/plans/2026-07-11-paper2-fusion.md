# Paper 2 WiFi+LiDAR Fusion (RQ4) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build symmetric WiFi+LiDAR fusion — a tight particle filter whose weights are the product of the WiFi bistatic likelihood and a LiDAR scan-match likelihood (map = union of both modalities' points) — plus a naive loose baseline, and answer RQ4 with a cost-normalized verdict.

**Architecture:** One new module `src/wifi_radar_slam/fusion.py` reusing the existing back-end internals (`_triangulate_bistatic`, `_reproject_bistatic`, `_cluster` from the WiFi particle filter; `_voxel_downsample` from the LiDAR side; `cKDTree` for the scan-match). An experiment script runs both scenes × LiDAR models A/B × {tight, loose} and emits the same six metrics as RQ3, then prices the fused system with `cost.py`.

**Tech Stack:** Python 3, NumPy, `scipy.spatial.cKDTree` (already used by the LiDAR ICP). Sionna only for the scene-level runs (server), never for the unit tests.

## Global Constraints

- **Branch:** all work on `paper2-fusion`, cut from `paper2-wifi-vs-lidar`; merge back on completion. Never commit to `main` or any `paper1-*` ref.
- **Same contract as the other back-ends:** fusion returns `(est_traj (n,3), est_map (M,2))` so `eval/metrics.py` scores it unchanged and results slot into the RQ3 table.
- **Scan-match target is the accumulated LiDAR map only** — NOT the union. WiFi-triangulated reflectors are noisy under realistic CSI (map-acc ~4.8 m) and would corrupt scan matching. The **union is the output map only**.
- **Don't rig the balance:** `sigma_lidar=0.5` is a fixed, documented parameter (matching the WiFi bistatic sigma already in `run_slam`). Never tune it per scene to make fusion win.
- **Graceful degradation:** a frame with no WiFi detections or an empty scan must still run — whichever modality is present drives that update.
- **Loose baseline uses equal weights**, never GT-derived weights (that would leak ground truth).
- **A negative result is a real result** — "marginal, and not worth the money" is a legitimate RQ4 answer and gets reported plainly.
- **WiFi input is the realistic (joint 2-D MUSIC, commodity-CSI) case** on both scenes.
- **Pure-Python unit tests run in the default suite** (no Sionna in `fusion.py` at import or test time).
- **ORDERING HAZARD (load-bearing):** `SionnaLidarSensor` (LiDAR model B) **mutates the
  scene** — it adds a `lidar_tx` transmitter and sets `scattering_coefficient` on every
  material. The WiFi CSI **must** be simulated *before* any model-B sensor is constructed,
  or the WiFi channel is corrupted by the diffuse-scattering override. In
  `run_fusion.py` the WiFi CSI/detections are computed **before** the LiDAR-model loop.
  Do not reorder.

---

### Task 1: `_lidar_likelihood` + `fuse_loose` (pure helpers)

**Files:**
- Create: `src/wifi_radar_slam/fusion.py`
- Test: `tests/test_fusion.py`

**Interfaces:**
- Consumes: `_voxel_downsample` (from `lidar.sensor_sionna`), `cKDTree`.
- Produces:
  `_lidar_likelihood(particles, scan_pts, tree, sigma) -> np.ndarray (n_particles,)` —
  per-particle scan-match likelihood.
  `fuse_loose(wifi_traj, wifi_map, lidar_traj, lidar_map, voxel=0.5) -> (traj (n,3), map (M,2))`.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_fusion.py
import numpy as np
from wifi_radar_slam.fusion import _lidar_likelihood, fuse_loose
from scipy.spatial import cKDTree


def test_lidar_likelihood_peaks_at_the_true_pose():
    # map: a wall of points at x=5; scan (local) sees that wall from the origin
    wall = np.array([[5.0, y] for y in np.linspace(-2, 2, 21)])
    tree = cKDTree(wall)
    scan_local = wall.copy()                      # sensor at origin, yaw 0 -> local == world
    # particle 0 is the true pose; particle 1 is displaced 3 m
    particles = np.array([[0.0, 0.0, 0.0], [3.0, 0.0, 0.0]])
    lik = _lidar_likelihood(particles, scan_local, tree, sigma=0.5)
    assert lik.shape == (2,)
    assert lik[0] > lik[1]                        # true pose is far more likely
    assert lik[0] > 0.9                           # near-perfect match


def test_fuse_loose_averages_traj_and_unions_maps():
    wifi_traj = np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    lidar_traj = np.array([[0.0, 2.0, 0.1], [4.0, 2.0, 0.1]])
    wifi_map = np.array([[10.0, 3.0]])
    lidar_map = np.array([[5.0, 0.0], [5.0, 1.0]])
    traj, m = fuse_loose(wifi_traj, wifi_map, lidar_traj, lidar_map, voxel=0.5)
    # x,y are the equal-weight average; yaw comes from the LiDAR back-end
    assert np.allclose(traj[:, :2], [[0.0, 1.0], [3.0, 1.0]])
    assert np.allclose(traj[:, 2], [0.1, 0.1])
    # map is the union of both modalities
    assert m.shape[0] == 3
    assert any(np.allclose(p, [10.0, 3.0]) for p in m)     # the WiFi reflector survived
    assert any(np.allclose(p, [5.0, 0.0]) for p in m)      # a LiDAR point survived


def test_fuse_loose_handles_empty_maps():
    traj, m = fuse_loose(np.zeros((2, 3)), np.empty((0, 2)),
                         np.zeros((2, 3)), np.empty((0, 2)))
    assert traj.shape == (2, 3)
    assert m.shape == (0, 2)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_fusion.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'wifi_radar_slam.fusion'`

- [ ] **Step 3: Write minimal implementation**

```python
# src/wifi_radar_slam/fusion.py
"""WiFi + LiDAR fusion (paper 2, RQ4).

Symmetric fusion, deliberately NOT the literature's "WiFi assists LiDAR" shape: our
RQ3 result shows realistic WiFi is the better *localizer* while LiDAR is the only
modality that *maps*, so demoting WiFi to loop closure would waste its strength.

Tight fusion: one particle filter whose weight is the product of two independent
likelihoods -- WiFi bistatic reprojection x LiDAR scan-match -- with the output map the
union of WiFi-triangulated reflectors and LiDAR points.
Loose fusion: a deliberately naive output-level baseline.
"""
from __future__ import annotations
import numpy as np
from scipy.spatial import cKDTree

from .slam.particle_filter import (_triangulate_bistatic, _reproject_bistatic, _cluster)
from .lidar.sensor_sionna import _voxel_downsample


def _lidar_likelihood(particles: np.ndarray, scan_pts: np.ndarray,
                      tree: cKDTree, sigma: float) -> np.ndarray:
    """Per-particle scan-match likelihood: place the scan at each particle's pose and
    score the mean nearest-neighbour distance to the accumulated LiDAR map.

    Fully vectorised: all (particle x point) world positions are queried in one KD-tree
    call (workers=-1), so this stays cheap for 200 particles.
    """
    p = np.asarray(particles, dtype=float)
    s = np.asarray(scan_pts, dtype=float).reshape(-1, 2)
    if s.shape[0] == 0:
        return np.ones(p.shape[0])
    cos, sin = np.cos(p[:, 2]), np.sin(p[:, 2])
    x, y = s[:, 0][None, :], s[:, 1][None, :]                  # (1, S)
    wx = cos[:, None] * x - sin[:, None] * y + p[:, 0][:, None]   # (P, S)
    wy = sin[:, None] * x + cos[:, None] * y + p[:, 1][:, None]   # (P, S)
    pts = np.stack([wx.ravel(), wy.ravel()], axis=1)           # (P*S, 2)
    d, _ = tree.query(pts, workers=-1)
    d = d.reshape(p.shape[0], s.shape[0]).mean(axis=1)          # mean NN dist per particle
    return np.exp(-0.5 * d ** 2 / sigma ** 2)


def fuse_loose(wifi_traj, wifi_map, lidar_traj, lidar_map, voxel: float = 0.5):
    """Naive output-level fusion baseline.

    Trajectory = equal-weight average of the two (x, y); yaw is taken from the LiDAR
    back-end, the only one that actually estimates it. Map = voxel-deduplicated union.
    Deliberately naive: weighting by measured accuracy would leak ground truth. Its role
    is to reveal whether tight coupling beats blind combination.
    """
    w = np.asarray(wifi_traj, dtype=float)
    l = np.asarray(lidar_traj, dtype=float)
    n = min(len(w), len(l))
    traj = np.zeros((n, 3))
    traj[:, :2] = 0.5 * (w[:n, :2] + l[:n, :2])
    traj[:, 2] = l[:n, 2]
    parts = [np.asarray(m, dtype=float).reshape(-1, 2)
             for m in (wifi_map, lidar_map) if np.asarray(m).size]
    merged = np.vstack(parts) if parts else np.empty((0, 2))
    return traj, _voxel_downsample(merged, voxel)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_fusion.py -v`
Expected: PASS (3 tests)

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/fusion.py tests/test_fusion.py
git commit -m "paper2(fusion): scan-match likelihood + naive loose-fusion baseline"
```

---

### Task 2: `run_fused_slam` — tight fusion

**Files:**
- Modify: `src/wifi_radar_slam/fusion.py` (append)
- Test: `tests/test_fusion.py` (append)

**Interfaces:**
- Consumes: `_lidar_likelihood` (Task 1); `_triangulate_bistatic`, `_reproject_bistatic`,
  `_cluster` (WiFi particle filter); `Scan.to_world` (LiDAR).
- Produces: `run_fused_slam(detections, scans, ap_positions, velocity, timestep_s, rng,
  n_particles=200, init_pose=None, map_min_support=1, map_min_excess_m=0.0,
  sigma_lidar=0.5, scan_subsample=100, voxel=0.5) -> (est_traj (n,3), est_map (M,2))`.
  `detections[f]` is `(k,3)` `[path_len, aoa, ap_index]`; `scans[f]` is a `Scan`.

- [ ] **Step 1: Write the failing tests**

```python
# append to tests/test_fusion.py
from wifi_radar_slam.fusion import run_fused_slam
from wifi_radar_slam.lidar.pointcloud import Scan


def _synthetic_case(n=12, dt=0.1, speed=3.0, with_wifi=True, with_lidar=True):
    """Straight +x drive past a box of walls, with one WiFi reflector at (10, 3)."""
    gt = np.array([[speed * dt * f, 0.0, 0.0] for f in range(n)])
    velocity = np.tile([speed, 0.0], (n, 1))
    aps = [np.array([0.0, 20.0, 6.0])]
    ap_xy = aps[0][:2]
    refl = np.array([10.0, 3.0])
    # box of wall points enclosing the drive (gives ICP/scan-match full constraint)
    xs, ys = np.linspace(-2, 12, 60), np.linspace(-4, 4, 40)
    box = np.vstack([np.column_stack([xs, np.full_like(xs, -4.0)]),
                     np.column_stack([xs, np.full_like(xs, 4.0)]),
                     np.column_stack([np.full_like(ys, -2.0), ys]),
                     np.column_stack([np.full_like(ys, 12.0), ys])])
    detections, scans = [], []
    for f in range(n):
        if with_wifi:
            d = refl - gt[f, :2]
            path = np.linalg.norm(ap_xy - refl) + np.linalg.norm(d)
            detections.append(np.array([[path, np.arctan2(d[1], d[0]), 0.0]]))
        else:
            detections.append(np.empty((0, 3)))
        scans.append(Scan(box - gt[f, :2]) if with_lidar else Scan.empty())
    return gt, velocity, aps, detections, scans, refl, box


def test_fused_slam_recovers_straight_trajectory():
    gt, vel, aps, dets, scans, _, _ = _synthetic_case()
    est, _ = run_fused_slam(dets, scans, aps, vel, 0.1, np.random.default_rng(0),
                            init_pose=gt[0])
    ate = np.sqrt(np.mean(np.sum((est[:, :2] - gt[:, :2]) ** 2, axis=1)))
    assert ate < 0.5


def test_fused_map_contains_both_modalities():
    gt, vel, aps, dets, scans, refl, box = _synthetic_case()
    _, est_map = run_fused_slam(dets, scans, aps, vel, 0.1, np.random.default_rng(0),
                                init_pose=gt[0])
    assert est_map.shape[0] > 0
    # a WiFi-triangulated reflector near (10,3) survived the union
    assert np.min(np.linalg.norm(est_map - refl, axis=1)) < 1.0
    # and LiDAR wall points survived too (e.g. the y=-4 wall)
    assert np.min(np.linalg.norm(est_map - np.array([5.0, -4.0]), axis=1)) < 1.0


def test_graceful_degradation_when_a_modality_is_missing():
    # LiDAR only (no WiFi detections at all)
    gt, vel, aps, dets, scans, _, _ = _synthetic_case(with_wifi=False)
    est, _ = run_fused_slam(dets, scans, aps, vel, 0.1, np.random.default_rng(0),
                            init_pose=gt[0])
    assert np.all(np.isfinite(est))
    # WiFi only (all scans empty)
    gt, vel, aps, dets, scans, _, _ = _synthetic_case(with_lidar=False)
    est, _ = run_fused_slam(dets, scans, aps, vel, 0.1, np.random.default_rng(0),
                            init_pose=gt[0])
    assert np.all(np.isfinite(est))
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_fusion.py -v`
Expected: FAIL with `ImportError: cannot import name 'run_fused_slam'`

- [ ] **Step 3: Write minimal implementation** (append to `fusion.py`)

```python
def run_fused_slam(detections, scans, ap_positions, velocity, timestep_s: float, rng,
                   n_particles: int = 200, init_pose=None, map_min_support: int = 1,
                   map_min_excess_m: float = 0.0, sigma_lidar: float = 0.5,
                   scan_subsample: int = 100, voxel: float = 0.5):
    """Tight WiFi+LiDAR fusion: one particle filter, two independent likelihoods.

    weight = w_wifi(bistatic reprojection) x w_lidar(scan match vs the accumulated
    LiDAR map). The scan-match target is the LiDAR map ONLY -- WiFi reflectors are too
    noisy under realistic CSI to be a registration target. The OUTPUT map is the union
    of WiFi-triangulated reflectors and LiDAR points.
    """
    n_frames = len(detections)
    particles = np.zeros((n_particles, 3))
    if init_pose is not None:
        particles[:, 0] = init_pose[0]
        particles[:, 1] = init_pose[1]
        particles[:, 2] = init_pose[2] if len(init_pose) > 2 else 0.0
    weights = np.ones(n_particles) / n_particles
    est_traj = np.zeros((n_frames, 3))
    wifi_points: list[np.ndarray] = []
    lidar_cells: dict[tuple[int, int], np.ndarray] = {}

    def _accumulate_lidar(world_pts: np.ndarray) -> None:
        for p in world_pts:
            key = (int(round(p[0] / voxel)), int(round(p[1] / voxel)))
            lidar_cells.setdefault(key, p)

    pos_noise = 0.05
    for f in range(n_frames):
        if f > 0:
            vx, vy = velocity[f]
            particles[:, 0] += vx * timestep_s + rng.normal(0, pos_noise, n_particles)
            particles[:, 1] += vy * timestep_s + rng.normal(0, pos_noise, n_particles)

        updated = False

        dets = detections[f]                       # --- WiFi bistatic likelihood ---
        if dets.shape[0] > 0:
            mean_pose = np.average(particles, axis=0, weights=weights)
            for path_len, aoa, ap_i in dets:
                ap_xy = np.asarray(ap_positions[int(ap_i)])[:2]
                refl = _triangulate_bistatic(mean_pose[:2], ap_xy, path_len, aoa,
                                             min_excess_m=map_min_excess_m)
                if refl is None:
                    continue
                wifi_points.append(refl)
                pr = np.array([_reproject_bistatic(p[:2], ap_xy, refl) for p in particles])
                err = (pr[:, 0] - path_len) ** 2 + (pr[:, 1] - aoa) ** 2
                weights *= np.exp(-0.5 * err / (0.5 ** 2))
            updated = True

        scan = scans[f]                            # --- LiDAR scan-match likelihood ---
        if len(scan) > 0 and len(lidar_cells) >= 3:
            target = np.array(list(lidar_cells.values()))
            pts = scan.points
            if scan_subsample and pts.shape[0] > scan_subsample:
                pts = pts[rng.choice(pts.shape[0], scan_subsample, replace=False)]
            weights *= _lidar_likelihood(particles, pts, cKDTree(target), sigma_lidar)
            updated = True

        if updated:
            weights += 1e-300
            weights /= weights.sum()
            neff = 1.0 / np.sum(weights ** 2)
            if neff < n_particles / 2:
                idx = rng.choice(n_particles, n_particles, p=weights)
                particles = particles[idx]
                weights = np.ones(n_particles) / n_particles

        est_traj[f] = np.average(particles, axis=0, weights=weights)
        if len(scan) > 0:
            _accumulate_lidar(scan.to_world(est_traj[f]))

    wifi_map = (_cluster(np.array(wifi_points), min_support=map_min_support)
                if wifi_points else np.empty((0, 2)))
    lidar_map = (np.array(list(lidar_cells.values())) if lidar_cells
                 else np.empty((0, 2)))
    parts = [m for m in (wifi_map, lidar_map) if m.size]
    est_map = _voxel_downsample(np.vstack(parts), voxel) if parts else np.empty((0, 2))
    return est_traj, est_map
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_fusion.py -v`
Expected: PASS (6 tests)

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/fusion.py tests/test_fusion.py
git commit -m "paper2(fusion): tight fusion particle filter (bistatic x scan-match, union map)"
```

---

### Task 3: Street realistic-WiFi config + experiment + RQ4 results

**Files:**
- Create: `configs/street_metal_music.yaml`
- Create: `experiments/run_fusion.py`
- Modify: `docs/results-paper2.md` (append a "Fusion (RQ4)" section)

**Interfaces:**
- Consumes: `run_fused_slam`, `fuse_loose` (Tasks 1–2); `build_scene`, `simulate_csi`,
  `extract_detections`; `geo_sensor` / `sionna_lidar_sensor`; `run_lidar_slam`; `run_slam`;
  `eval.metrics`; `cost.py` for the cost-normalized verdict.
- Produces: `data/fusion_results.json`.

- [ ] **Step 1: Create the street realistic-MUSIC config**

The street scene has no realistic-WiFi config (only `street_metal_oracle.yaml`). Mirror the
`controlled_music_joint.yaml` recipe (joint 2-D MUSIC, `world_aoa`, 160 MHz — WiFi's
strongest realistic setting, so we do not strawman it) onto the street scene.

```yaml
# configs/street_metal_music.yaml
run_name: street_metal_music
seed: 42
snr_db: 20.0
sensing_mode: music
world_aoa: true
joint_estimation: true     # joint 2-D delay-angle MUSIC (correct association)
map_min_support: 5
rf:
  carrier_hz: 5.2e9
  bandwidth_hz: 160.0e6    # WiFi's strongest realistic setting (do not strawman it)
  n_subcarriers: 128
  n_rx_antennas: 4
  antenna_spacing_frac: 0.5
trajectory:
  length_m: 60.0
  speed_mps: 5.0
  timestep_s: 0.05
  shape: straight
scene:
  name: street_canyon_metal
  ap_positions:
    - [-25.0, -6.0, 5.0]
    - [0.0, 6.0, 5.0]
    - [25.0, -6.0, 5.0]
  targets: []
```

- [ ] **Step 2: Write the experiment script**

```python
# experiments/run_fusion.py
"""RQ4: WiFi+LiDAR fusion on both scenes, for both LiDAR models (A/B).

For each (scene, lidar_model) emits the six metrics for:
  wifi_only, lidar_only, fused_tight, fused_loose
Run on a host with sionna-rt (amd server), throttled:
    WRS_NUM_SAMPLES=1000000 nice -n 19 ionice -c3 python experiments/run_fusion.py
"""
import json
import numpy as np

from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.channel.simulator import simulate_csi
from wifi_radar_slam.sensing.frontend import extract_detections
from wifi_radar_slam.geometry import velocity_from_poses
from wifi_radar_slam.slam.particle_filter import run_slam
from wifi_radar_slam.lidar.config import OUSTER_OS1
from wifi_radar_slam.lidar.sensor_geo import geo_sensor
from wifi_radar_slam.lidar.sensor_sionna import sionna_lidar_sensor
from wifi_radar_slam.lidar.slam_icp import run_lidar_slam
from wifi_radar_slam.fusion import run_fused_slam, fuse_loose
from wifi_radar_slam.eval.metrics import (ate, rpe, chamfer, occupancy_iou,
                                          map_accuracy, map_completeness)

SCENES = {
    "controlled_wall": "configs/controlled_music_joint.yaml",
    "street_canyon": "configs/street_metal_music.yaml",
}
LIDAR_MODELS = {"A_geometric": geo_sensor, "B_sionna": sionna_lidar_sensor}


def _metrics(est_traj, est_map, gt_traj, gt_xy):
    return {
        "ate": ate(est_traj, gt_traj), "rpe": rpe(est_traj, gt_traj),
        "chamfer": chamfer(est_map, gt_xy),
        "map_accuracy": map_accuracy(est_map, gt_xy),
        "map_completeness": map_completeness(est_map, gt_xy),
        "iou": occupancy_iou(est_map, gt_xy, cell=1.0),
    }


def main() -> None:
    results = {}
    for scene, cfgpath in SCENES.items():
        cfg = load_config(cfgpath)
        built = build_scene(cfg)
        gt, gt_xy = built.trajectory, built.ground_truth_map[:, :2]
        vel = velocity_from_poses(gt, cfg.trajectory.timestep_s)

        # --- realistic WiFi detections (commodity CSI -> joint 2-D MUSIC) ---
        rng = np.random.default_rng(cfg.seed)
        csi = simulate_csi(built, cfg.rf, cfg.snr_db, rng)
        dets = extract_detections(csi, cfg.rf, n_paths=3, world_aoa=cfg.world_aoa,
                                  joint=cfg.joint_estimation)
        w_traj, w_map = run_slam(dets, built.ap_positions, vel,
                                 cfg.trajectory.timestep_s, np.random.default_rng(0),
                                 init_pose=gt[0], map_min_support=cfg.map_min_support,
                                 map_min_excess_m=cfg.map_min_excess_m)
        results.setdefault(scene, {})["wifi_only"] = _metrics(w_traj, w_map, gt, gt_xy)

        for mname, make_sensor in LIDAR_MODELS.items():
            sensor = make_sensor(built, OUSTER_OS1, np.random.default_rng(0))
            scans = [sensor(gt[f]) for f in range(len(gt))]

            l_traj, l_map = run_lidar_slam(scans, vel, cfg.trajectory.timestep_s,
                                           np.random.default_rng(0), init_pose=gt[0])
            f_traj, f_map = run_fused_slam(dets, scans, built.ap_positions, vel,
                                           cfg.trajectory.timestep_s,
                                           np.random.default_rng(0), init_pose=gt[0],
                                           map_min_support=cfg.map_min_support,
                                           map_min_excess_m=cfg.map_min_excess_m)
            lo_traj, lo_map = fuse_loose(w_traj, w_map, l_traj, l_map)

            results[scene][f"lidar_only_{mname}"] = _metrics(l_traj, l_map, gt, gt_xy)
            results[scene][f"fused_tight_{mname}"] = _metrics(f_traj, f_map, gt, gt_xy)
            results[scene][f"fused_loose_{mname}"] = _metrics(lo_traj, lo_map, gt, gt_xy)
            print(f"[{scene}/{mname}] tight={results[scene][f'fused_tight_{mname}']}")

    with open("data/fusion_results.json", "w") as f:
        json.dump(results, f, indent=2)
    print("saved -> data/fusion_results.json")


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: Parse-check locally and run the full local suite**

Run: `python -m py_compile experiments/run_fusion.py && echo OK`
Run: `pytest -q` → all pass (6 new fusion tests included).

- [ ] **Step 4: Run on the amd server, throttled**

Sync the branch to the server, then:
`WRS_NUM_SAMPLES=1000000 nice -n 19 ionice -c3 python experiments/run_fusion.py`
Expected: prints a tight-fusion metrics line per (scene, LiDAR model) and writes
`data/fusion_results.json`. Model B does a Sionna path-solve per pose, so the street/B
combination is the slow one (minutes); keep it throttled and watch server load.

- [ ] **Step 5: Write the RQ4 results section + cost-normalized verdict**

Append a `## Fusion (RQ4)` section to `docs/results-paper2.md` containing, per scene and
LiDAR model, the four rows (wifi_only, lidar_only, fused_tight, fused_loose) across the six
metrics, plus the *oracle best-of-each* reference (WiFi's ATE row + LiDAR's map rows, taken
from the existing RQ3 table — no new run needed).

Then compute the **cost-normalized verdict** with `cost.py`: price the fused system at
`wifi_package + ouster_os1` and report `$·ATE` and `$/IoU` for fused vs LiDAR-only. State
the RQ4 answer explicitly as one of **significant / marginal / none**, and separately
whether it is **worth paying for both** (the fused price is dominated by the LiDAR, so
fusion must beat LiDAR-only substantially to justify itself).

```bash
git add configs/street_metal_music.yaml experiments/run_fusion.py \
        data/fusion_results.json docs/results-paper2.md
git commit -m "paper2(fusion): RQ4 results — tight vs loose fusion vs solo, cost-normalized"
```

---

## After this plan

Merge `paper2-fusion` into `paper2-wifi-vs-lidar`, update the DOSSIER with the RQ4 verdict,
then start the final sub-project: **RQ2 — deep-learning enhancement** (can DL close the WiFi
mapping-coverage gap?), which gets its own brainstorm → spec → plan cycle.
