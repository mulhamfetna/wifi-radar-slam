# Paper 2 LiDAR Model B — Sionna optical-ray proxy (branch 2) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement LiDAR model B — a Sionna ray-traced "optical" LiDAR that models the sensor as a monostatic node (transmitter co-located with the vehicle) whose materials diffusely backscatter, reads the single-scatter interaction vertices as returns, and plugs into the branch-0 `make_sensor` seam to produce the six WiFi-comparable metrics on both scenes.

**Architecture:** One new module `src/wifi_radar_slam/lidar/sensor_sionna.py`. Pure, locally-testable helpers (`_voxel_downsample`, `vertices_to_scan`) convert world hit points to a sensor-local `Scan` (range filter, radial noise, world→local, density cap). A Sionna-gated `SionnaLidarSensor` runs `PathSolver` per pose with `diffuse_reflection=True` and material `scattering_coefficient>0`, selects single-scatter valid non-floor paths on the LiDAR-TX row, and feeds their interaction vertices to `vertices_to_scan`. An experiment script runs it on both scenes on the amd server.

**Tech Stack:** Python 3, NumPy. Sionna RT 2.0.1 `PathSolver` (server only). Reuses branch-0 `lidar.pointcloud.Scan`, `lidar.runner.run_lidar`, `lidar.config.OUSTER_OS1`, and `geometry.RX_HEIGHT_M`. Mirrors the per-frame solve pattern in `sensing/oracle.py`.

## Global Constraints

- **Branch:** all work on `paper2-lidar-sionna`, cut from `paper2-wifi-vs-lidar`; merge back on completion. Never commit to `main`/`paper1-*`.
- **NumPy only** for library logic; `import sionna.rt`/`import mitsuba` are **lazy, inside methods** (as in `scene/builder.py` and `sensing/oracle.py`) so `sensor_sionna.py` imports without Sionna and its pure helpers test locally.
- **2D BEV comparison plane** — returns reduce to `xy`; `run_lidar` reuses `eval/metrics.py` unchanged.
- **Validated physics (server spike, 2026-07-10):** monostatic + specular-only ⇒ ~0 returns; monostatic + `diffuse_reflection=True` + `scattering_coefficient=0.7` ⇒ thousands of single-scatter non-floor returns (controlled scene: 8417 hits, ranges 8–37 m). This is the model's basis.
- **Path array indexing (Sionna RT 2.0.1):** `interactions.numpy()` is `(depth, n_rx, n_tx, n_paths)` → take `[:,0]` (rx0) then `[:,tx]`; `tau/valid/objects.numpy()[0]` is `(n_tx,n_paths)` → `[tx]`; `vertices.numpy()` is `(depth, n_rx, n_tx, n_paths, 3)` → `[:,0,tx]` → `(depth,n_paths,3)`. Monostatic round-trip: `range = tau*c/2`, but we read the **interaction vertex** directly (no range/bearing trig).
- **Single-scatter selection:** exactly one non-NONE interaction across depth (`np.count_nonzero(inter,axis=0)==1`) AND `valid`, excluding floor `object_id`s — identical criterion to `sensing/oracle.py`.
- **RNG passed in**; **pose** `(x,y,yaw)`; **`make_sensor` seam:** `sionna_lidar_sensor(built, cfg, rng) -> (pose -> Scan)`.
- **Config semantics:** model B ignores `LidarConfig` beam params (angular_res/fov/n_beams — it is ray-traced, not swept) but **honors `min_range_m`/`max_range_m`/`range_sigma_m`**; document this. Scans are voxel-downsampled (default `scan_voxel=0.2 m`) to cap ICP cost at diffuse-return densities.
- **Server workflow:** actual runs on amd (`/home/dev/mulham/wifi-radar-slam`, `.venv` with `sionna-rt`), throttled `nice -n 19 ionice -c3`; sample count via `WRS_NUM_SAMPLES`.

---

### Task 1: pure helpers — `_voxel_downsample` + `vertices_to_scan`

**Files:**
- Create: `src/wifi_radar_slam/lidar/sensor_sionna.py`
- Test: `tests/test_lidar_sensor_sionna.py`

**Interfaces:**
- Consumes: `Scan` (branch 0), pose `(x,y,yaw)`, `LidarConfig` (min/max range, range_sigma).
- Produces: `_voxel_downsample(pts, voxel) -> np.ndarray`; `vertices_to_scan(world_hits, pose, cfg, rng, scan_voxel=0.2) -> Scan` — range-filter, radial noise, world→local, downsample. Used by Task 2.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_lidar_sensor_sionna.py
import numpy as np
from wifi_radar_slam.lidar.config import LidarConfig
from wifi_radar_slam.lidar.sensor_sionna import _voxel_downsample, vertices_to_scan


def _cfg(sigma=0.0):
    return LidarConfig(angular_res_deg=2.0, fov_deg=360.0, max_range_m=50.0,
                       min_range_m=0.5, range_sigma_m=sigma)


def test_voxel_downsample_collapses_duplicates():
    pts = np.array([[0.01, 0.0], [0.02, 0.01], [5.0, 5.0]])   # first two share a 0.2 cell
    out = _voxel_downsample(pts, 0.2)
    assert out.shape[0] == 2


def test_vertices_to_scan_world_to_local_and_range_filter():
    # hits at 5 m ahead and 80 m away; sensor at origin facing +x
    hits = np.array([[5.0, 0.0], [80.0, 0.0]])
    scan = vertices_to_scan(hits, (0.0, 0.0, 0.0), _cfg(), np.random.default_rng(0))
    # far hit dropped by max_range; near hit kept, local == world here
    assert len(scan) == 1
    assert np.allclose(scan.points, [[5.0, 0.0]], atol=1e-9)


def test_vertices_to_scan_respects_yaw():
    # hit 5 m north in world; sensor facing +y (yaw=90deg) -> straight ahead locally
    hits = np.array([[0.0, 5.0]])
    scan = vertices_to_scan(hits, (0.0, 0.0, np.pi / 2), _cfg(), np.random.default_rng(0))
    assert np.allclose(scan.points, [[5.0, 0.0]], atol=1e-9)


def test_vertices_to_scan_empty_is_empty():
    assert len(vertices_to_scan(np.empty((0, 2)), (0, 0, 0), _cfg(),
                                np.random.default_rng(0))) == 0
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_lidar_sensor_sionna.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'wifi_radar_slam.lidar.sensor_sionna'`

- [ ] **Step 3: Write minimal implementation**

```python
# src/wifi_radar_slam/lidar/sensor_sionna.py
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_lidar_sensor_sionna.py -v`
Expected: PASS (4 tests)

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/lidar/sensor_sionna.py tests/test_lidar_sensor_sionna.py
git commit -m "paper2(lidar/B): pure helpers - vertices_to_scan + voxel downsample"
```

---

### Task 2: `SionnaLidarSensor` + `sionna_lidar_sensor` factory (Sionna-gated)

**Files:**
- Modify: `src/wifi_radar_slam/lidar/sensor_sionna.py` (append the sensor + factory)
- Test: `tests/test_lidar_sionna_scene.py` (new, Sionna-gated)

**Interfaces:**
- Consumes: `vertices_to_scan` (Task 1), a `BuiltScene`, `RX_HEIGHT_M`, Sionna `PathSolver`.
- Produces: `SionnaLidarSensor(built, cfg, rng, scattering=0.7, max_depth=2, scan_voxel=0.2)` with `__call__(pose) -> Scan`; `sionna_lidar_sensor(built, cfg, rng) -> SionnaLidarSensor` — the `make_sensor` factory `run_lidar` consumes.

- [ ] **Step 1: Write the failing test** (Sionna-gated; runs on the server)

```python
# tests/test_lidar_sionna_scene.py
import numpy as np
import pytest
pytest.importorskip("sionna")
from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.lidar.config import OUSTER_OS1
from wifi_radar_slam.lidar.sensor_sionna import sionna_lidar_sensor, SionnaLidarSensor
from wifi_radar_slam.lidar.runner import run_lidar


def test_model_b_returns_points_and_runs(monkeypatch):
    monkeypatch.setenv("WRS_NUM_SAMPLES", "100000")     # keep the ray-trace test fast
    cfg = load_config("configs/controlled_oracle.yaml")
    built = build_scene(cfg)
    built.trajectory = built.trajectory[:3]
    # sensor yields a non-empty scan (diffuse backscatter works)
    sensor = SionnaLidarSensor(built, OUSTER_OS1, np.random.default_rng(0))
    assert len(sensor(built.trajectory[0])) > 0
    # end-to-end metrics via the shared runner
    m = run_lidar(built, OUSTER_OS1, sionna_lidar_sensor, np.random.default_rng(0),
                  cfg.trajectory.timestep_s)
    assert set(m) == {"ate", "rpe", "chamfer", "map_accuracy", "map_completeness", "iou"}
    assert np.isfinite(m["ate"])
```

- [ ] **Step 2: Run to verify it fails/skips appropriately**

Run (local): `pytest tests/test_lidar_sionna_scene.py -v` → SKIPPED (no Sionna) — expected.
The real fail-then-pass happens on the server in Task 3; locally we cannot import the sensor's runtime, only confirm the module imports (Task 1 tests already exercise that).

- [ ] **Step 3: Write minimal implementation** (append to `sensor_sionna.py`)

```python
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
```

- [ ] **Step 4: Verify local suite still green + module imports without Sionna**

Run: `python -c "import wifi_radar_slam.lidar.sensor_sionna as m; print(hasattr(m,'SionnaLidarSensor'))"` → `True`
Run: `pytest tests/test_lidar_sensor_sionna.py -q` → PASS; `pytest tests/test_lidar_sionna_scene.py -q` → skipped.

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/lidar/sensor_sionna.py tests/test_lidar_sionna_scene.py
git commit -m "paper2(lidar/B): SionnaLidarSensor (monostatic diffuse) + make_sensor factory"
```

---

### Task 3: Experiment script + results on both scenes (server)

**Files:**
- Create: `experiments/run_lidar_sionna.py`
- Test: none new (covered by Tasks 1–2 + the gated scene test).

**Interfaces:**
- Consumes: `load_config`, `build_scene`, `OUSTER_OS1`, `sionna_lidar_sensor`, `run_lidar`.
- Produces: a CLI printing/saving model-B metrics for both scenes.

- [ ] **Step 1: Write the experiment script**

```python
# experiments/run_lidar_sionna.py
"""Model B (Sionna optical-ray proxy) LiDAR baseline on the simulated scenes.

Run on a host with sionna-rt installed (amd server), throttled:
    WRS_NUM_SAMPLES=1000000 nice -n 19 ionice -c3 python experiments/run_lidar_sionna.py
Monostatic node + diffuse backscatter; single-scatter interaction vertices are the
returns. Emits the six WiFi-comparable metrics (same shape as the WiFi runner).
"""
import json
import numpy as np
from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.lidar.config import OUSTER_OS1
from wifi_radar_slam.lidar.sensor_sionna import sionna_lidar_sensor
from wifi_radar_slam.lidar.runner import run_lidar

SCENES = {
    "controlled_wall": "configs/controlled_oracle.yaml",
    "street_canyon": "configs/street_metal_oracle.yaml",
}


def main() -> None:
    rng = np.random.default_rng(0)
    results = {}
    for label, cfgpath in SCENES.items():
        cfg = load_config(cfgpath)
        built = build_scene(cfg)
        m = run_lidar(built, OUSTER_OS1, sionna_lidar_sensor, rng,
                      cfg.trajectory.timestep_s)
        results[label] = m
        print(f"[model B] {label}: " + json.dumps(m))
    with open("data/lidar_sionna_results.json", "w") as f:
        json.dump(results, f, indent=2)
    print("saved -> data/lidar_sionna_results.json")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Parse-check locally (no Sionna needed)**

Run: `python -m py_compile experiments/run_lidar_sionna.py && echo OK`
Expected: `OK`.

- [ ] **Step 3: Run on the amd server, throttled**

Sync branch to server, then:
`WRS_NUM_SAMPLES=1000000 nice -n 19 ionice -c3 python experiments/run_lidar_sionna.py`
Expected: a metrics line per scene + `data/lidar_sionna_results.json`. Note: the diffuse
solve is heavier than model A (thousands of returns/frame); if a full trajectory is slow,
lower `WRS_NUM_SAMPLES` (physics validated down to 1e5) or `max_depth`. Watch server load
and keep it throttled.

- [ ] **Step 4: Record results and commit**

Add a "Model B (Sionna optical-ray proxy)" row to the paper-2 comparison table in
`papers/2-wifi-vs-lidar/DOSSIER.md`, beside model A. Note the key contrast: model B is
a *physics* LiDAR (diffuse EM backscatter) vs model A's *geometric* ray-cast — comparing
their metrics tells us how much the geometric abstraction misses.

```bash
git add experiments/run_lidar_sionna.py data/lidar_sionna_results.json \
        papers/2-wifi-vs-lidar/DOSSIER.md
git commit -m "paper2(lidar/B): model-B baseline results on both scenes"
```

- [ ] **Step 5: Full suite green**

Run: `pytest -q`
Expected: all pass; `test_lidar_sionna_scene.py` passes on the server / skips locally.

---

## After branch 2

Merge `paper2-lidar-sionna` into `paper2-wifi-vs-lidar`, update the DOSSIER (A-vs-B
comparison note), then branch 3 (`paper2-lidar-kitti`, model C — KITTI external
validity). After C, assemble the full WiFi-vs-LiDAR comparison table.
