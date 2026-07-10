# Paper 2 LiDAR Model A — geometric bbox-segment ray-cast (branch 1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement LiDAR model A — a geometric 2D LiDAR that ray-casts each scene object's bounding-box wall segments at the scan plane — and plug it into the branch-0 `make_sensor` seam so it produces the six WiFi-comparable metrics on both simulated scenes.

**Architecture:** Two new modules in the existing `src/wifi_radar_slam/lidar/` package. `mesh_slice.py` turns scene objects into 2D wall segments via `obj.mi_mesh.bbox()` (the API `_footprint_ground_truth` already uses). `sensor_geo.py` ray-casts those segments in pure NumPy (ray-vs-segment, nearest hit = correct occlusion) and exposes a `geo_sensor(built, cfg, rng)` factory matching the seam. An experiment script runs it on both scenes on a Sionna-equipped host. All ray math is unit-tested locally; only the thin scene→segments loop is Sionna-gated.

**Tech Stack:** Python 3, NumPy only. Reuses branch-0 `lidar.config.LidarConfig`/`OUSTER_OS1`, `lidar.pointcloud.Scan`, `lidar.runner.run_lidar`, and `geometry.RX_HEIGHT_M`.

## Global Constraints

- **Branch:** all work on `paper2-lidar-geo`, cut from `paper2-wifi-vs-lidar`; merge back on completion. Never commit to `main` or any `paper1-*` ref.
- **NumPy only** — no SciPy/sklearn/new deps. No `import sionna`/`import mitsuba` anywhere in `lidar/` — model A reaches geometry only through `built.scene.objects[...].mi_mesh.bbox()` at call time, so the modules import fine without Sionna and only *running* `scene_segments` on a real scene needs it.
- **2D BEV comparison plane** — segments and scans are `xy`; the scan plane is the horizontal slice at `z = RX_HEIGHT_M` (1.5 m). LiDAR reuses `eval/metrics.py` unchanged via `run_lidar`.
- **Pure-Python tests run in the default suite**; the one scene-level test that needs a real Sionna scene is gated with `pytest.importorskip("sionna")` (matches `tests/test_scene_smoke.py`).
- **RNG passed in** (`np.random.default_rng`), never created in library code.
- **Pose convention** `(x, y, yaw)`; a ray for beam bearing `b` points along world angle `yaw + b`. Ranges are along the unit ray so `range == t`.
- **`make_sensor` seam:** `geo_sensor(built, cfg, rng) -> (pose -> Scan)`, same signature branch-0's `reference_sensor` satisfies and `run_lidar` consumes.
- **Fidelity note (documented limitation):** model A ray-casts bounding-box footprints, faithful for box-like buildings and approximate for cars; a triangle-exact Mitsuba `ray_intersect` version is a deferred follow-up branch, not this one.

---

### Task 1: `_bbox_to_segments` + `scene_segments` (mesh → 2D wall segments)

**Files:**
- Create: `src/wifi_radar_slam/lidar/mesh_slice.py`
- Test: `tests/test_lidar_mesh_slice.py`

**Interfaces:**
- Consumes: a `BuiltScene` (`built.scene.objects`, each with `.mi_mesh.bbox()` → `.min`/`.max`).
- Produces: `_bbox_to_segments(bbmin, bbmax, z_height) -> np.ndarray` shape `(S,2,2)` (0 or 4 segments), pure. `scene_segments(built, z_height) -> np.ndarray` `(S,2,2)` — all non-floor objects' wall segments at the scan plane. Used by Task 3.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_lidar_mesh_slice.py
import numpy as np
from wifi_radar_slam.lidar.mesh_slice import _bbox_to_segments


def test_box_cut_by_scan_plane_yields_four_edges():
    segs = _bbox_to_segments(np.array([0.0, 0.0, -1.0]),
                             np.array([2.0, 4.0, 3.0]), z_height=1.0)
    assert segs.shape == (4, 2, 2)
    # the four rectangle corners are all present across the segment endpoints
    corners = {tuple(p) for s in segs for p in s}
    assert corners == {(0.0, 0.0), (2.0, 0.0), (2.0, 4.0), (0.0, 4.0)}


def test_box_not_spanning_scan_height_is_skipped():
    segs = _bbox_to_segments(np.array([0.0, 0.0, -1.0]),
                             np.array([2.0, 4.0, 0.5]), z_height=1.0)   # top at 0.5 < 1.0
    assert segs.shape == (0, 2, 2)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_lidar_mesh_slice.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'wifi_radar_slam.lidar.mesh_slice'`

- [ ] **Step 3: Write minimal implementation**

```python
# src/wifi_radar_slam/lidar/mesh_slice.py
from __future__ import annotations
import numpy as np


def _bbox_to_segments(bbmin, bbmax, z_height: float) -> np.ndarray:
    """The 4 xy rectangle edges of an axis-aligned bbox, if the horizontal scan
    plane z=z_height cuts it; else an empty (0,2,2) array. Pure geometry."""
    bbmin = np.asarray(bbmin, dtype=float).ravel()
    bbmax = np.asarray(bbmax, dtype=float).ravel()
    if z_height < bbmin[2] or z_height > bbmax[2]:
        return np.empty((0, 2, 2))
    x0, y0, x1, y1 = bbmin[0], bbmin[1], bbmax[0], bbmax[1]
    c = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
    return np.array([[c[0], c[1]], [c[1], c[2]], [c[2], c[3]], [c[3], c[0]]])


def scene_segments(built, z_height: float) -> np.ndarray:
    """2D wall segments (S,2,2) of every non-floor scene object at the scan plane.

    Reads object bounding boxes via `obj.mi_mesh.bbox()` (same API as the pipeline's
    footprint ground truth), so it requires a real Sionna-built scene at call time.
    """
    segs = []
    for name, obj in built.scene.objects.items():
        if "floor" in name.lower():
            continue
        bb = obj.mi_mesh.bbox()
        s = _bbox_to_segments(np.array(bb.min).ravel(), np.array(bb.max).ravel(), z_height)
        if s.shape[0]:
            segs.append(s)
    return np.vstack(segs) if segs else np.empty((0, 2, 2))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_lidar_mesh_slice.py -v`
Expected: PASS (2 tests)

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/lidar/mesh_slice.py tests/test_lidar_mesh_slice.py
git commit -m "paper2(lidar/A): bbox -> 2D wall segments at the scan plane"
```

---

### Task 2: `_ray_segments_scan` — pure-NumPy ray-vs-segment ray-cast

**Files:**
- Create: `src/wifi_radar_slam/lidar/sensor_geo.py`
- Test: `tests/test_lidar_sensor_geo.py`

**Interfaces:**
- Consumes: `LidarConfig` (bearings/ranges), `Scan.from_ranges`/`Scan.empty`, pose `(x,y,yaw)`.
- Produces: `_ray_segments_scan(segments, pose, cfg, rng) -> Scan` — for each beam, the nearest segment intersection within `[min_range, max_range]`, with Gaussian range noise. Used by Task 3.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_lidar_sensor_geo.py
import numpy as np
from wifi_radar_slam.lidar.config import LidarConfig
from wifi_radar_slam.lidar.sensor_geo import _ray_segments_scan


def _cfg():
    return LidarConfig(angular_res_deg=2.0, fov_deg=360.0, max_range_m=100.0,
                       min_range_m=0.5, range_sigma_m=0.0)


def test_beam_hits_wall_at_correct_range():
    # vertical wall segment at x=5, y in [-2,2]; sensor at origin facing +x
    wall = np.array([[[5.0, -2.0], [5.0, 2.0]]])
    scan = _ray_segments_scan(wall, (0.0, 0.0, 0.0), _cfg(), np.random.default_rng(0))
    world = scan.to_world((0.0, 0.0, 0.0))
    assert len(scan) > 0
    # the forward (bearing~0) beam return sits on the wall at x=5
    fwd = world[np.argmin(np.abs(world[:, 1]))]
    assert np.isclose(fwd[0], 5.0, atol=1e-6)


def test_nearest_hit_wins_occlusion():
    # two parallel walls at x=5 and x=8; the near one occludes the far one
    walls = np.array([[[5.0, -2.0], [5.0, 2.0]], [[8.0, -2.0], [8.0, 2.0]]])
    scan = _ray_segments_scan(walls, (0.0, 0.0, 0.0), _cfg(), np.random.default_rng(0))
    world = scan.to_world((0.0, 0.0, 0.0))
    fwd = world[np.argmin(np.abs(world[:, 1]))]
    assert np.isclose(fwd[0], 5.0, atol=1e-6)          # never 8


def test_no_segments_returns_empty():
    scan = _ray_segments_scan(np.empty((0, 2, 2)), (0.0, 0.0, 0.0), _cfg(),
                              np.random.default_rng(0))
    assert len(scan) == 0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_lidar_sensor_geo.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'wifi_radar_slam.lidar.sensor_geo'`

- [ ] **Step 3: Write minimal implementation**

```python
# src/wifi_radar_slam/lidar/sensor_geo.py
from __future__ import annotations
import numpy as np
from ..geometry import RX_HEIGHT_M
from .pointcloud import Scan


def _ray_segments_scan(segments, pose, cfg, rng) -> Scan:
    """Ray-cast a 2D LiDAR at `pose` against wall segments (S,2,2).

    For each beam bearing, solve o + t*d = a + u*(b-a) for every segment; a valid
    hit needs u in [0,1] and t in [min_range, max_range]; the nearest such t is the
    return (correct occlusion). Ranges get Gaussian noise; misses drop out.
    """
    segments = np.asarray(segments, dtype=float).reshape(-1, 2, 2)
    px, py = float(pose[0]), float(pose[1])
    yaw = float(pose[2]) if len(pose) > 2 else 0.0
    o = np.array([px, py])
    if segments.shape[0] == 0:
        return Scan.empty()
    a = segments[:, 0, :]
    e = segments[:, 1, :] - a                       # segment direction vectors (S,2)
    ao = a - o                                      # (S,2)

    out_b, out_r = [], []
    for beam in cfg.bearings():
        ang = yaw + beam
        d = np.array([np.cos(ang), np.sin(ang)])
        denom = d[0] * e[:, 1] - d[1] * e[:, 0]     # cross(d, e)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (ao[:, 0] * e[:, 1] - ao[:, 1] * e[:, 0]) / denom   # cross(ao,e)/denom
            u = (ao[:, 0] * d[1] - ao[:, 1] * d[0]) / denom         # cross(ao,d)/denom
        hit = (np.abs(denom) > 1e-12) & (u >= 0.0) & (u <= 1.0) \
            & (t >= cfg.min_range_m) & (t <= cfg.max_range_m)
        if not hit.any():
            continue
        r = float(t[hit].min()) + rng.normal(0, cfg.range_sigma_m)
        out_b.append(beam)
        out_r.append(r)
    if not out_b:
        return Scan.empty()
    return Scan.from_ranges(np.array(out_b), np.array(out_r))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_lidar_sensor_geo.py -v`
Expected: PASS (3 tests)

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/lidar/sensor_geo.py tests/test_lidar_sensor_geo.py
git commit -m "paper2(lidar/A): pure-NumPy ray-vs-segment ray-cast (occlusion-correct)"
```

---

### Task 3: `GeoSensor` + `geo_sensor` factory (the make_sensor seam)

**Files:**
- Modify: `src/wifi_radar_slam/lidar/sensor_geo.py` (append `GeoSensor`, `geo_sensor`)
- Test: `tests/test_lidar_sensor_geo.py` (append a local `GeoSensor` test); `tests/test_lidar_geo_scene.py` (new, Sionna-gated)

**Interfaces:**
- Consumes: `_ray_segments_scan` (Task 2), `scene_segments` (Task 1), `RX_HEIGHT_M`.
- Produces: `GeoSensor(segments, cfg, rng)` with `__call__(pose) -> Scan`; `geo_sensor(built, cfg, rng, z_height=RX_HEIGHT_M) -> GeoSensor` — the `make_sensor` factory `run_lidar` consumes.

- [ ] **Step 1: Write the failing tests**

```python
# append to tests/test_lidar_sensor_geo.py
from wifi_radar_slam.lidar.sensor_geo import GeoSensor


def test_geosensor_calls_are_scans():
    wall = np.array([[[5.0, -2.0], [5.0, 2.0]]])
    sensor = GeoSensor(wall, _cfg(), np.random.default_rng(0))
    scan = sensor((0.0, 0.0, 0.0))
    assert len(scan) > 0
    assert GeoSensor(np.empty((0, 2, 2)), _cfg(), np.random.default_rng(0))((0, 0, 0)) is not None
```

```python
# tests/test_lidar_geo_scene.py
import numpy as np
import pytest
pytest.importorskip("sionna")
from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.lidar.config import OUSTER_OS1
from wifi_radar_slam.lidar.sensor_geo import geo_sensor
from wifi_radar_slam.lidar.runner import run_lidar


def test_model_a_runs_on_controlled_scene():
    cfg = load_config("configs/controlled_oracle.yaml")
    built = build_scene(cfg)
    built.trajectory = built.trajectory[:3]            # 3 frames -> fast smoke
    m = run_lidar(built, OUSTER_OS1, geo_sensor, np.random.default_rng(0),
                  cfg.trajectory.timestep_s)
    assert set(m) == {"ate", "rpe", "chamfer", "map_accuracy", "map_completeness", "iou"}
    assert np.isfinite(m["ate"])
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_lidar_sensor_geo.py::test_geosensor_calls_are_scans -v`
Expected: FAIL with `ImportError: cannot import name 'GeoSensor'`
(The gated `test_lidar_geo_scene.py` will *skip* locally without Sionna — that is expected; it runs on the server.)

- [ ] **Step 3: Write minimal implementation** (append to `sensor_geo.py`)

```python
class GeoSensor:
    """Model A: geometric bbox-segment LiDAR. Static segments ray-cast per pose."""

    def __init__(self, segments, cfg, rng):
        self.segments = np.asarray(segments, dtype=float).reshape(-1, 2, 2)
        self.cfg = cfg
        self.rng = rng

    def __call__(self, pose) -> Scan:
        if self.segments.shape[0] == 0:
            return Scan.empty()
        return _ray_segments_scan(self.segments, pose, self.cfg, self.rng)


def geo_sensor(built, cfg, rng, z_height: float = RX_HEIGHT_M) -> "GeoSensor":
    """make_sensor factory: slice the scene into wall segments, return a GeoSensor."""
    from .mesh_slice import scene_segments
    return GeoSensor(scene_segments(built, z_height), cfg, rng)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_lidar_sensor_geo.py -v` (local, all pass)
Expected: PASS (4 tests). `pytest tests/test_lidar_geo_scene.py -v` → SKIPPED locally (no Sionna).

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/lidar/sensor_geo.py tests/test_lidar_sensor_geo.py tests/test_lidar_geo_scene.py
git commit -m "paper2(lidar/A): GeoSensor + geo_sensor factory (make_sensor seam)"
```

---

### Task 4: Experiment script + results on both scenes

**Files:**
- Create: `experiments/run_lidar_geo.py`
- Test: none new (covered by Tasks 1–3 + the gated scene test); this task produces results, not code paths.

**Interfaces:**
- Consumes: `load_config`, `build_scene`, `OUSTER_OS1`, `geo_sensor`, `run_lidar`.
- Produces: a CLI that prints and saves model-A metrics for both scenes. Run on a Sionna-equipped host.

- [ ] **Step 1: Write the experiment script**

```python
# experiments/run_lidar_geo.py
"""Model A (geometric bbox-segment LiDAR) baseline on the simulated scenes.

Run on a host with sionna-rt installed (e.g. the amd server), throttled:
    nice -n 19 ionice -c3 python experiments/run_lidar_geo.py
Builds each scene, ray-casts the bbox-segment LiDAR, runs ICP SLAM, and reports
the six WiFi-comparable metrics (same metric shape as the WiFi runner).
"""
import json
import numpy as np
from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.lidar.config import OUSTER_OS1
from wifi_radar_slam.lidar.sensor_geo import geo_sensor
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
        m = run_lidar(built, OUSTER_OS1, geo_sensor, rng, cfg.trajectory.timestep_s)
        results[label] = m
        print(f"[model A] {label}: " + json.dumps(m))
    with open("data/lidar_geo_results.json", "w") as f:
        json.dump(results, f, indent=2)
    print("saved -> data/lidar_geo_results.json")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Verify the script parses cleanly (locally, no Sionna needed)**

Run: `python -m py_compile experiments/run_lidar_geo.py && echo OK`
Expected: prints `OK` with no syntax error. (The script is run as a file, not imported
as a package, so a syntax check is the right local gate; execution needs Sionna.)

- [ ] **Step 3: Run on the Sionna-equipped host (amd server), throttled**

Run (on server, repo synced, venv with `sionna-rt`):
`nice -n 19 ionice -c3 python experiments/run_lidar_geo.py`
Expected: prints a metrics line per scene and writes `data/lidar_geo_results.json`. Note: at `OUSTER_OS1` 360°/0.35° (~1028 beams) the pure-Python beam loop is a few seconds per frame — fine throttled; reduce beams via a coarser `LidarConfig` if a full trajectory is slow.

- [ ] **Step 4: Record results and commit**

Add a "Model A (geometric LiDAR)" row to the paper-2 comparison in
`papers/2-wifi-vs-lidar/DOSSIER.md` (and `docs/results-v1.md`) with the six metrics
for both scenes, next to the existing WiFi oracle/realistic numbers.

```bash
git add experiments/run_lidar_geo.py data/lidar_geo_results.json \
        papers/2-wifi-vs-lidar/DOSSIER.md docs/results-v1.md
git commit -m "paper2(lidar/A): model-A baseline results on both scenes"
```

- [ ] **Step 5: Full suite green**

Run: `pytest -q`
Expected: all pass; `test_lidar_geo_scene.py` passes on the server / skips locally.

---

## After branch 1

Merge `paper2-lidar-geo` into `paper2-wifi-vs-lidar`, update the DOSSIER, then start
branch 2 (`paper2-lidar-sionna`, model B — Sionna optical-ray proxy) against the same
`make_sensor` seam. Model B is the first that genuinely needs Sionna path solving, so
its plan will lean on the server workflow throughout.
