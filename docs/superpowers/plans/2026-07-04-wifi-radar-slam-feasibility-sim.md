# WiFi-Radar-for-SLAM Feasibility Simulation — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a physics-based, ray-traced (Sionna RT) simulation that shows whether a vehicle receiving ambient sub-7 GHz WiFi in an outdoor parking-lot scene can map its surroundings and localize itself (SLAM), and characterize when it works.

**Architecture:** A six-stage disk-cached pipeline — `scene → channel → sensing → slam → eval`, driven by a config-based experiment runner. Pure-Python algorithmic stages (config, geometry, sensing super-resolution, particle-filter SLAM, evaluation) are built test-first with synthetic data; the two Sionna-RT stages (scene, channel) are built against the real ray tracer behind a thin wrapper, guarded by a hello-world sanity task so API drift surfaces immediately.

**Tech Stack:** Python 3.11, Sionna RT, TensorFlow (GPU), NumPy, SciPy, Matplotlib, PyYAML, pytest.

## Global Constraints

- **Python:** 3.11 (Sionna RT / TensorFlow supported range).
- **Sionna RT:** pin `sionna>=0.19,<1.1` in `pyproject.toml`; the wrapper isolates the `sionna.rt` API so a version bump touches one file.
- **GPU:** required for Sionna RT stages (confirmed available). Pure-Python stages and their tests must run **without** GPU or Sionna installed (import Sionna lazily, only inside `src/wifi_radar_slam/scene/` and `channel/`).
- **Determinism:** every stochastic step takes an explicit `numpy.random.Generator` seeded from config `seed`. No global `np.random` calls.
- **License/attribution:** AGPL-3.0-or-later; author Mulham Fetna (ORCID 0009-0006-4432-798X). Do not change licensing.
- **Package name:** `wifi_radar_slam`. Import root `src/wifi_radar_slam/`.
- **Units:** SI throughout — metres, seconds, Hz, radians. Document any exception at the call site.
- **Artifacts:** each stage reads/writes `.npz`/`.json` under a per-run directory `results/<run_name>/`; a stage never recomputes an upstream stage whose artifact exists unless `--force`.
- **Config values (v1 nominal, copied verbatim into `configs/nominal.yaml`):** carrier 5.2e9 Hz; bandwidth options {20e6, 40e6, 80e6, 160e6}; nominal bandwidth 40e6; subcarriers 64; rx antennas 4 (uniform linear array, λ/2 spacing); APs 3; vehicle speed 5 m/s; trajectory length 60 m; timestep 0.05 s; noise floor SNR 20 dB nominal.

---

## File Structure

```
pyproject.toml                              # package + deps + pytest config
src/wifi_radar_slam/
  __init__.py
  config.py                                 # dataclasses + YAML loader (Task 1)
  geometry.py                               # poses, trajectory, mirror images, maps (Task 2)
  io_artifacts.py                           # save/load .npz/.json per stage (Task 3)
  sensing/
    __init__.py
    superres.py                             # MUSIC/ESPRIT delay-AoA-Doppler (Task 6)
    frontend.py                             # CSI -> detections per frame (Task 7)
  slam/
    __init__.py
    particle_filter.py                      # RBPF virtual-anchor SLAM (Task 8)
  eval/
    __init__.py
    metrics.py                              # Chamfer, IoU, ATE, RPE (Task 9)
    figures.py                              # plots (Task 9)
  scene/
    __init__.py
    builder.py                              # Sionna RT scene + ground truth (Task 4)
  channel/
    __init__.py
    simulator.py                            # Sionna RT paths -> CSI timeseries (Task 5)
  runner.py                                 # Phase A / Phase B orchestration (Task 10)
configs/
  nominal.yaml                              # Phase A config
  sweep.yaml                                # Phase B sweep grid
experiments/
  run_phase_a.py                            # entry point (Task 10)
  run_phase_b.py                            # entry point (Task 11)
tests/
  test_config.py  test_geometry.py  test_io_artifacts.py
  test_superres.py  test_frontend.py  test_particle_filter.py
  test_metrics.py  test_scene_smoke.py  test_channel_smoke.py  test_runner.py
results/                                    # git-ignored artifacts
```

Build order follows dependencies: skeleton → config → geometry → io → sensing → slam → eval (all Sionna-free, fully unit-tested) → scene → channel (Sionna) → runner (integration).

---

### Task 0: Project skeleton and environment

**Files:**
- Create: `pyproject.toml`, `src/wifi_radar_slam/__init__.py`, `tests/test_import.py`

**Interfaces:**
- Produces: installable package `wifi_radar_slam` with version `0.1.0`.

- [ ] **Step 1: Write `pyproject.toml`**

```toml
[build-system]
requires = ["setuptools>=68"]
build-backend = "setuptools.build_meta"

[project]
name = "wifi-radar-slam"
version = "0.1.0"
description = "Ambient WiFi as a radar replacement for automotive SLAM (feasibility simulation)"
requires-python = ">=3.11,<3.12"
license = { text = "AGPL-3.0-or-later" }
authors = [{ name = "Mulham Fetna", email = "contact@mulhamfetna.com" }]
dependencies = [
  "numpy>=1.26",
  "scipy>=1.11",
  "matplotlib>=3.8",
  "pyyaml>=6.0",
]

[project.optional-dependencies]
sim = ["sionna>=0.19,<1.1", "tensorflow>=2.13"]
dev = ["pytest>=7.4"]

[tool.setuptools.packages.find]
where = ["src"]

[tool.pytest.ini_options]
testpaths = ["tests"]
addopts = "-q"
```

- [ ] **Step 2: Write the package init**

`src/wifi_radar_slam/__init__.py`:
```python
"""WiFi-radar-for-SLAM feasibility simulation."""
__version__ = "0.1.0"
```

- [ ] **Step 3: Write the failing import test**

`tests/test_import.py`:
```python
def test_package_imports():
    import wifi_radar_slam
    assert wifi_radar_slam.__version__ == "0.1.0"
```

- [ ] **Step 4: Install and run**

Run: `pip install -e ".[dev]" && pytest tests/test_import.py -v`
Expected: PASS. (Sionna deliberately not installed yet — the `sim` extra is separate.)

- [ ] **Step 5: Commit**

```bash
git add pyproject.toml src/wifi_radar_slam/__init__.py tests/test_import.py
git commit -m "chore: project skeleton and packaging"
```

---

### Task 1: Config schema and YAML loader

**Files:**
- Create: `src/wifi_radar_slam/config.py`, `configs/nominal.yaml`, `tests/test_config.py`

**Interfaces:**
- Produces:
  - `RFConfig(carrier_hz: float, bandwidth_hz: float, n_subcarriers: int, n_rx_antennas: int, antenna_spacing_frac: float)`
  - `TrajectoryConfig(length_m: float, speed_mps: float, timestep_s: float, shape: str)`
  - `SceneConfig(name: str, ap_positions: list[tuple[float,float,float]], targets: list[dict])`
  - `RunConfig(run_name: str, seed: int, snr_db: float, rf: RFConfig, trajectory: TrajectoryConfig, scene: SceneConfig)`
  - `load_config(path: str) -> RunConfig`

- [ ] **Step 1: Write the failing test**

`tests/test_config.py`:
```python
from wifi_radar_slam.config import load_config, RunConfig

def test_load_nominal(tmp_path):
    cfg = load_config("configs/nominal.yaml")
    assert isinstance(cfg, RunConfig)
    assert cfg.rf.carrier_hz == 5.2e9
    assert cfg.rf.bandwidth_hz == 40e6
    assert cfg.rf.n_rx_antennas == 4
    assert cfg.trajectory.speed_mps == 5.0
    assert len(cfg.scene.ap_positions) == 3
    assert cfg.seed == 42

def test_derived_frame_count():
    cfg = load_config("configs/nominal.yaml")
    # 60 m at 5 m/s = 12 s; /0.05 s = 240 frames
    assert cfg.trajectory.n_frames == 240
```

- [ ] **Step 2: Run to verify it fails**

Run: `pytest tests/test_config.py -v`
Expected: FAIL (`ModuleNotFoundError: wifi_radar_slam.config`).

- [ ] **Step 3: Write `configs/nominal.yaml`**

```yaml
run_name: phase_a_nominal
seed: 42
snr_db: 20.0
rf:
  carrier_hz: 5.2e9
  bandwidth_hz: 40.0e6
  n_subcarriers: 64
  n_rx_antennas: 4
  antenna_spacing_frac: 0.5
trajectory:
  length_m: 60.0
  speed_mps: 5.0
  timestep_s: 0.05
  shape: straight        # straight | curved
scene:
  name: parking_lot
  ap_positions:          # metres (x, y, z); on building facades
    - [0.0, 20.0, 6.0]
    - [40.0, -20.0, 6.0]
    - [60.0, 20.0, 6.0]
  targets:               # static reflectors
    - {kind: car,  center: [10.0, 3.0, 0.75], size: [4.5, 1.8, 1.5]}
    - {kind: car,  center: [20.0, 3.0, 0.75], size: [4.5, 1.8, 1.5]}
    - {kind: pole, center: [15.0, -3.0, 1.5], size: [0.2, 0.2, 3.0]}
    - {kind: wall, center: [30.0, 6.0, 1.5], size: [20.0, 0.3, 3.0]}
```

- [ ] **Step 4: Implement `config.py`**

```python
from __future__ import annotations
from dataclasses import dataclass, field
import yaml


@dataclass(frozen=True)
class RFConfig:
    carrier_hz: float
    bandwidth_hz: float
    n_subcarriers: int
    n_rx_antennas: int
    antenna_spacing_frac: float


@dataclass(frozen=True)
class TrajectoryConfig:
    length_m: float
    speed_mps: float
    timestep_s: float
    shape: str

    @property
    def duration_s(self) -> float:
        return self.length_m / self.speed_mps

    @property
    def n_frames(self) -> int:
        return int(round(self.duration_s / self.timestep_s))


@dataclass(frozen=True)
class SceneConfig:
    name: str
    ap_positions: list[tuple[float, float, float]]
    targets: list[dict]


@dataclass(frozen=True)
class RunConfig:
    run_name: str
    seed: int
    snr_db: float
    rf: RFConfig
    trajectory: TrajectoryConfig
    scene: SceneConfig


def load_config(path: str) -> RunConfig:
    with open(path, "r") as f:
        raw = yaml.safe_load(f)
    rf = RFConfig(**raw["rf"])
    traj = TrajectoryConfig(**raw["trajectory"])
    scene = SceneConfig(
        name=raw["scene"]["name"],
        ap_positions=[tuple(p) for p in raw["scene"]["ap_positions"]],
        targets=list(raw["scene"]["targets"]),
    )
    return RunConfig(
        run_name=raw["run_name"], seed=int(raw["seed"]),
        snr_db=float(raw["snr_db"]), rf=rf, trajectory=traj, scene=scene,
    )
```

- [ ] **Step 5: Run to verify pass**

Run: `pytest tests/test_config.py -v`
Expected: PASS (both tests).

- [ ] **Step 6: Commit**

```bash
git add src/wifi_radar_slam/config.py configs/nominal.yaml tests/test_config.py
git commit -m "feat: config schema and YAML loader"
```

---

### Task 2: Geometry utilities

**Files:**
- Create: `src/wifi_radar_slam/geometry.py`, `tests/test_geometry.py`

**Interfaces:**
- Produces:
  - `straight_trajectory(length_m, speed_mps, timestep_s) -> np.ndarray` shape `(n_frames, 3)` of `[x, y, yaw]` poses (z fixed at 1.5 m rx height, returned separately as constant).
  - `velocity_from_poses(poses, timestep_s) -> np.ndarray` shape `(n_frames, 2)`.
  - `mirror_image(ap_xyz, wall_point, wall_normal) -> np.ndarray` shape `(3,)` — the virtual anchor (mirror of AP across a plane).
  - `targets_to_pointmap(targets, spacing=0.5) -> np.ndarray` shape `(M, 3)` — ground-truth surface point cloud sampled on target boxes.

- [ ] **Step 1: Write the failing test**

`tests/test_geometry.py`:
```python
import numpy as np
from wifi_radar_slam.geometry import (
    straight_trajectory, velocity_from_poses, mirror_image, targets_to_pointmap,
)

def test_straight_trajectory_shape_and_endpoints():
    poses = straight_trajectory(length_m=60.0, speed_mps=5.0, timestep_s=0.05)
    assert poses.shape == (240, 3)
    assert np.isclose(poses[0, 0], 0.0)
    assert np.isclose(poses[-1, 0], 60.0 - 60.0/240)  # last sample just before end
    assert np.allclose(poses[:, 1], 0.0)               # straight along x
    assert np.allclose(poses[:, 2], 0.0)               # yaw 0

def test_velocity_constant():
    poses = straight_trajectory(60.0, 5.0, 0.05)
    vel = velocity_from_poses(poses, 0.05)
    assert vel.shape == (240, 2)
    assert np.allclose(vel[1:, 0], 5.0, atol=1e-6)     # 5 m/s in x

def test_mirror_image_across_y_wall():
    # wall in the plane y=6, normal +y; AP at y=20 mirrors to y=-8
    va = mirror_image(np.array([10.0, 20.0, 6.0]),
                      wall_point=np.array([0.0, 6.0, 0.0]),
                      wall_normal=np.array([0.0, 1.0, 0.0]))
    assert np.allclose(va, [10.0, -8.0, 6.0])

def test_pointmap_covers_boxes():
    targets = [{"kind": "pole", "center": [0.0, 0.0, 1.5], "size": [0.2, 0.2, 3.0]}]
    pts = targets_to_pointmap(targets, spacing=0.5)
    assert pts.shape[1] == 3
    assert pts.shape[0] > 0
    assert np.all(np.abs(pts[:, 0]) <= 0.2)            # within box x-extent
```

- [ ] **Step 2: Run to verify it fails**

Run: `pytest tests/test_geometry.py -v`
Expected: FAIL (module not found).

- [ ] **Step 3: Implement `geometry.py`**

```python
from __future__ import annotations
import numpy as np

RX_HEIGHT_M = 1.5


def straight_trajectory(length_m: float, speed_mps: float, timestep_s: float) -> np.ndarray:
    n = int(round((length_m / speed_mps) / timestep_s))
    x = np.arange(n) * speed_mps * timestep_s
    poses = np.zeros((n, 3))
    poses[:, 0] = x
    return poses


def velocity_from_poses(poses: np.ndarray, timestep_s: float) -> np.ndarray:
    vel = np.zeros((poses.shape[0], 2))
    vel[1:] = (poses[1:, :2] - poses[:-1, :2]) / timestep_s
    vel[0] = vel[1] if poses.shape[0] > 1 else 0.0
    return vel


def mirror_image(ap_xyz: np.ndarray, wall_point: np.ndarray, wall_normal: np.ndarray) -> np.ndarray:
    n = wall_normal / np.linalg.norm(wall_normal)
    d = np.dot(ap_xyz - wall_point, n)
    return ap_xyz - 2.0 * d * n


def targets_to_pointmap(targets: list[dict], spacing: float = 0.5) -> np.ndarray:
    pts = []
    for t in targets:
        cx, cy, cz = t["center"]
        sx, sy, sz = t["size"]
        # sample the six faces of the axis-aligned box on a grid
        xs = np.arange(-sx / 2, sx / 2 + 1e-9, spacing)
        ys = np.arange(-sy / 2, sy / 2 + 1e-9, spacing)
        zs = np.arange(-sz / 2, sz / 2 + 1e-9, spacing)
        for x in xs:
            for y in ys:
                pts.append([cx + x, cy + y, cz - sz / 2])
                pts.append([cx + x, cy + y, cz + sz / 2])
        for x in xs:
            for z in zs:
                pts.append([cx + x, cy - sy / 2, cz + z])
                pts.append([cx + x, cy + sy / 2, cz + z])
        for y in ys:
            for z in zs:
                pts.append([cx - sx / 2, cy + y, cz + z])
                pts.append([cx + sx / 2, cy + y, cz + z])
    return np.unique(np.array(pts), axis=0)
```

- [ ] **Step 4: Run to verify pass**

Run: `pytest tests/test_geometry.py -v`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/geometry.py tests/test_geometry.py
git commit -m "feat: geometry utilities (trajectory, mirror images, ground-truth pointmap)"
```

---

### Task 3: Artifact I/O

**Files:**
- Create: `src/wifi_radar_slam/io_artifacts.py`, `tests/test_io_artifacts.py`

**Interfaces:**
- Produces:
  - `run_dir(run_name: str) -> pathlib.Path` (creates `results/<run_name>/`).
  - `save_array(run_name, stage, name, array)` / `load_array(run_name, stage, name) -> np.ndarray`.
  - `save_json(run_name, stage, name, obj)` / `load_json(run_name, stage, name) -> dict`.
  - `exists(run_name, stage, name) -> bool`.

- [ ] **Step 1: Write the failing test**

`tests/test_io_artifacts.py`:
```python
import numpy as np
from wifi_radar_slam import io_artifacts as io

def test_roundtrip_array(tmp_path, monkeypatch):
    monkeypatch.setattr(io, "RESULTS_ROOT", tmp_path)
    a = np.arange(6).reshape(2, 3).astype(float)
    io.save_array("r1", "channel", "csi", a)
    assert io.exists("r1", "channel", "csi")
    b = io.load_array("r1", "channel", "csi")
    assert np.allclose(a, b)

def test_roundtrip_json(tmp_path, monkeypatch):
    monkeypatch.setattr(io, "RESULTS_ROOT", tmp_path)
    io.save_json("r1", "eval", "metrics", {"ate": 0.3})
    assert io.load_json("r1", "eval", "metrics")["ate"] == 0.3
```

- [ ] **Step 2: Run to verify it fails**

Run: `pytest tests/test_io_artifacts.py -v`
Expected: FAIL (module not found).

- [ ] **Step 3: Implement `io_artifacts.py`**

```python
from __future__ import annotations
import json
import pathlib
import numpy as np

RESULTS_ROOT = pathlib.Path("results")


def run_dir(run_name: str) -> pathlib.Path:
    d = RESULTS_ROOT / run_name
    d.mkdir(parents=True, exist_ok=True)
    return d


def _path(run_name: str, stage: str, name: str, ext: str) -> pathlib.Path:
    d = run_dir(run_name) / stage
    d.mkdir(parents=True, exist_ok=True)
    return d / f"{name}.{ext}"


def save_array(run_name: str, stage: str, name: str, array: np.ndarray) -> None:
    np.savez_compressed(_path(run_name, stage, name, "npz"), data=array)


def load_array(run_name: str, stage: str, name: str) -> np.ndarray:
    with np.load(_path(run_name, stage, name, "npz")) as z:
        return z["data"]


def save_json(run_name: str, stage: str, name: str, obj: dict) -> None:
    _path(run_name, stage, name, "json").write_text(json.dumps(obj, indent=2))


def load_json(run_name: str, stage: str, name: str) -> dict:
    return json.loads(_path(run_name, stage, name, "json").read_text())


def exists(run_name: str, stage: str, name: str) -> bool:
    return (_path(run_name, stage, name, "npz").exists()
            or _path(run_name, stage, name, "json").exists())
```

- [ ] **Step 4: Run to verify pass**

Run: `pytest tests/test_io_artifacts.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/io_artifacts.py tests/test_io_artifacts.py
git commit -m "feat: per-run artifact I/O (npz/json, cache-aware)"
```

---

### Task 4: Sensing — super-resolution parameter estimation

**Files:**
- Create: `src/wifi_radar_slam/sensing/__init__.py`, `src/wifi_radar_slam/sensing/superres.py`, `tests/test_superres.py`

**Interfaces:**
- Consumes: nothing (operates on raw CSI arrays).
- Produces:
  - `estimate_delays(csi_freq: np.ndarray, bandwidth_hz: float, n_paths: int) -> np.ndarray` — MUSIC delay spectrum peak picking; `csi_freq` shape `(n_subcarriers,)` complex; returns delays (s), length ≤ `n_paths`.
  - `estimate_aoa(csi_ant: np.ndarray, spacing_frac: float, n_paths: int) -> np.ndarray` — MUSIC over the antenna dimension; `csi_ant` shape `(n_rx_antennas,)`; returns angles (rad).

This is the most testable stage: feed synthetic multi-tone CSI with known delays/angles and require recovery.

- [ ] **Step 1: Write the failing test**

`tests/test_superres.py`:
```python
import numpy as np
from wifi_radar_slam.sensing.superres import estimate_delays, estimate_aoa

def _synth_freq(delays, subcarrier_freqs):
    # sum of complex exponentials exp(-j 2pi f tau) across subcarriers
    csi = np.zeros(subcarrier_freqs.shape, dtype=complex)
    for tau in delays:
        csi += np.exp(-1j * 2 * np.pi * subcarrier_freqs * tau)
    return csi

def test_recover_two_delays():
    bw = 160e6
    n = 128
    freqs = np.linspace(-bw/2, bw/2, n)
    true = np.array([20e-9, 60e-9])         # 20 ns, 60 ns  (~6 m, ~18 m paths)
    csi = _synth_freq(true, freqs)
    est = np.sort(estimate_delays(csi, bandwidth_hz=bw, n_paths=2))
    assert np.allclose(est, true, atol=3e-9)

def _synth_ant(angles, n_ant, spacing_frac):
    csi = np.zeros(n_ant, dtype=complex)
    idx = np.arange(n_ant)
    for a in angles:
        csi += np.exp(-1j * 2 * np.pi * spacing_frac * idx * np.sin(a))
    return csi

def test_recover_two_angles():
    true = np.array([-0.3, 0.5])            # radians
    csi = _synth_ant(true, n_ant=8, spacing_frac=0.5)
    est = np.sort(estimate_aoa(csi, spacing_frac=0.5, n_paths=2))
    assert np.allclose(est, np.sort(true), atol=0.05)
```

- [ ] **Step 2: Run to verify it fails**

Run: `pytest tests/test_superres.py -v`
Expected: FAIL (module not found).

- [ ] **Step 3: Implement `superres.py`**

```python
from __future__ import annotations
import numpy as np


def _music_1d(samples: np.ndarray, steering, grid, n_sources: int) -> np.ndarray:
    """Generic 1-D MUSIC. `samples` (N,) snapshot; steering(theta)->(N,) vector."""
    x = samples.reshape(-1, 1)
    # single-snapshot: build a spatial-smoothing covariance via forward subarrays
    N = x.shape[0]
    L = N // 2
    sub = np.stack([samples[i:i + L] for i in range(N - L + 1)], axis=1)  # (L, K)
    R = sub @ sub.conj().T / sub.shape[1]
    evals, evecs = np.linalg.eigh(R)
    noise = evecs[:, : L - n_sources]                                    # noise subspace
    spectrum = []
    for g in grid:
        a = steering(g, L)
        denom = np.linalg.norm(noise.conj().T @ a) ** 2 + 1e-12
        spectrum.append(1.0 / denom)
    spectrum = np.array(spectrum)
    # pick n_sources highest peaks
    peaks = _pick_peaks(spectrum, n_sources)
    return grid[peaks]


def _pick_peaks(spectrum: np.ndarray, k: int) -> np.ndarray:
    interior = np.where((spectrum[1:-1] > spectrum[:-2]) & (spectrum[1:-1] > spectrum[2:]))[0] + 1
    if interior.size < k:
        return np.argsort(spectrum)[-k:]
    order = interior[np.argsort(spectrum[interior])[::-1]]
    return order[:k]


def estimate_delays(csi_freq: np.ndarray, bandwidth_hz: float, n_paths: int) -> np.ndarray:
    n = csi_freq.shape[0]
    df = bandwidth_hz / n
    grid = np.linspace(0.0, (n - 1) / bandwidth_hz, 2000)   # delay grid up to 1/df

    def steering(tau, L):
        k = np.arange(L)
        return np.exp(-1j * 2 * np.pi * (k * df) * tau)

    return _music_1d(csi_freq, steering, grid, n_paths)


def estimate_aoa(csi_ant: np.ndarray, spacing_frac: float, n_paths: int) -> np.ndarray:
    grid = np.linspace(-np.pi / 2, np.pi / 2, 2000)

    def steering(theta, L):
        idx = np.arange(L)
        return np.exp(-1j * 2 * np.pi * spacing_frac * idx * np.sin(theta))

    return _music_1d(csi_ant, steering, grid, n_paths)
```

> Note for the implementer: the single-snapshot forward-smoothing covariance above is deliberately simple. If `test_recover_two_delays` misses at `atol=3e-9`, increase the delay `grid` density to 4000 and the subarray count via `L = 2*N//3`. Do not add features beyond passing the two tests.

- [ ] **Step 4: Run to verify pass**

Run: `pytest tests/test_superres.py -v`
Expected: PASS (2 tests). Tune grid/`L` per the note if needed.

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/sensing/__init__.py src/wifi_radar_slam/sensing/superres.py tests/test_superres.py
git commit -m "feat: MUSIC super-resolution for delay and AoA"
```

---

### Task 5: Sensing front-end — CSI → per-frame detections

**Files:**
- Create: `src/wifi_radar_slam/sensing/frontend.py`, `tests/test_frontend.py`

**Interfaces:**
- Consumes: `estimate_delays`, `estimate_aoa` (Task 4); `RFConfig` (Task 1).
- Produces:
  - `extract_detections(csi_timeseries: np.ndarray, rf: RFConfig, n_paths: int = 3) -> list[np.ndarray]` — input shape `(n_frames, n_ap, n_rx_antennas, n_subcarriers)` complex; returns per-frame array shape `(n_paths_found, 3)` columns `[range_m, aoa_rad, ap_index]`. Range = delay·c.

- [ ] **Step 1: Write the failing test**

`tests/test_frontend.py`:
```python
import numpy as np
from wifi_radar_slam.config import RFConfig
from wifi_radar_slam.sensing.frontend import extract_detections

C = 299792458.0

def _make_csi(n_frames, rf, delay_s, aoa_rad):
    freqs = np.linspace(-rf.bandwidth_hz/2, rf.bandwidth_hz/2, rf.n_subcarriers)
    idx = np.arange(rf.n_rx_antennas)
    csi = np.zeros((n_frames, 1, rf.n_rx_antennas, rf.n_subcarriers), dtype=complex)
    for f in range(n_frames):
        delay_phase = np.exp(-1j * 2*np.pi*freqs*delay_s)             # (sub,)
        ant_phase = np.exp(-1j*2*np.pi*rf.antenna_spacing_frac*idx*np.sin(aoa_rad))
        csi[f, 0] = ant_phase[:, None] * delay_phase[None, :]
    return csi

def test_single_target_range_and_angle():
    rf = RFConfig(5.2e9, 160e6, 128, 8, 0.5)
    delay = 40e-9
    aoa = 0.3
    csi = _make_csi(5, rf, delay, aoa)
    dets = extract_detections(csi, rf, n_paths=1)
    assert len(dets) == 5
    r, a, ap = dets[0][0]
    assert np.isclose(r, delay*C, atol=1.0)      # within 1 m
    assert np.isclose(a, aoa, atol=0.05)
    assert ap == 0
```

- [ ] **Step 2: Run to verify it fails**

Run: `pytest tests/test_frontend.py -v`
Expected: FAIL (module not found).

- [ ] **Step 3: Implement `frontend.py`**

```python
from __future__ import annotations
import numpy as np
from ..config import RFConfig
from .superres import estimate_delays, estimate_aoa

C = 299792458.0


def extract_detections(csi_timeseries: np.ndarray, rf: RFConfig, n_paths: int = 3) -> list[np.ndarray]:
    n_frames, n_ap = csi_timeseries.shape[0], csi_timeseries.shape[1]
    out = []
    for f in range(n_frames):
        rows = []
        for ap in range(n_ap):
            block = csi_timeseries[f, ap]                    # (n_ant, n_sub)
            csi_freq = block.mean(axis=0)                    # collapse antennas -> delays
            delays = estimate_delays(csi_freq, rf.bandwidth_hz, n_paths)
            csi_ant = block.mean(axis=1)                     # collapse subcarriers -> AoA
            angles = estimate_aoa(csi_ant, rf.antenna_spacing_frac, n_paths)
            k = min(len(delays), len(angles))
            for i in range(k):
                rows.append([delays[np.argsort(delays)][i] * C,
                             angles[np.argsort(angles)][i], float(ap)])
        out.append(np.array(rows) if rows else np.empty((0, 3)))
    return out
```

> Note: pairing delays with angles by sort index is a v1 simplification (adequate for the single/low-target nominal case and the tests). Joint delay-AoA estimation is a documented future refinement, not part of v1.

- [ ] **Step 4: Run to verify pass**

Run: `pytest tests/test_frontend.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/sensing/frontend.py tests/test_frontend.py
git commit -m "feat: sensing front-end (CSI to per-frame range/AoA detections)"
```

---

### Task 6: SLAM — Rao-Blackwellized particle filter (virtual-anchor)

**Files:**
- Create: `src/wifi_radar_slam/slam/__init__.py`, `src/wifi_radar_slam/slam/particle_filter.py`, `tests/test_particle_filter.py`

**Interfaces:**
- Consumes: detections (Task 5) `list[(k,3)]`; AP positions; `velocity` (Task 2).
- Produces:
  - `run_slam(detections, ap_positions, velocity, timestep_s, rng, n_particles=200) -> tuple[np.ndarray, np.ndarray]` — returns `est_trajectory` shape `(n_frames, 3)` `[x,y,yaw]` and `est_map` shape `(L, 2)` estimated reflector points.

v1 estimator: particles carry the vehicle pose (propagated by odometry = known velocity + noise); per particle, reflector positions are triangulated from range+AoA detections and accumulated; weights come from detection reprojection consistency. Test on a synthetic straight run with one reflector where the answer is analytic.

- [ ] **Step 1: Write the failing test**

`tests/test_particle_filter.py`:
```python
import numpy as np
from wifi_radar_slam.slam.particle_filter import run_slam

def test_recovers_straight_trajectory():
    n = 20
    dt = 0.05
    velocity = np.tile([5.0, 0.0], (n, 1))
    # one reflector at (10, 3); build detections: range+AoA from the true path
    aps = [np.array([0.0, 20.0, 6.0])]
    refl = np.array([10.0, 3.0])
    dets = []
    for f in range(n):
        px, py = f * 5.0 * dt, 0.0
        d = refl - np.array([px, py])
        rng = np.linalg.norm(d)
        aoa = np.arctan2(d[1], d[0])
        dets.append(np.array([[rng, aoa, 0.0]]))
    rng_gen = np.random.default_rng(0)
    traj, mp = run_slam(dets, aps, velocity, dt, rng_gen, n_particles=300)
    assert traj.shape == (n, 3)
    # end position near (0.05*5*19, 0)
    assert np.isclose(traj[-1, 0], 4.75, atol=0.5)
    assert np.isclose(traj[-1, 1], 0.0, atol=0.5)
    # a mapped point near the true reflector
    assert np.min(np.linalg.norm(mp - refl, axis=1)) < 1.0
```

- [ ] **Step 2: Run to verify it fails**

Run: `pytest tests/test_particle_filter.py -v`
Expected: FAIL (module not found).

- [ ] **Step 3: Implement `particle_filter.py`**

```python
from __future__ import annotations
import numpy as np


def _reproject(pose, reflector):
    d = reflector - pose[:2]
    return np.linalg.norm(d), np.arctan2(d[1], d[0])


def run_slam(detections, ap_positions, velocity, timestep_s, rng,
             n_particles: int = 200):
    n_frames = len(detections)
    particles = np.zeros((n_particles, 3))                 # x, y, yaw
    weights = np.ones(n_particles) / n_particles
    est_traj = np.zeros((n_frames, 3))
    mapped_points: list[np.ndarray] = []

    pos_noise = 0.05
    for f in range(n_frames):
        # odometry propagation with process noise
        vx, vy = velocity[f]
        particles[:, 0] += vx * timestep_s + rng.normal(0, pos_noise, n_particles)
        particles[:, 1] += vy * timestep_s + rng.normal(0, pos_noise, n_particles)

        dets = detections[f]
        if dets.shape[0] > 0:
            # triangulate reflectors from the best particle and score all particles
            best = particles[np.argmax(weights)]
            for rng_m, aoa, _ap in dets:
                refl = best[:2] + rng_m * np.array([np.cos(aoa), np.sin(aoa)])
                mapped_points.append(refl)
                # weight update: consistency of each particle with this detection
                pr = np.array([_reproject(p, refl) for p in particles])
                err = (pr[:, 0] - rng_m) ** 2 + (pr[:, 1] - aoa) ** 2
                weights *= np.exp(-0.5 * err / (0.5 ** 2))
            weights += 1e-300
            weights /= weights.sum()

            # resample if effective sample size collapses
            neff = 1.0 / np.sum(weights ** 2)
            if neff < n_particles / 2:
                idx = rng.choice(n_particles, n_particles, p=weights)
                particles = particles[idx]
                weights = np.ones(n_particles) / n_particles

        est_traj[f] = np.average(particles, axis=0, weights=weights)

    est_map = _cluster(np.array(mapped_points)) if mapped_points else np.empty((0, 2))
    return est_traj, est_map


def _cluster(points: np.ndarray, radius: float = 0.5) -> np.ndarray:
    """Greedy merge of nearby mapped points into landmark centroids."""
    kept = []
    used = np.zeros(len(points), dtype=bool)
    for i in range(len(points)):
        if used[i]:
            continue
        d = np.linalg.norm(points - points[i], axis=1)
        group = d < radius
        used |= group
        kept.append(points[group].mean(axis=0))
    return np.array(kept)
```

- [ ] **Step 4: Run to verify pass**

Run: `pytest tests/test_particle_filter.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/slam/__init__.py src/wifi_radar_slam/slam/particle_filter.py tests/test_particle_filter.py
git commit -m "feat: RBPF virtual-anchor SLAM (trajectory + map estimation)"
```

---

### Task 7: Evaluation — metrics and figures

**Files:**
- Create: `src/wifi_radar_slam/eval/__init__.py`, `src/wifi_radar_slam/eval/metrics.py`, `src/wifi_radar_slam/eval/figures.py`, `tests/test_metrics.py`

**Interfaces:**
- Consumes: est/ground-truth trajectory `(n,3)`, est/ground-truth map `(*,2)`/`(*,3)`.
- Produces:
  - `ate(est_traj, gt_traj) -> float` (RMS absolute trajectory error, xy).
  - `rpe(est_traj, gt_traj, delta=1) -> float` (relative pose error).
  - `chamfer(est_map, gt_map_xy) -> float`.
  - `occupancy_iou(est_map, gt_map_xy, cell=1.0, bounds=None) -> float`.
  - `plot_map(est_map, gt_map_xy, est_traj, gt_traj, path)` (figures.py; saves PNG).

- [ ] **Step 1: Write the failing test**

`tests/test_metrics.py`:
```python
import numpy as np
from wifi_radar_slam.eval.metrics import ate, rpe, chamfer, occupancy_iou

def test_ate_zero_for_identical():
    t = np.random.default_rng(0).normal(size=(10, 3))
    assert ate(t, t) == 0.0

def test_ate_constant_offset():
    t = np.zeros((10, 3)); s = t.copy(); s[:, 0] += 2.0
    assert np.isclose(ate(s, t), 2.0)

def test_chamfer_zero_for_identical():
    m = np.array([[0.0, 0.0], [1.0, 1.0]])
    assert np.isclose(chamfer(m, m), 0.0)

def test_iou_identical_grid():
    m = np.array([[0.0, 0.0], [2.0, 2.0]])
    val = occupancy_iou(m, m, cell=1.0, bounds=(-1, 3, -1, 3))
    assert np.isclose(val, 1.0)
```

- [ ] **Step 2: Run to verify it fails**

Run: `pytest tests/test_metrics.py -v`
Expected: FAIL (module not found).

- [ ] **Step 3: Implement `metrics.py`**

```python
from __future__ import annotations
import numpy as np


def ate(est_traj: np.ndarray, gt_traj: np.ndarray) -> float:
    d = est_traj[:, :2] - gt_traj[:, :2]
    return float(np.sqrt(np.mean(np.sum(d ** 2, axis=1))))


def rpe(est_traj: np.ndarray, gt_traj: np.ndarray, delta: int = 1) -> float:
    de = est_traj[delta:, :2] - est_traj[:-delta, :2]
    dg = gt_traj[delta:, :2] - gt_traj[:-delta, :2]
    d = de - dg
    return float(np.sqrt(np.mean(np.sum(d ** 2, axis=1))))


def chamfer(est_map: np.ndarray, gt_map_xy: np.ndarray) -> float:
    if est_map.size == 0 or gt_map_xy.size == 0:
        return float("inf")
    def _nn(a, b):
        return np.mean([np.min(np.linalg.norm(b - p, axis=1)) for p in a])
    return 0.5 * (_nn(est_map, gt_map_xy) + _nn(gt_map_xy, est_map))


def occupancy_iou(est_map, gt_map_xy, cell: float = 1.0, bounds=None) -> float:
    if bounds is None:
        allpts = np.vstack([est_map, gt_map_xy])
        xmin, ymin = allpts.min(0) - cell
        xmax, ymax = allpts.max(0) + cell
    else:
        xmin, xmax, ymin, ymax = bounds
    def _grid(pts):
        gx = np.floor((pts[:, 0] - xmin) / cell).astype(int)
        gy = np.floor((pts[:, 1] - ymin) / cell).astype(int)
        return set(zip(gx.tolist(), gy.tolist()))
    a, b = _grid(est_map), _grid(gt_map_xy)
    if not a and not b:
        return 1.0
    return len(a & b) / len(a | b)
```

- [ ] **Step 4: Implement `figures.py`**

```python
from __future__ import annotations
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def plot_map(est_map, gt_map_xy, est_traj, gt_traj, path: str) -> None:
    fig, ax = plt.subplots(figsize=(8, 6))
    if gt_map_xy.size:
        ax.scatter(gt_map_xy[:, 0], gt_map_xy[:, 1], s=4, c="0.6", label="ground-truth map")
    if est_map.size:
        ax.scatter(est_map[:, 0], est_map[:, 1], s=12, c="C1", marker="x", label="estimated map")
    ax.plot(gt_traj[:, 0], gt_traj[:, 1], "k--", label="ground-truth path")
    ax.plot(est_traj[:, 0], est_traj[:, 1], "C0-", label="estimated path")
    ax.set_aspect("equal"); ax.legend(); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
```

- [ ] **Step 5: Run to verify pass**

Run: `pytest tests/test_metrics.py -v`
Expected: PASS (4 tests).

- [ ] **Step 6: Commit**

```bash
git add src/wifi_radar_slam/eval/ tests/test_metrics.py
git commit -m "feat: evaluation metrics (ATE/RPE/Chamfer/IoU) and map figure"
```

---

### Task 8: Scene builder (Sionna RT)

**Files:**
- Create: `src/wifi_radar_slam/scene/__init__.py`, `src/wifi_radar_slam/scene/builder.py`, `tests/test_scene_smoke.py`

**Interfaces:**
- Consumes: `SceneConfig`, `TrajectoryConfig`, `RFConfig` (Task 1); geometry helpers (Task 2).
- Produces:
  - `build_scene(cfg) -> BuiltScene` where `BuiltScene` has `.scene` (Sionna `Scene`), `.trajectory` `(n,3)`, `.ap_positions list[np.ndarray]`, `.ground_truth_map` `(M,3)`.

Sionna RT is required here; the test is a **GPU/Sionna smoke test** (skipped when Sionna is absent), not a pure unit test.

- [ ] **Step 1: Write the smoke test**

`tests/test_scene_smoke.py`:
```python
import numpy as np
import pytest
pytest.importorskip("sionna")
from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene

def test_build_scene_smoke():
    cfg = load_config("configs/nominal.yaml")
    built = build_scene(cfg)
    assert built.trajectory.shape == (cfg.trajectory.n_frames, 3)
    assert len(built.ap_positions) == 3
    assert built.ground_truth_map.shape[1] == 3
    assert built.ground_truth_map.shape[0] > 0
```

- [ ] **Step 2: Run to verify it fails (or skips without Sionna)**

Run: `pytest tests/test_scene_smoke.py -v`
Expected: FAIL (`build_scene` undefined) when Sionna present; SKIP when absent.

- [ ] **Step 3: Implement `builder.py`**

```python
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from ..config import RunConfig
from ..geometry import straight_trajectory, targets_to_pointmap, RX_HEIGHT_M

import sionna.rt as rt   # lazy heavy import isolated to this module


@dataclass
class BuiltScene:
    scene: "rt.Scene"
    trajectory: np.ndarray
    ap_positions: list
    ground_truth_map: np.ndarray


def build_scene(cfg: RunConfig) -> BuiltScene:
    # start from an empty scene; add a ground plane + box targets
    scene = rt.load_scene()  # empty scene
    scene.frequency = cfg.rf.carrier_hz

    # ground plane
    scene.add(rt.Rectangle(name="ground", size=[200.0, 200.0],
                           position=[30.0, 0.0, 0.0],
                           material=rt.RadioMaterial("itu_concrete")))

    # box targets (cars/poles/walls) as cuboids
    for i, t in enumerate(cfg.scene.targets):
        scene.add(rt.Box(name=f"target_{i}", size=list(t["size"]),
                         position=list(t["center"]),
                         material=rt.RadioMaterial("itu_metal" if t["kind"] == "car"
                                                   else "itu_concrete")))

    # transmit array = APs (isotropic-ish), receive array = vehicle ULA
    scene.tx_array = rt.PlanarArray(num_rows=1, num_cols=1,
                                    vertical_spacing=0.5, horizontal_spacing=0.5,
                                    pattern="iso", polarization="V")
    scene.rx_array = rt.PlanarArray(num_rows=1, num_cols=cfg.rf.n_rx_antennas,
                                    vertical_spacing=0.5,
                                    horizontal_spacing=cfg.rf.antenna_spacing_frac,
                                    pattern="iso", polarization="V")

    ap_positions = [np.array(p, dtype=float) for p in cfg.scene.ap_positions]
    for i, ap in enumerate(ap_positions):
        scene.add(rt.Transmitter(name=f"ap_{i}", position=ap.tolist()))

    traj = straight_trajectory(cfg.trajectory.length_m, cfg.trajectory.speed_mps,
                               cfg.trajectory.timestep_s)
    gt_map = targets_to_pointmap(cfg.scene.targets, spacing=0.5)
    return BuiltScene(scene=scene, trajectory=traj,
                      ap_positions=ap_positions, ground_truth_map=gt_map)
```

> Implementer note: exact class names (`Rectangle`, `Box`, `RadioMaterial`, `PlanarArray`, `Transmitter`, `load_scene`) target Sionna RT 0.19.x. If the pinned Sionna exposes different constructors, adapt **only within this file** (that is its purpose) and keep `BuiltScene`'s fields identical so downstream tasks are unaffected. Verify against `python -c "import sionna.rt as rt; help(rt)"`.

- [ ] **Step 4: Run the smoke test**

Run: `pytest tests/test_scene_smoke.py -v`
Expected: PASS on the GPU box.

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/scene/ tests/test_scene_smoke.py
git commit -m "feat: Sionna RT scene builder with ground-truth map"
```

---

### Task 9: Channel simulator (Sionna RT → CSI)

**Files:**
- Create: `src/wifi_radar_slam/channel/__init__.py`, `src/wifi_radar_slam/channel/simulator.py`, `tests/test_channel_smoke.py`

**Interfaces:**
- Consumes: `BuiltScene` (Task 8), `RFConfig`, `snr_db`, `rng`.
- Produces:
  - `simulate_csi(built, rf, snr_db, rng) -> np.ndarray` complex, shape `(n_frames, n_ap, n_rx_antennas, n_subcarriers)`.

- [ ] **Step 1: Write the smoke test**

`tests/test_channel_smoke.py`:
```python
import numpy as np
import pytest
pytest.importorskip("sionna")
from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.channel.simulator import simulate_csi

def test_simulate_csi_shape():
    cfg = load_config("configs/nominal.yaml")
    built = build_scene(cfg)
    # shorten trajectory for the smoke test
    built.trajectory = built.trajectory[:4]
    csi = simulate_csi(built, cfg.rf, cfg.snr_db, np.random.default_rng(0))
    assert csi.shape == (4, 3, cfg.rf.n_rx_antennas, cfg.rf.n_subcarriers)
    assert np.iscomplexobj(csi)
    assert np.all(np.isfinite(csi))
```

- [ ] **Step 2: Run to verify it fails / skips**

Run: `pytest tests/test_channel_smoke.py -v`
Expected: FAIL (`simulate_csi` undefined) with Sionna present; SKIP without.

- [ ] **Step 3: Implement `simulator.py`**

```python
from __future__ import annotations
import numpy as np
from ..config import RFConfig
from ..scene.builder import BuiltScene
from ..geometry import RX_HEIGHT_M

import sionna.rt as rt


def _subcarrier_freqs(rf: RFConfig) -> np.ndarray:
    return np.linspace(-rf.bandwidth_hz / 2, rf.bandwidth_hz / 2, rf.n_subcarriers)


def simulate_csi(built: BuiltScene, rf: RFConfig, snr_db: float, rng) -> np.ndarray:
    scene = built.scene
    n_frames = built.trajectory.shape[0]
    n_ap = len(built.ap_positions)
    freqs = _subcarrier_freqs(rf)
    csi = np.zeros((n_frames, n_ap, rf.n_rx_antennas, rf.n_subcarriers), dtype=complex)

    # one receiver, repositioned per frame; velocity set for Doppler
    scene.add(rt.Receiver(name="veh", position=[0.0, 0.0, RX_HEIGHT_M]))

    for f in range(n_frames):
        x, y, yaw = built.trajectory[f]
        scene.receivers["veh"].position = [float(x), float(y), RX_HEIGHT_M]
        paths = scene.compute_paths(max_depth=3, num_samples=1_000_000)
        # frequency response: (num_rx, num_rx_ant, num_tx, num_tx_ant, num_freqs)
        h_freq = paths.cfr(frequencies=freqs, normalize=False).numpy()
        # collapse tx antenna (1) and rx index (1); keep [tx=ap, rx_ant, freq]
        # h_freq shape -> squeeze to (n_ap, n_rx_antennas, n_subcarriers)
        h = np.squeeze(h_freq)
        if h.ndim == 2:              # single AP edge case
            h = h[None, ...]
        csi[f] = np.transpose(h, (0, 1, 2))[:n_ap]

    # additive white Gaussian noise at the configured SNR
    sig_p = np.mean(np.abs(csi) ** 2) + 1e-30
    noise_p = sig_p / (10 ** (snr_db / 10))
    noise = (rng.normal(size=csi.shape) + 1j * rng.normal(size=csi.shape)) * np.sqrt(noise_p / 2)
    return csi + noise
```

> Implementer note: `compute_paths(...)` / `paths.cfr(...)` are the Sionna RT 0.19.x calls for a channel frequency response. Newer Sionna exposes `PathSolver` + `paths.cir()`; if so, adapt within this file and keep the returned array shape `(n_frames, n_ap, n_rx_antennas, n_subcarriers)` exactly. The axis `squeeze`/`transpose` must be verified against the real array shape printed once during bring-up — add a temporary `print(h_freq.shape)` on first run, then remove.

- [ ] **Step 4: Run the smoke test**

Run: `pytest tests/test_channel_smoke.py -v`
Expected: PASS on the GPU box (shape + finiteness).

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/channel/ tests/test_channel_smoke.py
git commit -m "feat: Sionna RT channel simulator producing CSI timeseries"
```

---

### Task 10: Phase-A runner (end-to-end nominal)

**Files:**
- Create: `src/wifi_radar_slam/runner.py`, `experiments/run_phase_a.py`, `tests/test_runner.py`

**Interfaces:**
- Consumes: all prior stages.
- Produces:
  - `run_phase_a(cfg, rng, force=False) -> dict` — runs the full pipeline with disk caching; returns metrics dict `{ate, rpe, chamfer, iou}` and writes `results/<run>/eval/metrics.json` + map figure.

The runner is tested with the Sionna stages **monkeypatched** by a synthetic channel, so the end-to-end wiring is verified without a GPU.

- [ ] **Step 1: Write the failing test**

`tests/test_runner.py`:
```python
import numpy as np
from wifi_radar_slam.config import load_config
from wifi_radar_slam import runner

def test_phase_a_wiring(monkeypatch, tmp_path):
    from wifi_radar_slam import io_artifacts as io
    monkeypatch.setattr(io, "RESULTS_ROOT", tmp_path)
    cfg = load_config("configs/nominal.yaml")

    # fake scene + channel so no Sionna/GPU is needed
    class FakeBuilt:
        trajectory = np.column_stack([np.linspace(0, 10, 20), np.zeros(20), np.zeros(20)])
        ap_positions = [np.array([0.0, 20.0, 6.0])]
        ground_truth_map = np.array([[10.0, 3.0, 0.75]])

    def fake_build(_cfg): return FakeBuilt()
    def fake_csi(built, rf, snr, rng):
        n = built.trajectory.shape[0]
        return np.ones((n, 1, rf.n_rx_antennas, rf.n_subcarriers), dtype=complex)

    monkeypatch.setattr(runner, "build_scene", fake_build)
    monkeypatch.setattr(runner, "simulate_csi", fake_csi)

    metrics = runner.run_phase_a(cfg, np.random.default_rng(0))
    assert set(metrics) == {"ate", "rpe", "chamfer", "iou"}
    assert np.isfinite(metrics["ate"])
```

- [ ] **Step 2: Run to verify it fails**

Run: `pytest tests/test_runner.py -v`
Expected: FAIL (module/attr not found).

- [ ] **Step 3: Implement `runner.py`**

```python
from __future__ import annotations
import numpy as np
from .config import RunConfig
from .geometry import velocity_from_poses
from .sensing.frontend import extract_detections
from .slam.particle_filter import run_slam
from .eval.metrics import ate, rpe, chamfer, occupancy_iou
from .eval.figures import plot_map
from . import io_artifacts as io
from .scene.builder import build_scene          # patched in tests
from .channel.simulator import simulate_csi      # patched in tests


def run_phase_a(cfg: RunConfig, rng, force: bool = False) -> dict:
    run = cfg.run_name
    built = build_scene(cfg)

    if force or not io.exists(run, "channel", "csi"):
        csi = simulate_csi(built, cfg.rf, cfg.snr_db, rng)
        io.save_array(run, "channel", "csi", csi)
    else:
        csi = io.load_array(run, "channel", "csi")

    detections = extract_detections(csi, cfg.rf, n_paths=3)
    velocity = velocity_from_poses(built.trajectory, cfg.trajectory.timestep_s)
    est_traj, est_map = run_slam(detections, built.ap_positions, velocity,
                                 cfg.trajectory.timestep_s, rng)

    gt_traj = built.trajectory
    gt_xy = built.ground_truth_map[:, :2]
    metrics = {
        "ate": ate(est_traj, gt_traj),
        "rpe": rpe(est_traj, gt_traj),
        "chamfer": chamfer(est_map, gt_xy),
        "iou": occupancy_iou(est_map, gt_xy, cell=1.0),
    }
    io.save_json(run, "eval", "metrics", metrics)
    plot_map(est_map, gt_xy, est_traj, gt_traj,
             str(io.run_dir(run) / "eval" / "map.png"))
    return metrics
```

- [ ] **Step 4: Write the CLI entry point**

`experiments/run_phase_a.py`:
```python
import numpy as np
from wifi_radar_slam.config import load_config
from wifi_radar_slam.runner import run_phase_a

if __name__ == "__main__":
    cfg = load_config("configs/nominal.yaml")
    metrics = run_phase_a(cfg, np.random.default_rng(cfg.seed))
    print(metrics)
```

- [ ] **Step 5: Run to verify pass**

Run: `pytest tests/test_runner.py -v`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add src/wifi_radar_slam/runner.py experiments/run_phase_a.py tests/test_runner.py
git commit -m "feat: Phase-A end-to-end runner with disk caching"
```

---

### Task 11: Phase-B sweep runner

**Files:**
- Create: `configs/sweep.yaml`, `experiments/run_phase_b.py`
- Modify: `src/wifi_radar_slam/runner.py` (add `run_phase_b`)

**Interfaces:**
- Consumes: `run_phase_a`.
- Produces: `run_phase_b(base_cfg, sweep, rng) -> list[dict]` — for each grid point, sets the swept field, runs Phase A under a derived `run_name`, collects metrics + swept value; writes `results/sweep/summary.json` and a curve figure per swept parameter.

- [ ] **Step 1: Write `configs/sweep.yaml`**

```yaml
base: configs/nominal.yaml
sweeps:
  bandwidth_hz: [20.0e6, 40.0e6, 80.0e6, 160.0e6]
  snr_db: [0.0, 10.0, 20.0, 30.0]
  n_aps: [1, 2, 3, 4]
  speed_mps: [1.0, 5.0, 10.0, 15.0]
```

- [ ] **Step 2: Write the failing test**

Append to `tests/test_runner.py`:
```python
def test_phase_b_grid(monkeypatch, tmp_path):
    from wifi_radar_slam import io_artifacts as io
    from wifi_radar_slam import runner
    import numpy as np
    monkeypatch.setattr(io, "RESULTS_ROOT", tmp_path)
    # stub run_phase_a to avoid re-running the pipeline
    monkeypatch.setattr(runner, "run_phase_a",
                        lambda cfg, rng, force=False: {"ate": 1.0, "rpe": 0.1,
                                                       "chamfer": 2.0, "iou": 0.5})
    from wifi_radar_slam.config import load_config
    base = load_config("configs/nominal.yaml")
    results = runner.run_phase_b(base, {"bandwidth_hz": [20e6, 160e6]},
                                 np.random.default_rng(0))
    assert len(results) == 2
    assert results[0]["swept_param"] == "bandwidth_hz"
```

- [ ] **Step 3: Run to verify it fails**

Run: `pytest tests/test_runner.py::test_phase_b_grid -v`
Expected: FAIL (`run_phase_b` undefined).

- [ ] **Step 4: Add `run_phase_b` to `runner.py`**

```python
import dataclasses

def run_phase_b(base_cfg: RunConfig, sweep: dict, rng) -> list[dict]:
    results = []
    for param, values in sweep.items():
        for v in values:
            rf = base_cfg.rf
            cfg = base_cfg
            if param == "bandwidth_hz":
                rf = dataclasses.replace(base_cfg.rf, bandwidth_hz=float(v))
                cfg = dataclasses.replace(base_cfg, rf=rf,
                                          run_name=f"sweep_{param}_{v:.0f}")
            elif param == "snr_db":
                cfg = dataclasses.replace(base_cfg, snr_db=float(v),
                                          run_name=f"sweep_{param}_{v:.0f}")
            elif param == "speed_mps":
                traj = dataclasses.replace(base_cfg.trajectory, speed_mps=float(v))
                cfg = dataclasses.replace(base_cfg, trajectory=traj,
                                          run_name=f"sweep_{param}_{v:.0f}")
            elif param == "n_aps":
                sc = dataclasses.replace(
                    base_cfg.scene,
                    ap_positions=base_cfg.scene.ap_positions[: int(v)])
                cfg = dataclasses.replace(base_cfg, scene=sc,
                                          run_name=f"sweep_{param}_{int(v)}")
            m = run_phase_a(cfg, rng)
            results.append({"swept_param": param, "value": float(v), **m})
    io.save_json("sweep", "eval", "summary", {"results": results})
    return results
```

- [ ] **Step 5: Write the entry point**

`experiments/run_phase_b.py`:
```python
import numpy as np, yaml
from wifi_radar_slam.config import load_config
from wifi_radar_slam.runner import run_phase_b

if __name__ == "__main__":
    spec = yaml.safe_load(open("configs/sweep.yaml"))
    base = load_config(spec["base"])
    results = run_phase_b(base, spec["sweeps"], np.random.default_rng(base.seed))
    print(f"{len(results)} sweep points written to results/sweep/eval/summary.json")
```

- [ ] **Step 6: Run to verify pass**

Run: `pytest tests/test_runner.py -v`
Expected: PASS (all runner tests).

- [ ] **Step 7: Commit**

```bash
git add configs/sweep.yaml experiments/run_phase_b.py src/wifi_radar_slam/runner.py tests/test_runner.py
git commit -m "feat: Phase-B parameter sweep runner (bandwidth/SNR/speed/AP density)"
```

---

### Task 12: Full-suite green + bring-up doc

**Files:**
- Create: `docs/RUNNING.md`

- [ ] **Step 1: Run the full Sionna-free suite**

Run: `pytest -q`
Expected: all pure-Python tests PASS; the two `_smoke` tests SKIP (no Sionna in this env) — that is acceptable here.

- [ ] **Step 2: On the GPU box, run the smoke tests**

Run: `pip install -e ".[sim,dev]" && pytest tests/test_scene_smoke.py tests/test_channel_smoke.py -v`
Expected: PASS. Fix any Sionna API mismatch **inside** `scene/builder.py` / `channel/simulator.py` only.

- [ ] **Step 3: Run Phase A and Phase B for real**

Run: `python experiments/run_phase_a.py && python experiments/run_phase_b.py`
Expected: `results/phase_a_nominal/eval/metrics.json` + `map.png`; `results/sweep/eval/summary.json`.

- [ ] **Step 4: Write `docs/RUNNING.md`** documenting the exact commands above, the GPU requirement, and where outputs land.

- [ ] **Step 5: Commit**

```bash
git add docs/RUNNING.md
git commit -m "docs: bring-up and run instructions"
```

---

## Self-Review

**Spec coverage:**
- §1 goal (map + localize, sub-7 GHz, outdoor lot) → Tasks 8–11. ✓
- §2 Phase A metrics (Chamfer/IoU/ATE/RPE) → Task 7 + 10. ✓
- §2 Phase B sweeps (AP density, SNR, speed, bandwidth) → Task 11. ✓
- §3.1 scene builder → Task 8. §3.2 channel → Task 9. §3.3 sensing/super-resolution → Tasks 4–5. §3.4 RBPF virtual-anchor SLAM → Task 6. §3.5 evaluator → Task 7. §3.6 experiment runner → Tasks 10–11. ✓
- §4 disk-cached artifacts → Task 3, used in Task 10. ✓
- §5 stack/layout, Sionna-free pure stages → enforced by Global Constraints + lazy imports. ✓
- §6 risks: ray-trace cost (smoke test uses 4 frames), under-resolution (Phase B), data association (v1 uses geometric triangulation from known odometry; blind association listed as future refinement in Task 5 note), ego-Doppler (velocity from odometry in Task 10). ✓
- §7 acceptance → Task 12. ✓

**Placeholder scan:** No "TBD/handle edge cases/write tests for the above" — every code step has real code; Sionna API-drift notes point to a specific isolation file, not vague deferral. ✓

**Type consistency:** `RunConfig`/`RFConfig` fields match across Tasks 1, 8, 9, 10, 11. `extract_detections` returns `list[(k,3)]` consumed by `run_slam` (Task 6/10). `est_map (L,2)` / `gt_xy (M,2)` consistent in metrics (Task 7) and runner (Task 10). CSI shape `(n_frames, n_ap, n_rx_antennas, n_subcarriers)` identical in Tasks 5, 9, 10. ✓

**Known v1 simplifications (documented, intentional):** delay–AoA pairing by sort index (Task 5 note); SLAM uses odometry-driven particles with geometric landmark triangulation rather than full joint data association (Task 6 + Task 5 note) — both adequate for the nominal/low-target feasibility claim and flagged as future work in the spec's out-of-scope list.
