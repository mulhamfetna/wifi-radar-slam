# Paper 2 LiDAR Model C — KITTI external validity (branch 3) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run our shared ICP SLAM (`run_lidar_slam`) on **real** KITTI odometry LiDAR (sequence 04) and report aligned ATE against KITTI ground-truth poses — an external-validity anchor showing our idealized models A/B and our pipeline behave like real LiDAR. This is **not** an in-scene rival baseline (different world, no shared GT, no fusion).

**Architecture:** One new module `src/wifi_radar_slam/lidar/kitti.py` with pure loaders/alignment: read a velodyne `.bin` and slice a horizontal band into a 2D `Scan`; parse KITTI poses + calib into a BEV ground-truth trajectory; rigid-align an estimated trajectory to GT and compute ATE (reusing branch-0's `_rigid_2d`/`_apply`). A fetch script pulls only sequence 04 from KITTI's public S3 via HTTP range requests (`remotezip`), and a run script drives SLAM + ATE on the server.

**Tech Stack:** Python 3, NumPy. `remotezip` (experiment-only, pip-installed on the server) for partial download. Reuses branch-0 `lidar.pointcloud.Scan`, `lidar.slam_icp.{run_lidar_slam,_rigid_2d,_apply}`.

## Global Constraints

- **Branch:** all work on `paper2-lidar-kitti`, cut from `paper2-wifi-vs-lidar`; merge back on completion. Never commit to `main`/`paper1-*`.
- **NumPy only** in library code; `remotezip` is imported only inside the fetch **experiment** script, not the package (keeps `pip install -e .` core deps unchanged).
- **No large data in git:** KITTI scans download to `data/kitti/` which is **git-ignored**; only the small results JSON + DOSSIER row are committed. Add `data/kitti/` to `.gitignore`.
- **2D BEV plane:** velodyne points are sliced to a horizontal band → `xy`; GT is BEV-projected `(x, z)` in the KITTI camera-0 world frame.
- **Pure loaders/alignment test locally** with synthesized `.bin`/pose/calib text; the download and full SLAM run are server-only (documented, not gated tests — they need network + data).
- **KITTI coordinate facts (verified 2026-07-10):** `poses/SS.txt` lines are 3×4 `[R|t]` camera-to-world (translation = columns 3/7/11); `calib.txt` `Tr:` is 3×4 velodyne→camera. Velodyne origin in world at frame i: `R_i @ Tr[:,3] + t_i`; BEV ground trajectory = its `(x, z)`.
- **Metric scope:** model C yields **only an aligned-ATE anchor** (there is no GT *map*, so Chamfer/IoU/completeness do not apply). Reported separately from the in-scene A/B table.
- **Velocity prior:** `run_lidar_slam` needs a per-frame velocity to seed each ICP; use velocity differenced from the GT trajectory at 10 Hz — the **same recipe the sim runs use** (`velocity_from_poses` on GT). The final pose is ICP's output (velocity only seeds the ICP init), so this measures real scan-matching drift, consistent with A/B.
- **Server:** amd (`/home/dev/mulham/wifi-radar-slam`, `.venv`), throttled `nice -n 19 ionice -c3`.

---

### Task 1: `kitti.py` — pure loaders + trajectory alignment

**Files:**
- Create: `src/wifi_radar_slam/lidar/kitti.py`
- Test: `tests/test_lidar_kitti.py`

**Interfaces:**
- Consumes: `Scan` (branch 0), `_rigid_2d`/`_apply` (branch 0 `slam_icp`).
- Produces: `load_velodyne_scan(path, z_lo=-0.5, z_hi=0.5) -> Scan`; `load_gt_trajectory(poses_text, calib_text) -> np.ndarray (N,2)`; `align_2d_ate(est_xy, gt_xy) -> float`. Used by Task 3.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_lidar_kitti.py
import numpy as np
from wifi_radar_slam.lidar.kitti import (load_velodyne_scan, load_gt_trajectory,
                                        align_2d_ate)


def test_velodyne_scan_slices_z_band(tmp_path):
    # 3 points: two inside z-band [-0.5,0.5], one above it
    pts = np.array([[1, 2, 0.0, 0.1], [3, 4, 0.4, 0.2], [5, 6, 2.0, 0.3]], dtype=np.float32)
    f = tmp_path / "000000.bin"
    pts.tofile(f)
    scan = load_velodyne_scan(str(f))
    assert len(scan) == 2
    assert np.allclose(np.sort(scan.points[:, 0]), [1.0, 3.0])


def test_gt_trajectory_identity_calib():
    # two frames: identity rotation, translations along world x then z.
    # Tr = identity rotation, zero offset -> velo origin == pose translation.
    poses = "1 0 0 0 0 1 0 0 0 0 1 0\n1 0 0 2 0 1 0 0 0 0 1 5\n"
    calib = ("P0: 0 0 0 0 0 0 0 0 0 0 0 0\n"
             "Tr: 1 0 0 0 0 1 0 0 0 0 1 0\n")
    gt = load_gt_trajectory(poses, calib)
    assert gt.shape == (2, 2)
    assert np.allclose(gt[0], [0.0, 0.0])
    assert np.allclose(gt[1], [2.0, 5.0])          # (x, z)


def test_align_2d_ate_recovers_after_rigid_offset():
    gt = np.array([[0, 0], [1, 0], [2, 0], [2, 1]], dtype=float)
    # est = gt rotated 0.3 rad + translated; alignment should drive ATE ~0
    th = 0.3
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    est = gt @ R.T + np.array([3.0, -2.0])
    assert align_2d_ate(est, gt) < 1e-6
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_lidar_kitti.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'wifi_radar_slam.lidar.kitti'`

- [ ] **Step 3: Write minimal implementation**

```python
# src/wifi_radar_slam/lidar/kitti.py
"""KITTI odometry external-validity: real velodyne LiDAR through our ICP SLAM.

Pure NumPy loaders + trajectory alignment; no network here (fetch is a separate
experiment script). BEV: velodyne points sliced to a horizontal band -> (x,y);
KITTI GT poses (camera-to-world) + Tr (velodyne->camera) -> velodyne (x,z) track.
"""
from __future__ import annotations
import numpy as np
from .pointcloud import Scan
from .slam_icp import _rigid_2d, _apply


def load_velodyne_scan(path: str, z_lo: float = -0.5, z_hi: float = 0.5) -> Scan:
    """Read a KITTI velodyne .bin (N x [x,y,z,reflectance]) and slice a horizontal
    band into a 2D BEV Scan (points already in the sensor-local velodyne frame)."""
    pts = np.fromfile(path, dtype=np.float32).reshape(-1, 4)
    band = (pts[:, 2] >= z_lo) & (pts[:, 2] <= z_hi)
    return Scan(pts[band, :2].astype(float))


def _parse_pose_matrices(text: str) -> np.ndarray:
    rows = [list(map(float, ln.split())) for ln in text.strip().splitlines() if ln.strip()]
    return np.array(rows).reshape(-1, 3, 4)          # (N,3,4) camera-to-world


def _parse_calib_Tr(text: str) -> np.ndarray:
    for ln in text.strip().splitlines():
        if ln.startswith("Tr:"):
            return np.array(list(map(float, ln.split()[1:]))).reshape(3, 4)
    raise ValueError("no 'Tr:' line in calib text")


def load_gt_trajectory(poses_text: str, calib_text: str) -> np.ndarray:
    """BEV ground-truth trajectory (N,2) of the velodyne origin, as KITTI (x,z)."""
    P = _parse_pose_matrices(poses_text)             # (N,3,4)
    Tr = _parse_calib_Tr(calib_text)                 # (3,4) velo->cam
    tr_t = Tr[:, 3]                                   # velo origin in camera frame
    R, t = P[:, :, :3], P[:, :, 3]                   # (N,3,3), (N,3)
    velo_world = np.einsum("nij,j->ni", R, tr_t) + t  # (N,3) in cam-0 world
    return velo_world[:, [0, 2]]                      # BEV ground plane (x, z)


def align_2d_ate(est_xy, gt_xy) -> float:
    """Rigidly align est to GT (2D Kabsch) and return ATE (RMSE of positions)."""
    est_xy = np.asarray(est_xy, dtype=float)[:, :2]
    gt_xy = np.asarray(gt_xy, dtype=float)[:, :2]
    n = min(len(est_xy), len(gt_xy))
    est_xy, gt_xy = est_xy[:n], gt_xy[:n]
    x, y, yaw = _rigid_2d(est_xy, gt_xy)
    aligned = _apply(est_xy, x, y, yaw)
    return float(np.sqrt(np.mean(np.sum((aligned - gt_xy) ** 2, axis=1))))
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_lidar_kitti.py -v`
Expected: PASS (3 tests)

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/lidar/kitti.py tests/test_lidar_kitti.py
git commit -m "paper2(lidar/C): KITTI loaders + 2D trajectory alignment (pure)"
```

---

### Task 2: fetch script — partial KITTI seq-04 download (server)

**Files:**
- Create: `experiments/fetch_kitti.py`
- Modify: `.gitignore` (add `data/kitti/`)

**Interfaces:**
- Consumes: `remotezip` (server pip). Produces `data/kitti/sequences/04/{velodyne/*.bin,calib.txt}` and `data/kitti/poses/04.txt`.

- [ ] **Step 1: Add the gitignore entry**

Append to `.gitignore`:
```
# KITTI external-validity data (branch 3, model C) — large, fetched on demand
data/kitti/
```

- [ ] **Step 2: Write the fetch script**

```python
# experiments/fetch_kitti.py
"""Fetch one KITTI odometry sequence (default 04) from KITTI's public S3 using
HTTP range requests, so we download ~0.5 GB instead of the 84 GB full velodyne zip.

Server-only (needs network + `pip install remotezip`):
    .venv/bin/pip install remotezip
    nice -n 19 ionice -c3 python experiments/fetch_kitti.py
"""
import os
from remotezip import RemoteZip

BASE = "https://s3.eu-central-1.amazonaws.com/avg-kitti/"
SEQ = "04"
OUT = "data/kitti"


def main() -> None:
    vdir = f"{OUT}/sequences/{SEQ}/velodyne"
    os.makedirs(vdir, exist_ok=True)
    os.makedirs(f"{OUT}/poses", exist_ok=True)
    with RemoteZip(BASE + "data_odometry_poses.zip") as z:
        open(f"{OUT}/poses/{SEQ}.txt", "wb").write(z.read(f"dataset/poses/{SEQ}.txt"))
    with RemoteZip(BASE + "data_odometry_calib.zip") as z:
        open(f"{OUT}/sequences/{SEQ}/calib.txt", "wb").write(
            z.read(f"dataset/sequences/{SEQ}/calib.txt"))
    with RemoteZip(BASE + "data_odometry_velodyne.zip") as z:
        vel = sorted(n for n in z.namelist()
                     if f"/sequences/{SEQ}/velodyne/" in n and n.endswith(".bin"))
        print(f"downloading {len(vel)} velodyne frames for seq {SEQ}")
        for i, n in enumerate(vel):
            open(f"{vdir}/{os.path.basename(n)}", "wb").write(z.read(n))
            if i % 50 == 0:
                print("  ", i, "/", len(vel))
    print("done ->", vdir)


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: Parse-check locally**

Run: `python -m py_compile experiments/fetch_kitti.py && echo OK`
Expected: `OK`.

- [ ] **Step 4: Run on the server**

`.venv/bin/pip install remotezip` then
`nice -n 19 ionice -c3 python experiments/fetch_kitti.py`
Expected: 271 frames written to `data/kitti/sequences/04/velodyne/` (~0.5 GB), plus poses/calib.

- [ ] **Step 5: Commit (gitignore only; data is ignored)**

```bash
git add .gitignore experiments/fetch_kitti.py
git commit -m "paper2(lidar/C): KITTI seq-04 partial fetch script + gitignore data/kitti"
```

---

### Task 3: run script — real-LiDAR SLAM + aligned ATE (server)

**Files:**
- Create: `experiments/run_lidar_kitti.py`

**Interfaces:**
- Consumes: `load_velodyne_scan`, `load_gt_trajectory`, `align_2d_ate` (Task 1), `run_lidar_slam` (branch 0).
- Produces: aligned ATE for seq 04, printed and saved to `data/kitti_results.json` (committed).

- [ ] **Step 1: Write the run script**

```python
# experiments/run_lidar_kitti.py
"""Model C: run our ICP SLAM on real KITTI seq-04 velodyne and report aligned ATE
vs GT poses — an external-validity anchor for the idealized models A/B.

Server-only (needs the fetched data). Throttled:
    nice -n 19 ionice -c3 python experiments/run_lidar_kitti.py
"""
import glob
import json
import numpy as np
from wifi_radar_slam.lidar.kitti import (load_velodyne_scan, load_gt_trajectory,
                                        align_2d_ate)
from wifi_radar_slam.lidar.slam_icp import run_lidar_slam

SEQ = "04"
ROOT = "data/kitti"
DT = 0.1                      # KITTI velodyne is 10 Hz
MAX_FRAMES = 271


def main() -> None:
    files = sorted(glob.glob(f"{ROOT}/sequences/{SEQ}/velodyne/*.bin"))[:MAX_FRAMES]
    scans = [load_velodyne_scan(f) for f in files]
    n = len(scans)
    poses = open(f"{ROOT}/poses/{SEQ}.txt").read()
    calib = open(f"{ROOT}/sequences/{SEQ}/calib.txt").read()
    gt = load_gt_trajectory(poses, calib)[:n]
    # velocity prior from GT (same recipe as the sim runs); final pose is ICP's output
    vel = np.zeros((n, 2))
    vel[1:] = (gt[1:] - gt[:-1]) / DT
    est, _ = run_lidar_slam(scans, vel, DT, np.random.default_rng(0),
                            init_pose=(gt[0, 0], gt[0, 1], 0.0))
    ate = align_2d_ate(est, gt)
    out = {"sequence": SEQ, "frames": n, "aligned_ate_m": ate}
    print(f"[model C] KITTI seq {SEQ}: aligned ATE = {ate:.3f} m over {n} frames")
    with open("data/kitti_results.json", "w") as fh:
        json.dump(out, fh, indent=2)
    print("saved -> data/kitti_results.json")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Parse-check locally**

Run: `python -m py_compile experiments/run_lidar_kitti.py && echo OK`
Expected: `OK`.

- [ ] **Step 3: Run on the server, throttled**

`nice -n 19 ionice -c3 python experiments/run_lidar_kitti.py`
Expected: prints an aligned ATE (m) for seq 04 and writes `data/kitti_results.json`.
Note: KITTI velodyne scans are dense (~120k pts); if ICP is slow, raise the z-band
lower bound or voxel-downsample scans before SLAM (document any such change).

- [ ] **Step 4: Record results and commit**

Add a "Model C (KITTI external validity)" note to `papers/2-wifi-vs-lidar/DOSSIER.md`:
the aligned ATE on real seq-04 LiDAR, framed as the anchor confirming our ICP pipeline
behaves plausibly on real data (context: A/B sim ATE 0.03–0.86 m). Interpret whether
the KITTI ATE lands in a comparable real-LiDAR range.

```bash
git add experiments/run_lidar_kitti.py data/kitti_results.json \
        papers/2-wifi-vs-lidar/DOSSIER.md
git commit -m "paper2(lidar/C): KITTI external-validity ATE (seq 04)"
```

- [ ] **Step 5: Full suite green**

Run: `pytest -q`
Expected: all pass (the 3 new pure KITTI tests included; no new gated tests).

---

## After branch 3

Merge `paper2-lidar-kitti` into `paper2-wifi-vs-lidar`, update the DOSSIER. With A, B,
and C in, **assemble the full WiFi-vs-LiDAR comparison table** — pull the paper-1 WiFi
oracle + realistic joint-MUSIC numbers beside the LiDAR A/B envelope (both scenes),
with the KITTI ATE as the real-LiDAR anchor. That table is paper 2's RQ3 core and the
natural point to tag a first paper-2 milestone. Later sub-projects (fusion, DL
enhancement, cost model, venue) each get their own brainstorming → spec → plan cycle.
