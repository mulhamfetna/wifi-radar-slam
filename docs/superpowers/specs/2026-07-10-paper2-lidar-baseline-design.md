# Paper 2, sub-project 1 — LiDAR baseline & WiFi-vs-LiDAR comparison substrate

**Date:** 2026-07-10
**Status:** approved (brainstorming), pending execution
**Paper:** 2 — *WiFi sensing as a drop-in LiDAR replacement for SLAM*
(`papers/2-wifi-vs-lidar/DOSSIER.md`)
**Integration branch:** `paper2-wifi-vs-lidar`

## Scope

This spec covers **only** paper 2's first sub-project: standing up a **LiDAR baseline
that runs in the same simulated scenes as the WiFi pipeline**, plus the shared
evaluation substrate that lets WiFi and LiDAR be compared apples-to-apples. It
directly serves paper-2 research question **RQ3** (is WiFi more / equally / less
accurate than LiDAR?) and lays the substrate for **RQ1** (full replacement) and
**RQ4** (fusion).

It is **NOT** the whole paper. Explicit later sub-projects, each its own
brainstorming → spec → plan cycle: **WiFi/LiDAR fusion (RQ4)**, **deep-learning
enhancement (RQ2)**, **cost model (RQ5)**, and **target-venue selection**. Those are
out of scope here.

## Motivation for building three LiDAR models

Rather than argue on paper which LiDAR abstraction is "fair," we build the candidate
models and let results decide, then keep the defensible one as the paper's baseline.
The three are **not** equal-status siblings:

- **A — idealized geometric LiDAR** and **B — Sionna optical-ray LiDAR** both produce
  point clouds **in our two scenes against the same footprint ground truth**. They are
  head-to-head baselines and both can later feed fusion.
- **C — real LiDAR (KITTI)** cannot share our scene geometry or GT and cannot fuse
  with our WiFi sim. It is an **external-validity cross-check** — does our idealized
  LiDAR + ICP SLAM behave like real LiDAR on a standard benchmark? — not a third rival
  baseline. Its deliverable is a validation figure that anchors A/B to reality.

## The comparison plane: 2D BEV

The existing pipeline is **2D**: trajectories are `xy`, maps are bird's-eye footprint
point sets, and every metric in `eval/metrics.py` (ATE, RPE, Chamfer, map-accuracy,
map-completeness, occupancy-IoU) operates in `xy`. To compare directly, the LiDAR side
**reduces to the same 2D BEV plane**: we simulate/consume a horizontal LiDAR ring (or
project the 3D cloud to the ground plane), so LiDAR reports the *identical* six
metrics on the *identical* ground truth. This is stated as an explicit, documented
comparison choice, not hidden.

## Architecture

New shared package `src/wifi_radar_slam/lidar/` (additive; available to both papers,
does not touch paper-1 content):

```
lidar/
  __init__.py
  config.py        # LidarConfig: angular_res_deg, range_sigma_m, max_range_m,
                   #   fov_deg, min_range_m, scan plane; presets pinned to a datasheet
  pointcloud.py    # Scan/PointCloud container (2D BEV points + per-return metadata)
  sensor_geo.py    # Model A: geometric ray-cast against scene meshes -> Scan
  sensor_sionna.py # Model B: Sionna RT EM paths reused as an optical-return proxy
  slam_icp.py      # point-cloud -> ICP/scan-match SLAM -> (trajectory, aggregated map)
  runner.py        # scene + LidarConfig + sensor -> per-pose scans -> SLAM -> metrics
  kitti.py         # Model C: KITTI odometry ingest -> Scan stream (external validity)
```

Metrics are **reused unchanged** from `eval/metrics.py` — the LiDAR runner emits the
same result dict shape as the WiFi runner so a single comparison table can hold both.

### Data flow (per scene)

```
scene meshes (shared Sionna/Mitsuba scene, footprint GT)
      │
      ├── Model A: geometric ray-cast (Mitsuba ray_intersect, pure geometry, no EM)
      ├── Model B: Sionna RT paths reinterpreted as optical returns
      │          (both -> 2D BEV Scan per GT pose, with LidarConfig noise/occlusion)
      ▼
  ICP / scan-match SLAM  ─────────────►  estimated trajectory + aggregated 2D map
      │
      ▼
  eval/metrics.py (ATE, RPE, Chamfer, map-acc, map-completeness, IoU)
      │
      ▼
  comparison table:  WiFi (oracle | realistic joint-MUSIC)  vs  LiDAR-A  vs  LiDAR-B
                     (both scenes)         + KITTI external-validity row for A
```

## Branch sequence (each off `paper2-wifi-vs-lidar`, merged back on completion)

| Order | Branch | Builds | Deliverable merged back |
|-------|--------|--------|--------------------------|
| 0 | `paper2-lidar-harness` | `lidar/` skeleton: `config`, `pointcloud`, `slam_icp`, `runner`; metrics wiring; one datasheet-pinned `LidarConfig` preset; unit tests on synthetic clouds | Comparison substrate all models reuse |
| 1 | `paper2-lidar-geo` (A) | `sensor_geo.py` geometric ray-cast in both scenes | LiDAR baseline #1 numbers (6 metrics × 2 scenes) |
| 2 | `paper2-lidar-sionna` (B) | `sensor_sionna.py` optical-ray proxy | LiDAR baseline #2 numbers; A-vs-B fidelity note |
| 3 | `paper2-lidar-kitti` (C) | `kitti.py` + external-validity run | Cross-validation figure anchoring A/B to real LiDAR |

Each branch: implement → evaluate on both scenes → record results in the paper-2
DOSSIER and `docs/results-v1.md` → merge to `paper2-wifi-vs-lidar`. Runs are cheap
locally (A is pure geometry, no Sionna); B's ray tracing and any parameter sweeps can
run on the amd server throttled (`nice -n 19 ionice -c3`) if needed. **No mid-stream
Zenodo release** — paper 2 develops openly and tags only at a real milestone (e.g. the
first complete WiFi-vs-LiDAR comparison).

## Error handling / honesty guards

- **Don't rig the baseline.** At automotive-datasheet parameters LiDAR will likely beat
  passive WiFi on mapping; that gap (traded against the cost gap) is part of the paper's
  story, not something to tune away. The ICP baseline uses standard, documented params.
- **ICP divergence:** naive scan-to-scan ICP can drift/diverge. Baseline uses
  scan-to-map accumulation with a documented outlier reject; if it still diverges on a
  scene, that is reported, not hidden.
- **Degenerate scans** (empty return, all-occluded) return an empty `Scan`; SLAM skips
  the update and holds the motion prediction, matching how the WiFi front-end handles
  no-detection frames.

## Testing

- Pure-Python unit tests (no Sionna) for `config`, `pointcloud`, `slam_icp` on
  synthetic point clouds with known transforms (recovered pose within tolerance),
  mirroring the existing test style. These run in the normal `pytest` set.
- Sensor models A/B and the scene runs are gated like the existing Sionna smoke tests
  (not in the default fast test set).
- The full existing test suite must still pass after each merge.

## Non-goals

- No WiFi/LiDAR fusion, no deep-learning enhancement, no cost model, no venue choice —
  each is its own later sub-project.
- No change to paper-1 *content*; no new repository; no code fork.
- Not producing the final paper-2 comparison narrative here — only the baseline numbers
  and the substrate that generates them.

## Acceptance

- `src/wifi_radar_slam/lidar/` exists; `slam_icp` recovers a known synthetic trajectory
  within tolerance; pure-Python LiDAR tests pass in the default suite.
- Models A and B each produce 2D BEV scans in both scenes and emit all six metrics in
  the same result shape as the WiFi runner.
- A single comparison table (both scenes) holds WiFi (oracle + realistic) alongside
  LiDAR-A and LiDAR-B; KITTI provides an external-validity row/figure for A.
- Each of branches 0–3 merges cleanly back to `paper2-wifi-vs-lidar`; the full existing
  test suite still passes; results recorded in the paper-2 DOSSIER.
