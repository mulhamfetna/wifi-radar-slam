# Paper 2 — Dossier (stub)

**Working title:** *WiFi Sensing as a Drop-in LiDAR Replacement for SLAM*
**Author:** Mulham Fetna (ORCID 0009-0006-4432-798X)
**Status:** **ACTIVE — just started.** Branch `paper2-wifi-vs-lidar` (developed
openly, merged to `main` as it matures). No experiments designed yet.

This dossier is paper 2's durable record. Update it as work proceeds.

## Premise
The original intent behind this project was a **LiDAR** replacement; paper 1
(radar-framing feasibility, submitted to IoT-J — see `../1-wifi-radar-slam/DOSSIER.md`)
established that ambient WiFi is a viable SLAM sensing modality with a clear
localization/mapping profile. Paper 2 takes the direct step: **can WiFi sensing be a
drop-in replacement for LiDAR in SLAM?** The headline motivation is **cost** — a full
WiFi sensing package is far cheaper than a single LiDAR unit.

## Research questions (to refine in the paper-2 brainstorming cycle)
1. Can ambient WiFi sensing **efficiently and fully replace LiDAR** for SLAM?
2. Is **pure WiFi** enough, or is **deep-learning enhancement** needed to reach
   LiDAR-equivalent results?
3. Is WiFi **more / equally / less accurate** than LiDAR as a replacement?
4. Does running **WiFi + LiDAR side by side (fusion)** improve efficiency/accuracy
   **significantly, marginally, or not at all**?
5. Quantify the **cost/efficiency** advantage (WiFi package vs one LiDAR) as the
   central value proposition.

## Relationship to paper 1 / shared code
Extends the shared `wifi_radar_slam` pipeline in `../../src/` (sensing → mapping →
SLAM → metrics → WiFiSLAM-Sim dataset → learned discriminator). New, additive shared
modules expected: a **LiDAR baseline** (point-cloud SLAM in the same simulated
scenes), a **WiFi/LiDAR fusion** path, and a **cost model**. This is an extension, not
replication: same substrate, new comparative + cost + fusion research questions.

## Progress

### Sub-project 1 — LiDAR baseline & comparison substrate (in progress)
Design: `../../docs/superpowers/specs/2026-07-10-paper2-lidar-baseline-design.md`.
Decision: build three LiDAR models, each on its own branch, and let results decide the
defensible baseline. **A** (geometric mesh ray-cast) and **B** (Sionna optical rays)
are head-to-head in-scene baselines; **C** (KITTI real LiDAR) is an external-validity
cross-check, not a rival. Comparison plane is **2D BEV** — LiDAR reuses the WiFi
metrics unchanged.

Branch sequence off `paper2-wifi-vs-lidar`:
- **Branch 0 — `paper2-lidar-harness` — DONE, merged.** Shared substrate
  `src/wifi_radar_slam/lidar/`: `LidarConfig` (+ `OUSTER_OS1` datasheet preset,
  120 m / ±3 cm / 360°), `Scan` container, point-to-point 2D `icp_align`, scan-to-map
  `run_lidar_slam` → `(traj, map)`, `ReferenceSensor` (analytic ray-cast) + the
  `make_sensor(built, cfg, rng) -> (pose -> Scan)` seam, and `run_lidar` emitting the
  six WiFi-comparable metrics (ATE, RPE, Chamfer, map-acc, map-completeness, IoU).
  NumPy-only, no Sionna. 11 new tests (45 pass / 2 Sionna skips). Plan:
  `../../docs/superpowers/plans/2026-07-10-paper2-lidar-harness.md`.
- **Branch 1 — `paper2-lidar-geo` (model A) — DONE, merged.**
  Geometric 2D LiDAR: each non-floor object's bounding box is sliced at the scan
  plane (`z = RX_HEIGHT_M = 1.5 m`) into wall segments (`mesh_slice.py`), then ray-cast
  in pure NumPy with occlusion-correct nearest-hit (`sensor_geo.py`); `geo_sensor` is the
  `make_sensor` factory. Ray math unit-tested locally; scene run is Sionna-gated
  (`tests/test_lidar_geo_scene.py`, passes on the amd server). Fidelity: bbox segments
  now (box-faithful, car-approximate); triangle-exact Mitsuba `ray_intersect` deferred.
  Plan: `../../docs/superpowers/plans/2026-07-10-paper2-lidar-geo-modelA.md`.

  **Model-A results** (`OUSTER_OS1`: 120 m / ±3 cm / 360°, ~1029 beams; amd server,
  23 s throttled; `data/lidar_geo_results.json`):

  | Scene | ATE (m) | RPE (m) | Chamfer (m) | map-acc (m) | map-complete (m) | IoU |
  |-------|--------:|--------:|------------:|------------:|-----------------:|----:|
  | controlled_wall | 0.102 | 0.030 | 0.209 | 0.250 | 0.168 | 0.977 |
  | street_canyon   | 0.026 | 0.017 | 8.674 | 0.251 | 17.097 | 0.163 |

  Reading: LiDAR **localization is excellent** (2.6 cm ATE on the street via ICP) and on
  the clean wall the **map is near-perfect** (IoU 0.98). On the street, **map-accuracy
  stays good (0.25 m)** — the points it maps are correct — but **map-completeness is
  poor (17 m)**: a single horizontal ring at 1.5 m, scored against the *full-footprint*
  GT (which includes sub-ring-height cars and occluded/backside facades the ring can
  never see), leaves much of the GT uncovered. This is a fair modeling artifact, not a
  bug — and it is scored on the **same GT** the WiFi side uses, so the comparison stays
  apples-to-apples. Caveat to revisit for the paper: completeness for a single-ring
  LiDAR vs full-footprint GT is coverage-bounded (options: multi-height rings, or
  restricting GT to z-visible objects). WiFi oracle/realistic rows will be placed
  beside these once the comparison table is assembled.
- **Branch 2 — `paper2-lidar-sionna` (model B) — DONE (results in), pending merge.**
  Sionna optical-ray proxy: the LiDAR is a **monostatic node** (a `lidar_tx`
  transmitter co-located with the vehicle RX) and scene materials are given
  `scattering_coefficient = 0.7` with `diffuse_reflection=True`, so facades backscatter
  toward the sensor. Single-scatter valid non-floor paths' **interaction vertices** are
  the returns (`sensor_sionna.py`); pure helpers `vertices_to_scan` / `_voxel_downsample`
  (range filter, radial noise, world→local, density cap) test locally, the PathSolver
  sensor is Sionna-gated (`tests/test_lidar_sionna_scene.py`, passes on amd). Config note:
  model B ignores beam params (it is ray-traced) but honors min/max-range + range-sigma.
  Physics validated by a server spike (specular monostatic ≈ 0 returns vs diffuse 8417).
  Plan: `../../docs/superpowers/plans/2026-07-10-paper2-lidar-sionna-modelB.md`.

  **Model-B results** (`OUSTER_OS1`, `WRS_NUM_SAMPLES=1e6`, `max_depth=2`; amd server,
  5.4 min throttled; `data/lidar_sionna_results.json`):

  | Scene | ATE (m) | RPE (m) | Chamfer (m) | map-acc (m) | map-complete (m) | IoU |
  |-------|--------:|--------:|------------:|------------:|-----------------:|----:|
  | controlled_wall | 0.483 | 0.055 | 0.187 | 0.251 | 0.123 | 1.000 |
  | street_canyon   | 0.857 | 0.117 | 3.734 | 2.125 | 5.344 | 0.261 |

  **A-vs-B (the two models bracket reality):** model A (geometric bbox ray-cast) gives
  **crisp, precise points and excellent localization** (street ATE 0.026 m, map-acc
  0.25 m) but **poor coverage** (street completeness 17 m, IoU 0.16). Model B (diffuse
  EM physics) gives **dense coverage** (controlled IoU 1.0; street completeness 5.3 m,
  chamfer 3.7 m — both far better than A) but **noisier points** (street map-acc 2.1 m)
  and **worse localization** (street ATE 0.86 m), because diffuse returns scatter off
  many vertices and blur ICP correspondences. Interpretation for the paper: the
  geometric abstraction is optimistic on precision/odometry and pessimistic on coverage;
  the physics proxy is the opposite. A real LiDAR sits between them — so the WiFi-vs-LiDAR
  comparison should report **both** as the LiDAR envelope, not a single baseline.
- **Branch 3 — `paper2-lidar-kitti` (model C)** — KITTI ingest + external-validity run.

## Next step
Branch 3 (`paper2-lidar-kitti`, model C — KITTI external validity) against the shared
ICP SLAM: ingest a KITTI odometry sequence, run our `run_lidar_slam` on real LiDAR
scans, and compare ATE against KITTI ground-truth poses — a cross-check that our
idealized models (A/B) behave like real LiDAR. C is NOT an in-scene rival baseline
(different world, no shared GT, no fusion). After C, assemble the full WiFi-vs-LiDAR
comparison table (place the WiFi oracle/realistic rows beside the A/B LiDAR envelope).
Later sub-projects (fusion, DL enhancement, cost model, venue) each get their own
brainstorming → spec → plan cycle; do not start them before design approval.

## Do-not-mix reminders
- Paper 1 is frozen (`v0.7.1` / `paper1-submitted`); do not alter its *content* when
  evolving shared code for paper 2.
- Keep paper-2 Claude-memory notes in `paper2-*` files (see `MEMORY.md`).
