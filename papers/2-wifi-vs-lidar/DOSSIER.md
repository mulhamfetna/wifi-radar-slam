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
- **Branch 1 — `paper2-lidar-geo` (model A)** — next. Mesh ray-cast sensor against the
  Sionna/Mitsuba scene meshes, plugged into the `make_sensor` seam.
- **Branch 2 — `paper2-lidar-sionna` (model B)** — Sionna optical-ray proxy.
- **Branch 3 — `paper2-lidar-kitti` (model C)** — KITTI ingest + external-validity run.

## Next step
Write branch 1's plan (model A — geometric mesh ray-cast) against the `make_sensor`
seam and `run_lidar` runner from branch 0, then implement. Later sub-projects (fusion,
DL enhancement, cost model, venue) each get their own brainstorming → spec → plan
cycle; do not start them before design approval.

## Do-not-mix reminders
- Paper 1 is frozen (`v0.7.1` / `paper1-submitted`); do not alter its *content* when
  evolving shared code for paper 2.
- Keep paper-2 Claude-memory notes in `paper2-*` files (see `MEMORY.md`).
