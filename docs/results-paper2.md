# Paper 2 results — WiFi vs LiDAR for SLAM

Paper 2 asks whether ambient-WiFi sensing can be a **drop-in LiDAR replacement** for
SLAM. This document collects the head-to-head comparison on the **same two simulated
scenes**, with the **same six metrics** (the shared `eval/metrics.py`), against the
**same footprint ground truth**. WiFi rows are the authoritative numbers from paper 1
(submitted to IEEE IoT-J, frozen at `v0.7.1`); LiDAR rows are paper 2's models A/B
(`data/lidar_geo_results.json`, `data/lidar_sionna_results.json`); the real-LiDAR
anchor is KITTI seq-04 (`data/kitti_results.json`).

Metrics (all 2-D BEV, metres except IoU): **ATE** trajectory error, **RPE** per-frame
relative error, **Chamfer** symmetric map error, **map-acc** est→GT precision,
**map-compl** GT→est coverage, **IoU** occupancy overlap. Lower is better except IoU.

## Comparison table

### Scene 1 — `controlled_wall` (clean single-reflector geometry)

| Sensor model | ATE | RPE | Chamfer | map-acc | map-compl | IoU |
|--------------|----:|----:|--------:|--------:|----------:|----:|
| WiFi — oracle (Sionna true paths)     | 0.045 | 0.007 | 0.51  | 0.25 | 0.77  | 0.79 |
| WiFi — realistic (joint 2-D MUSIC, CSI) | 0.027 | —   | 4.1   | 4.8  | 3.5   | ~0   |
| LiDAR-A — geometric (bbox ray-cast)   | 0.102 | 0.030 | 0.209 | 0.250 | 0.168 | 0.977 |
| LiDAR-B — Sionna optical (diffuse EM)  | 0.483 | 0.055 | 0.187 | 0.251 | 0.123 | 1.000 |

### Scene 2 — `street_canyon_metal` (reflective street canyon + cars)

| Sensor model | ATE | RPE | Chamfer | map-acc | map-compl | IoU |
|--------------|----:|----:|--------:|--------:|----------:|----:|
| WiFi — oracle                         | 0.116 | 0.007 | 12.3  | 0.30  | 24.4   | 0.077 |
| WiFi — realistic (commodity CSI)      | ~0.09 | —   | *bounded* | — | — | ~0 |
| LiDAR-A — geometric                   | 0.026 | 0.017 | 8.674 | 0.251 | 17.097 | 0.163 |
| LiDAR-B — Sionna optical              | 0.857 | 0.117 | 3.734 | 2.125 | 5.344  | 0.261 |

### Real-LiDAR external-validity anchor (different world; ATE only)

KITTI odometry **seq 04** (271 frames, 394 m) through the *same* ICP SLAM back-end:
**RPE 0.154 m/frame, aligned ATE 1.16 m ≈ 0.3 % drift** — real-LiDAR-plausible (SOTA
KITTI odometry ~0.1–0.5 %), confirming the LiDAR back-end that produces the A/B rows is
sound, not a simulator artefact.

## Reading the table (RQ3, and the emerging RQ1 answer)

**Localization — WiFi is competitive with, even better than, LiDAR.** Realistic
commodity-CSI WiFi with joint 2-D MUSIC localizes to **2.7 cm ATE** on the controlled
scene — better than both LiDAR models there (A 0.102 m, B 0.483 m) — and to ~9 cm on the
street, comparable to LiDAR-A's 2.6 cm. For the **trajectory/odometry** half of SLAM,
WiFi is a viable drop-in.

**Mapping — LiDAR dominates coverage; WiFi is coverage-bounded.** LiDAR-B reaches
**IoU 1.0** on the controlled wall and **5.3 m completeness** on the street; WiFi mapping
is strong only under oracle sensing on the clean wall (IoU 0.79) and collapses under
realistic CSI (IoU ≈ 0) and on the street (a single passive pass illuminates only a
subset of facades; commodity-CSI path discrimination floors map accuracy — paper 1's
thesis). For the **mapping** half, WiFi does **not** yet replace LiDAR.

**The two LiDAR models bracket reality.** A (geometric) is precise and localizes well but
low-coverage; B (diffuse physics) is dense-coverage but noisier and drifts more. A real
LiDAR sits between — so LiDAR is reported as an **A/B envelope**, not a single baseline.

**Emerging answer (RQ1):** ambient WiFi can drop-in replace LiDAR for **localization /
odometry** at a fraction of the cost (RQ5, to quantify), but **mapping** needs
enhancement — multi-pass accumulation, WiFi+LiDAR fusion (RQ4), or deep-learning
reconstruction (RQ2) — to approach LiDAR coverage. This scopes the next sub-projects.

## Fairness caveats

- **Sensor-model comparison, not a hardware benchmark.** WiFi-oracle and LiDAR-A/B are
  idealized *sensor models* on identical scenes/GT; WiFi-realistic is commodity CSI.
  LiDAR is at automotive-datasheet parameters (`OUSTER_OS1`: 120 m / ±3 cm / 360°).
- **Same GT for both**, so map-completeness penalizes *any* sensor for un-illuminated /
  occluded / sub-ring-height surfaces equally — the coverage gap is real, not a WiFi-only
  artefact (LiDAR-A shows the same on the street: completeness 17 m).
- WiFi rows are frozen paper-1 results (`docs/results-v1.md`); "—"/"bounded" mark
  metrics paper 1 did not tabulate for that scene/mode (realistic street mapping is the
  hard case paper 1 characterized qualitatively).
