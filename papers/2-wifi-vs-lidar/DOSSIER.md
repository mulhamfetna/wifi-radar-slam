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

## Literature & novelty (deep-research 2026-07-11)
Full synthesis: **`../../docs/literature-paper2.md`** (5 angles → 20 sources → 25
adversarially-verified claims; cost data sourced but not vote-verified).

**Novelty gap (high confidence):** no published work demonstrates commodity-CSI WiFi as
a *validated drop-in LiDAR replacement* for **on-vehicle/outdoor automotive SLAM** with a
head-to-head WiFi-vs-LiDAR accuracy comparison. Nearest: **P2SLAM** (standalone WiFi-CSI
SLAM but indoor, vs *visual* SLAM) and **radio-fingerprint SLAM** (outdoor/on-vehicle but
RSS-not-CSI, slow UGV, best accuracy needs LiDAR fusion). All other WiFi-in-SLAM work uses
WiFi to *augment* camera/LiDAR. Paper 2 occupies the open cell.

RQ anchors from the review: **RQ4 fusion** — prior work shows fusion > single modality
(WiFi-only 2.7 m → 0.88 m fused; EKF fusion 0.24–0.38 m vs WiFi 1.34 m vs LiDAR 0.62–2.88 m).
**RQ2 DL** — precedent for RF→geometry (transformer CSI→3D point cloud; U-Net/ViT RF→outdoor
geometry; RF-Pose through-wall). **RQ5 cost** — the WiFi-vs-LiDAR cost comparison itself
appears novel; sourced prices: WiFi-CSI RX $5–15 (ESP32) / $35–75 (Pi+nexmon) vs LiDAR
~$99 (RPLIDAR A1) / $200–600 (solid-state) / ~$75–80 k (legacy spinning).

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
- **Branch 3 — `paper2-lidar-kitti` (model C) — DONE (results in), pending merge.**
  Real-LiDAR external-validity anchor: our shared ICP SLAM run on **KITTI odometry
  seq 04** (271 frames, 394 m). Pure loaders (`kitti.py`: velodyne `.bin` → 2D BEV Scan
  via z-slice; poses + `Tr` calib → BEV `(x,z)` GT; 2D rigid alignment) test locally;
  `fetch_kitti.py` pulls only seq 04 (~0.5 GB) from KITTI's public S3 via `remotezip`
  range requests (avoids the 84 GB full-velodyne zip); `data/kitti/` is gitignored.
  Plan: `../../docs/superpowers/plans/2026-07-10-paper2-lidar-kitti-modelC.md`.

  **Model-C result** (amd server, 9.5 s; `data/kitti_results.json`):
  **RPE = 0.154 m/frame, aligned ATE = 1.16 m over 271 frames / 394 m ≈ 0.3 % drift.**
  This is real-LiDAR-plausible (SOTA KITTI odometry is ~0.1–0.5 % drift), confirming our
  ICP+metrics back-end behaves like real LiDAR — which validates the A/B sim results are
  produced by a sound pipeline, not an artefact of the simulator. (An early attempt gave
  ATE 36 m: the GT-differenced velocity prior was in the KITTI camera `(x,z)` frame,
  mismatched to the velodyne frame the SLAM runs in; the frame-agnostic **adaptive
  constant-velocity** motion model fixed it — see shared-code note below.)

### Shared-code improvements landed with C (benefit A/B too; results unchanged)
- **KD-tree ICP** (`slam_icp.icp_align`): brute-force O(scan·map) nearest-neighbour →
  `scipy.spatial.cKDTree` queried at `workers=-1` (all cores). **Exact NN**, so A/B/C
  metrics are unchanged, but the full KITTI run went from stalling >1 h (and OOM-risk on
  the growing map) to **9.5 s**. scipy is already a core dependency.
- **Adaptive constant-velocity motion model** (`run_lidar_slam(velocity=None)`): predict
  each ICP init from the previous *estimated* motion — frame-agnostic (correct even when
  GT lives in a different frame). Sim runs keep the explicit-velocity path (unaffected).
- **Progress logging**: `run_lidar_slam` takes a `progress(f, n, n_scan, n_map)` callback;
  the KITTI runner logs per-frame frame/scan/map + live ETA and loads scans across all
  cores (`multiprocessing.Pool`). No more blind waiting.

## WiFi-vs-LiDAR comparison (RQ3) — DONE
Full head-to-head table (both scenes, six metrics, same GT): **`../../docs/results-paper2.md`**.
Milestone tagged **`paper2-v0.1.0`**.

Emerging answer (RQ1): **WiFi is a viable drop-in for localization/odometry** — realistic
commodity-CSI WiFi (joint 2-D MUSIC) localizes to **2.7 cm ATE** on the controlled scene,
*better* than both LiDAR models (A 0.102 m, B 0.483 m), and ~9 cm on the street. **Mapping
is where LiDAR still dominates** — LiDAR-B reaches IoU 1.0 (controlled) / completeness 5.3 m
(street), while WiFi mapping is coverage-bounded (realistic IoU ≈ 0). So WiFi replaces LiDAR
for the trajectory half at a fraction of the cost (RQ5, TBD); the mapping half needs
enhancement — fusion (RQ4), deep learning (RQ2), or multi-pass. Real-LiDAR anchor: KITTI
seq-04 aligned ATE 1.16 m / 394 m (~0.3 % drift).

## Cost model (RQ5) — DONE
Spec: `../../docs/superpowers/specs/2026-07-11-paper2-cost-model-design.md`; results in
`../../docs/results-paper2.md` ("Cost (RQ5)"); data `data/cost_data.yaml` (sourced prices,
each citation + date) → `data/cost_results.json`. Milestone tagged **`paper2-v0.2.0`**.

**Headline.** WiFi package (ambient-free: Pi4+nexmon + antennas) **$40–95** vs the
**Ouster OS1 our A/B models actually simulate ($8–24 k) = 84–600× cheaper**; ~800–2000×
vs legacy spinning. **Localization value** (price × ATE): WiFi **1–9 $·m** vs LiDAR
**212–2448 $·m** — *two to three orders of magnitude better accuracy per dollar*.

**The decisive asymmetry.** **WiFi cannot buy map coverage at any price** — realistic-CSI
IoU ≈ 0, so $/IoU is **infinite**, while LiDAR converts money into coverage
(8 k–147 k $/IoU). This is the honest boundary of the drop-in-replacement claim and the
direct motivation for RQ2 (DL) and RQ4 (fusion).

**Honest caveats (in the docs).** The cost advantage is *not* unconditional: vs the
cheapest emerging automotive solid-state ($100–200) the gap narrows to **1.1–5×**, and with
**3 self-deployed APs** WiFi can be **more expensive** (0.3–1.5×). The dramatic story
depends on the **ambient-AP premise** holding. LiDAR rows are priced at the OS1 tier
(the sensor whose params produced our measured accuracy) — pricing OS1-grade accuracy at a
$150 solid-state tier we never simulated would be apples-to-oranges. The budget 2D scanner
is a price floor, not a peer.

## Fusion (RQ4) — DONE
Spec: `../../docs/superpowers/specs/2026-07-11-paper2-fusion-design.md`; results in
`../../docs/results-paper2.md` ("Fusion (RQ4)") + `data/fusion_results.json`.
Code: `src/wifi_radar_slam/fusion.py` (tight PF + loose baseline);
`configs/street_metal_music.yaml` (new realistic-WiFi street config).

**Symmetric fusion, not the literature's shape.** Prior work demotes WiFi to loop closure;
our RQ3 shows WiFi is the better *localizer*, so we fused symmetrically:
`weight = w_wifi(bistatic) × w_lidar(scan-match)`, output map = union.

**Answer: fusion helps CONDITIONALLY — the condition is sensor parity.** Tight fusion beats
**both** solo modalities in **3 of 4** configs (controlled/B **0.044** vs WiFi 0.081 / LiDAR
0.212; street/B **0.175** vs 0.281 / 0.844 — both ~79 % better than the LiDAR). But it
**degrades the stronger sensor under large accuracy mismatch**: street/A, LiDAR alone 0.027 →
fused 0.218 (**8× regression**), because equal-confidence weighting lets a 10×-worse WiFi pull
the filter off LiDAR's good solution. Reported rather than hidden by tuning `sigma`; the
honest fix — **confidence-adaptive weighting** — is future work.

**Mapping.** The union **adds coverage where LiDAR is incomplete** (street IoU 0.149→0.177 A,
0.262→0.309 B) but **slightly pollutes an already-good map** (controlled 0.977/1.0 → 0.913;
map-acc 0.25→0.33): noisy realistic-CSI reflectors fill gaps, they don't sharpen detail.

**Cost verdict.** WiFi is a **+0.2–1.2 %** addition to an OS1-class LiDAR, so **when fusion
helps it is essentially free** (~4.8× better $·m value; 36–79 % ATE gain for ~0.5 % more
money) — the strongest practical case in the paper for a **hybrid** rather than a
replacement. When it hurts, no price justifies it.

## Learned enhancement (RQ2) — DONE (decisive NEGATIVE result + a correction to paper 1)
Spec: `../../docs/superpowers/specs/2026-07-11-paper2-learned-map-filter-design.md`;
results in `../../docs/results-paper2.md` ("Enhancement (RQ2)");
code `src/wifi_radar_slam/map_filter.py` + `run_slam(map_filter=...)` hook.

**Answer: NO — a learned discriminator cannot close the WiFi mapping gap, because the gap is
not a discrimination problem.** Every ladder rung (none / heuristic / RandomForest / MLP)
leaves **IoU 0.000** on both scenes; the learned rungs reject everything and **empty the map**.

**Why — the floor has THREE components** (`experiments/isolate_mapping_floor.py`,
`data/mapping_floor_isolation.json`). A first diagnostic (oracle vs MUSIC params) conflated
two causes; the isolation experiment matches every MUSIC detection to its nearest **true**
Sionna path and, for genuine facade matches, triangulates the **same path** with true vs
MUSIC parameters — so any degradation is **pure estimation error, discrimination held
perfect**:

1. **Phantom detections (~89 %, DOMINANT).** 89.2 % (controlled) / 89.5 % (street) of MUSIC
   detections match **no real propagation path at all** — estimator artefacts. *You cannot
   discriminate among real paths when most detections are not real paths.* Neither paper 1
   nor our first analysis identified this.
2. **Estimation bias.** Controlled: a **6.45 m median range bias** ruins even correctly
   identified facade paths (true params 100 % within 1 m → MUSIC params **2.4 %**). That
   dwarfs the 0.94 m resolution limit at 160 MHz — a *bias*, not a resolution bound. Street:
   estimates of genuine facade paths are usable (**76.7 %** within 1 m).
3. **Discrimination failures (2–8 %).** Paper 1's mechanism — real, but the **smallest**.

**RQ2 answer.** A filter *selects*; it can neither *invent* the real paths 89 % of detections
lack, nor *correct* the range bias. The usable 2–9 % also proved **not separable** from the
phantom majority using MUSIC-observable features. Hence every rung failed.

**Refinement/correction to paper 1.** Its inference "discrimination is learnable (F1 ≈ 0.9)
⇒ mapping is fixable" **does not hold**: (a) discrimination is the *smallest* of the three
mechanisms; (b) the F1 used **`elevation`**, unmeasurable by a single-ULA 2-D front-end —
corrected F1 on observable features is **0.00–0.45** held-out, **0.00–0.20** cross-scene;
(c) it classified **true paths**, not **MUSIC detections**. Paper 1's *empirical* results
(oracle map, the 60 GHz/aperture null result) **stand** — this is a refinement of its
*interpretation*. Paper 1 is submitted and frozen: **fold this into its revision** when
reviews arrive (user decision), and cite the corrected version from paper 2. Do NOT edit the
frozen submission.

**Silver lining.** The street's 76.7 % (correctly matched facade paths triangulate within
1 m) shows the geometry **is** recoverable if phantoms and bias are fixed — the ceiling is set
by the **front-end**, not the physics. That is where a learned method must act.

**What it does not show.** Not that DL cannot help — only that **classification-based
filtering of estimated paths** cannot. The correct formulation must bypass/repair the
estimation stage (end-to-end CSI→geometry; literature precedent in `docs/literature-paper2.md`).
Future work, not claimed.

## Status: all 5 research questions answered
RQ1 (drop-in?) · RQ2 (DL enhancement) · RQ3 (accuracy) · RQ4 (fusion) · RQ5 (cost).

**Thesis.** Ambient WiFi is a viable **drop-in replacement for LiDAR *localization*** — it
matches or beats LiDAR at **84–600× lower cost** — but **not for mapping**, and the mapping
gap is **not cheaply patchable** (RQ2). The practical recommendation is therefore the
**hybrid**: adding WiFi to an existing LiDAR costs **~0.5 % more** and improves localization
**36–79 %** (RQ4), with the LiDAR supplying the coverage WiFi cannot.

## Manuscript — DRAFTED (`paper2-v0.5.0`)
**Title:** *Can Ambient WiFi Replace LiDAR for Automotive SLAM? Localization Yes, Mapping
No — and Why*. Target: **IEEE IoT-J**. Files: `main.tex`, `main.pdf`, `refs.bib`,
`README.md` (data-provenance table). Spec:
`../../docs/superpowers/specs/2026-07-11-paper2-manuscript-design.md`.

**Builds clean:** 7 pages, **0 undefined references/citations, 0 bibtex errors**
(`pdflatex; bibtex; pdflatex ×2`; IEEEtran vendored, no siunitx).

**Figures.** Figs. 2–6 are generated by `experiments/make_paper2_figures.py` **from the
committed JSONs** — no hand-typed numbers — and are **byte-reproducible** (CreationDate
stripped), so a test run leaves the tree clean. `tests/test_paper2_figures.py` asserts the
figure inputs equal the artifact contents, so a figure cannot silently drift from the data.
Palette Okabe-Ito (CVD-validated); hatching for greyscale print; never a dual axis.

**Bibliography verified against primary sources** (arXiv abstract pages, the RA-L PDF) —
this caught that **P2SLAM is IEEE RA-L, not T-RO**. Also fixed an inherited defect in
paper-1's `refs.bib`: `%` comments containing `@` (e.g. `@40 MHz`), which bibtex parses as
entry types (3 errors) — **fold into paper-1's revision**.

## Next step
**Submission package** (own cycle, mirroring paper 1): cover letter / editor comments,
supplementary material, keywords, IoT-J topic taxonomy, and a Zenodo release. Do not start
before design approval.

Open items to settle before submission:
- Re-check the `verified: false` prices in `data/cost_data.yaml` against primary vendor pages.
- Paper-1 revision: fold in the RQ2 refinement (discrimination is the smallest of three
  mechanisms; the `elevation` oracle feature) and the `refs.bib` `@`-in-comment fix.

## Do-not-mix reminders
- Paper 1 is frozen (`v0.7.1` / `paper1-submitted`); do not alter its *content* when
  evolving shared code for paper 2.
- Keep paper-2 Claude-memory notes in `paper2-*` files (see `MEMORY.md`).
