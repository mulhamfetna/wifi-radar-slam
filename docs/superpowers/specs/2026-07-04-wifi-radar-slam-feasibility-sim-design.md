# Design Spec — WiFi-Radar-for-SLAM: Feasibility Simulation (v1)

**Status:** Approved (design), 2026-07-04. **Next:** implementation plan (writing-plans).
**Project:** WiFi as a radar replacement for automotive SLAM.
**This milestone:** a high-fidelity, simulation-only feasibility study for the **sub-7 GHz commodity
WiFi** path. The 60 GHz / mmWave path is deliberately deferred to future development.

Related docs: `literature/00-literature-foundation.md`, `literature/01-detailed-survey-report.md`,
`literature/02-round2-isac-80211bf-mmwave.md`, `literature/references.bib`.

---

## 1. Goal & scope

Demonstrate, in physics-based ray-traced simulation, that a vehicle receiving **ambient sub-7 GHz WiFi**
in an outdoor parking-lot / campus scene can:

1. **build a map** of its static surroundings (parked cars, poles, walls, facades), and
2. **localize itself** along its path (SLAM),

and then **characterize the operating envelope** — the conditions under which this works or fails.

**In scope:** model-based signal processing and SLAM; a single representative scene plus parameter
sweeps; open-source reproducible code.

**Out of scope (YAGNI — named as future work):**
- Real hardware / real CSI capture.
- 60 GHz / mmWave (802.11ad/ay) — the "advanced path," deferred by decision.
- Moving clutter (other vehicles, pedestrians) — scene is static in v1.
- Learned / deep reconstruction (Person-in-WiFi style) — v1 is model-based only.
- Real-time performance.

## 2. Success criteria (staged)

**Phase A — nominal case (proves it works):**
- Reconstruct the parking-lot map and the vehicle trajectory in one representative configuration.
- Metrics: **map error** (Chamfer distance and/or occupancy IoU vs ground-truth layout) and
  **trajectory error** (ATE and RPE vs ground-truth path).
- Optional baseline: dead-reckoning-only trajectory, to show the WiFi map/SLAM adds value.
- Target numbers are set empirically after the first end-to-end runs (recorded in results, not guessed
  here).

**Phase B — operating envelope (proves when it works):**
- Sweep **AP density** (1→4+ APs), **SNR**, **vehicle speed**, and **WiFi bandwidth (20 → 160 MHz)**.
- Deliverable: curves/tables of accuracy vs each parameter, i.e. "works when ___, breaks when ___."
- The **bandwidth sweep** directly exercises the Round-2 finding (range resolution ΔR = c/2B:
  ~3.75 m @40 MHz → ~0.94 m @160 MHz), making the resolution ceiling a headline, quantified result.

## 3. Architecture — six isolated units

Each unit has one purpose, a defined interface, and writes its output to disk so stages run, cache, and
debug independently. Pipeline order: `config → scene → channel → sensing → SLAM → eval`.

### 3.1 Scene builder
- **Purpose:** define the 3D outdoor scene and ground truth.
- **Builds:** road/lot geometry; static targets (parked cars, light poles, walls, building facades);
  ambient AP positions (on facades); the vehicle's ground-truth trajectory and antenna placement.
- **Interface:** `build_scene(config) -> {sionna_scene, trajectory, ap_positions, ground_truth_map}`
- **Depends on:** Sionna RT scene format; config file.

### 3.2 Channel simulator (Sionna RT wrapper)
- **Purpose:** produce the WiFi channel the moving car observes.
- **Does:** for each time step along the trajectory, ray-trace each AP→vehicle-antenna channel
  (reflections/diffractions), obtain the channel impulse response (CIR), convert to frequency-domain
  **CSI** across the WiFi band and subcarriers (configurable 20/40/80/160 MHz), add thermal noise and
  realistic phase effects; include Doppler from vehicle motion.
- **Interface:** `simulate_csi(scene, trajectory, ap_positions, rf_config) -> csi_timeseries`
  (indexed by [time, ap, rx_antenna, subcarrier]).
- **Depends on:** Sionna RT (GPU); scene builder outputs.

### 3.3 Sensing front-end (CSI → detections)
- **Purpose:** turn raw CSI into geometric detections of reflectors in the vehicle frame.
- **Does:** per-frame super-resolution parameter estimation (MUSIC / ESPRIT) for delay, angle-of-arrival,
  and Doppler; separate ego-motion Doppler from the static-scene returns; output candidate reflection
  points (range + bearing) per frame.
- **Interface:** `extract_features(csi_timeseries, rf_config) -> detections_per_frame`
- **Depends on:** channel simulator outputs; RF config (bandwidth, antenna geometry).

### 3.4 SLAM back-end (multipath / virtual-anchor SLAM)
- **Purpose:** jointly estimate the vehicle trajectory and the map from the detections.
- **Does:** treat specular reflections as **virtual anchors** (mirror images of APs — the Channel-SLAM /
  Leitinger formulation); run a Bayesian estimator (particle filter or factor-graph belief propagation)
  to estimate vehicle pose over time and landmark/reflector positions → map.
- **Interface:** `run_slam(detections_per_frame, ap_positions) -> {est_trajectory, est_map}`
- **Data association:** v1 starts with ground-truth-aided association to isolate estimator behavior,
  then relaxes to blind association as a robustness step.
- **Depends on:** sensing front-end outputs.

### 3.5 Evaluator
- **Purpose:** quantify quality vs ground truth and produce paper figures.
- **Does:** map error (Chamfer / occupancy IoU), trajectory error (ATE / RPE); render map overlays and
  error plots.
- **Interface:** `evaluate(est_trajectory, est_map, ground_truth) -> {metrics, figures}`

### 3.6 Experiment runner
- **Purpose:** orchestrate Phase A and Phase B from config; collect results.
- **Does:** run the pipeline for one config (Phase A) or a sweep grid (Phase B); aggregate metrics into
  tables/figures for the paper.
- **Interface:** `run_experiment(config_or_sweep) -> results`

## 4. Data flow & artifacts

```
config.yaml
  → scene/         (scene mesh, trajectory, gt_map)
  → csi/           (csi_timeseries.npz)
  → detections/    (per-frame detections)
  → slam/          (est_trajectory, est_map)
  → results/       (metrics.json, figures/)
```

Intermediate artifacts are cached on disk; a stage can be re-run without recomputing upstream stages
(important because ray tracing is the expensive step).

## 5. Tech stack & repository layout

- **Language/tools:** Python; **Sionna RT** (TensorFlow, GPU — confirmed available); numpy, scipy,
  matplotlib; config via YAML; tests via pytest.
- **Layout:**
  ```
  src/wifi_radar_slam/{scene,channel,sensing,slam,eval,runner}/
  configs/            # scene + sweep definitions
  experiments/        # phase-A, phase-B entry points
  results/            # metrics + figures (git-ignored except summaries)
  tests/              # per-unit tests
  docs/               # this spec, plan, notes
  literature/         # existing verified survey + refs
  ```
- **Open, citable release:** AGPL-3.0-or-later (Mulham's default for new research software), with
  `CITATION.cff` and `.zenodo.json` prepared so a Zenodo DOI can be minted on first release. DOI process
  is activated interactively by the author (GitHub + Zenodo login).

## 6. Risks & mitigations

| Risk | Mitigation |
|------|-----------|
| Per-step ray tracing is slow | Keep the scene modest; batch time steps; cache CIRs; GPU. |
| Super-resolution under-resolves at low bandwidth | Expected — it's part of the Phase-B story; ensure enough antennas/subcarriers for the nominal case. |
| Outdoor SLAM data association is hard | Start ground-truth-aided; relax to blind association as a explicit robustness experiment. |
| Ego-Doppler vs target-Doppler separation | Use known vehicle motion (from trajectory model) to compensate; validate on a single-target sanity scene first. |
| Sionna RT API / mobility learning curve | Build a minimal one-AP, one-target "hello-world" channel before the full scene. |

## 7. Milestone acceptance

v1 is complete when:
1. The full pipeline runs end-to-end on the nominal Phase-A config and produces map + trajectory
   metrics against ground truth.
2. Phase-B sweeps (AP density, SNR, speed, bandwidth) produce the operating-envelope curves, including
   the bandwidth→resolution result.
3. The repository is reproducible (documented run commands, seeded configs) and release-ready
   (license + CITATION.cff + .zenodo.json).
