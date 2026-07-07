# Running the WiFi-Radar-for-SLAM simulation

## Environments

Two environments, by design:

| Environment | What runs | Needs |
|-------------|-----------|-------|
| **Dev (any machine)** | All pure-Python stages + their tests; the end-to-end runner test (Sionna stages monkeypatched) | Python ≥ 3.11, no GPU |
| **GPU box** | The real Sionna RT scene + channel stages; the actual Phase-A/Phase-B experiments | Python 3.11, CUDA GPU, `sionna` + `tensorflow` |

The pure-Python stages import Sionna **lazily**, so everything except the two `*_smoke` tests and the real experiments runs without Sionna installed.

## Dev setup (no GPU)

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e ".[dev]"
.venv/bin/python -m pytest -rs        # 20 passed, 2 skipped (sionna smoke tests)
```

The two skips (`test_scene_smoke`, `test_channel_smoke`) are expected off the GPU box — they `importorskip("sionna")`.

## GPU box setup (Sionna)

```bash
python3.11 -m venv .venv          # Sionna/TF need Python 3.11
.venv/bin/python -m pip install -e ".[sim,dev]"
.venv/bin/python -m pytest tests/test_scene_smoke.py tests/test_channel_smoke.py -v
```

If a Sionna API call mismatches the installed version, fix it **only inside**
`src/wifi_radar_slam/scene/builder.py` or `src/wifi_radar_slam/channel/simulator.py`
(they are the isolation layer) and keep their return shapes/`BuiltScene` fields identical.
On first bring-up, temporarily `print(h_freq.shape)` inside `simulate_csi` to confirm the
`squeeze`/axis handling, then remove it.

## Local bring-up on a low-VRAM GPU (e.g. 4 GB)

Ray-tracing sample count dominates VRAM, so validate the Sionna wiring at reduced scale before
committing to a full server run:

```bash
# tiny scene + far fewer ray samples so it fits in ~4 GB
WRS_NUM_SAMPLES=100000 .venv/bin/python -c "
import numpy as np
from wifi_radar_slam.config import load_config
from wifi_radar_slam.runner import run_phase_a
cfg = load_config('configs/smoke.yaml')
print(run_phase_a(cfg, np.random.default_rng(cfg.seed)))
"
```

If this still OOMs, lower `WRS_NUM_SAMPLES` further (e.g. 20000) and/or shrink `configs/smoke.yaml`.
This only checks that the ray tracer runs and shapes line up — it is **not** a scientific result.

## Running the experiments (GPU box / server)

**Phase A — nominal case:**
```bash
.venv/bin/python experiments/run_phase_a.py
```
Writes:
- `results/phase_a_nominal/channel/csi.npz` (cached channel; delete to force re-simulation)
- `results/phase_a_nominal/eval/metrics.json` — `{ate, rpe, chamfer, iou}`
- `results/phase_a_nominal/eval/map.png` — estimated vs ground-truth map + trajectory

**Phase B — operating-envelope sweep:**
```bash
.venv/bin/python experiments/run_phase_b.py
```
Sweeps AP density, SNR, vehicle speed, and WiFi bandwidth (20→160 MHz) per `configs/sweep.yaml`.
Writes `results/sweep/eval/summary.json` (one record per grid point: swept param, value, metrics).
The bandwidth sweep is the headline result — it exercises the range-resolution ceiling
(ΔR = c/2B: ~3.75 m @ 40 MHz → ~0.94 m @ 160 MHz).

## Configs

- `configs/nominal.yaml` — Phase-A scene, RF, trajectory.
- `configs/sweep.yaml` — Phase-B grid (`base:` + `sweeps:`).

All randomness is seeded from `seed` in the config, so runs are reproducible.

## Artifacts & caching

Each stage writes under `results/<run_name>/<stage>/`. A run reuses an existing
`channel/csi.npz` unless you pass `force=True` to `run_phase_a` or delete the file —
ray tracing is the expensive step, so this avoids recomputation.
