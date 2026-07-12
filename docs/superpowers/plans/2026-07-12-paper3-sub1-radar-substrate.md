# Paper 3 · Sub-project 1 — Radar Substrate Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a 77 GHz FMCW automotive-radar sensor for the existing SLAM substrate — a pure-NumPy beat-signal → range-FFT → azimuth-beamforming → CA-CFAR detection chain, a Sionna monostatic ray-traced front-end that feeds it, and a KITTI-protocol drift metric — so that paper 3's ablation (sub-project 3) has a sensor to run.

**Architecture:** A new `radar/` package mirrors the structure of `lidar/`. `processing.py` holds the *entire* signal chain as pure NumPy/SciPy with no Sionna import, so it is fully unit-testable on this laptop. `sensor.py` holds the only Sionna-dependent piece — a monostatic transmitter co-located with the vehicle receiver, with diffuse scattering enabled — which extracts ray-traced paths and hands `(delay, complex amplitude, azimuth)` triples to the pure chain. The chain emits a `lidar.pointcloud.Scan`, so the **existing scan-to-map ICP back-end is reused byte-for-byte**. `eval/drift.py` adds the KITTI-style drift metric that radar-odometry reviewers expect.

**Tech Stack:** Python 3, NumPy, SciPy (`scipy.ndimage` for CFAR, already a hard dependency), Sionna RT 2.0.1 (server-only), pytest.

**Branch:** `paper3-sub1-radar-substrate`, off `paper3-wifi-vs-radar`.

**Spec:** `docs/superpowers/specs/2026-07-12-paper3-wifi-vs-radar-design.md`

---

## Global Constraints

These apply to **every** task. They are copied from the spec and from what reading the existing code established.

1. **The full detection chain is non-negotiable.** Radar detections MUST emerge from beat signal → range FFT → azimuth beamforming → CFAR. Never read Sionna interaction vertices directly as detections (that is what the *LiDAR* model B does). Extracting paths directly would give radar **zero ghosts by construction** and rig RQ1, the paper's headline.
2. **Range–azimuth, not range–Doppler.** Scenes are static. See "Deviation from the spec's architecture block" below.
3. **The shared back-end is reused unchanged.** `lidar/slam_icp.py`, `lidar/pointcloud.Scan`, and `eval/metrics.py` are NOT to be modified. The radar sensor's job is to produce a `Scan`; everything downstream already exists.
4. **`radar/processing.py` MUST NOT import Sionna or Mitsuba, at module level or inside functions.** It is pure NumPy/SciPy and is tested locally. Only `radar/sensor.py` touches Sionna, and it does so with **lazy imports inside methods** (the established pattern — see `lidar/sensor_sionna.py:58`), so the module imports fine on a machine without Sionna.
5. **Sionna is not installed locally.** It lives only on the server `amd` at `/home/dev/mulham/wifi-radar-slam` (`.venv`). Tests must never require it. Follow the existing precedent: `tests/test_lidar_sensor_sionna.py` tests only the pure helpers and never instantiates the Sionna class.
6. **Speed of light:** `C = 299792458.0`, defined locally in the module, matching `sensing/frontend.py:6`.
7. **Frames:** a `Scan` holds points in the **sensor-local** frame with **+x forward**, matching `lidar/pointcloud.py:9`. Azimuth `θ` is measured from +x, positive toward +y.
8. **Every quantitative claim must trace to a committed artifact.** No number goes in a paper that a committed script did not produce.

---

## Deviation from the spec's architecture block — read this before Task 2

The spec's architecture block (written before the range–azimuth decision was finalised) sketches:

```
beat_cube(paths, cfg)  -> (n_rx, n_chirps, n_samples) complex
range_doppler(cube)    -> windowed 2-D FFT
```

**We are not building that, and the spec's own later text is why.** The spec decides, in "Range–azimuth, not range–Doppler (and why)", that the scenes are static and the chain is range–azimuth. Two consequences follow that the stale architecture block does not reflect:

- **A chirp axis buys nothing on a static scene.** With no moving targets, every chirp in the coherent processing interval carries the *same* signal and differs only in noise. Coherently integrating `n_chirps` of them is therefore *exactly* equivalent to generating the signal once and dividing the noise standard deviation by `sqrt(n_chirps)`. We model it that way. The output is a **`(n_rx, n_samples)` beat matrix**, not a 3-D cube.
- **This also retires pitfall #4.** The spec warns that Sionna's time evolution is synthetic — geometry frozen, only phase rotating, "only accurate over very short time spans" — and asks us to validate the CPI at vehicular speed. By collapsing the chirp axis analytically we **never rely on within-CPI evolution at all**, so the pitfall cannot bite. This is a strictly stronger position than validating it.
- It is also ~`n_chirps`× cheaper, which matters: a Sionna monostatic solve yields thousands of paths, and a 3-D contraction over them would dominate runtime.

`n_chirps` survives in `RadarConfig` as exactly what it now is: **the coherent-integration factor**. Doppler processing remains an optional future extension, as the spec says.

**Action:** Task 0 amends the spec's architecture block to match. Do not skip it — a spec that contradicts the code is worse than no spec.

---

## File Structure

| File | Responsibility |
|---|---|
| `src/wifi_radar_slam/radar/__init__.py` | Package marker; re-export `RadarConfig` and the presets. |
| `src/wifi_radar_slam/radar/config.py` | `RadarConfig` dataclass + derived RF quantities + the four cell presets. No logic beyond arithmetic. |
| `src/wifi_radar_slam/radar/processing.py` | The **entire** pure signal chain: `beat_matrix` → `range_fft` → `azimuth_beamform` → `cfar_2d` → `cluster_detections` → `detections_to_scan`. Pure NumPy/SciPy. |
| `src/wifi_radar_slam/radar/sensor.py` | `SionnaRadarSensor` (the only Sionna-touching code) + the `radar_sensor` factory on the `make_sensor` seam + the pure `paths_to_rays` helper that is unit-tested locally. |
| `src/wifi_radar_slam/eval/drift.py` | KITTI-protocol drift: SE(2) helpers, `path_lengths`, `drift`. |
| `tests/test_radar_config.py` | Preset arithmetic, derived quantities. |
| `tests/test_radar_processing.py` | The chain, stage by stage, plus the end-to-end two-target recovery test. |
| `tests/test_radar_sensor.py` | The pure `paths_to_rays` helper only (Sionna class is not instantiated locally). |
| `tests/test_drift.py` | Drift on synthetic trajectories, incl. the too-short-trajectory case. |
| `experiments/validate_radar_sensor.py` | **Server-only** gate: run the Sionna sensor on a known geometry and confirm it recovers a real wall at the true range and bearing. Resolves pitfalls #2 and #5 empirically. |

---

## The other thing reading the code turned up: our simulated trajectories are too short for KITTI drift

The KITTI protocol averages translational error over sub-sequences of **100–800 m**. Our simulated trajectories are **30–60 m** (`configs/nominal.yaml:11` → `length_m: 60.0`; the controlled scenes are 30 m). **Standard KITTI drift is therefore mathematically undefined on our simulated runs** — there is not a single 100 m sub-sequence to measure.

This is not a reason to weaken the metric. It is a reason to be precise about where it applies:

- **On the real-radar credibility anchor (sub-project 2 — Oxford / Boreas, km-scale drives):** use the **standard** 100–800 m lengths. That is the only place drift % needs to be directly comparable to the published CFEAR (1.09 %) and DRO (0.26 %) rows, and there it is fully valid.
- **On the simulated ablation cells (sub-project 3):** the primary metrics stay ATE/RPE plus the four map metrics, exactly as in paper 2. Drift may be reported at *reduced* sub-sequence lengths, but **only** if labelled as such and **never** placed in the same column as a published KITTI/Oxford number.

Therefore `drift()` takes `lengths` as a parameter, and — critically — **returns `NaN` with `n_segments=0` rather than inventing a number** when the trajectory is too short. Task 8 tests exactly that. This also supersedes paper 2's looser `docs/results-paper2.md:38` claim ("aligned ATE 1.16 m ≈ 0.3 % drift"), which was an ad-hoc ratio, not the KITTI protocol.

---

### Task 0: Branch, and correct the stale spec architecture block

**Files:**
- Modify: `docs/superpowers/specs/2026-07-12-paper3-wifi-vs-radar-design.md` (the `## Architecture` code block, and the acceptance bullet that names the chain)

- [ ] **Step 1: Cut the branch**

```bash
cd /mnt/data/projects/wifi-radar
git checkout paper3-wifi-vs-radar
git pull --ff-only
git checkout -b paper3-sub1-radar-substrate
```

- [ ] **Step 2: Replace the architecture code block**

In the spec, replace the `radar/` block under `## Architecture` with this (note `processing.py`'s new signatures and the `n_chirps` note):

```
radar/
  config.py       RadarConfig: carrier, bandwidth, chirp time, ADC samples, n_chirps
                  (= coherent-integration factor; the scenes are static, so chirps are
                  modelled analytically -- see below), ULA n_rx/spacing, azimuth grid,
                  min/max range, CFAR guard/training cells, Pfa.
                  Presets: RADAR_77G_4G, RADAR_77G_160M, WIFI_5G2_160M (cell B).
  processing.py   PURE NumPy/SciPy signal chain (no Sionna; tested locally):
                    beat_matrix(taus, amps, azimuths, cfg, rng) -> (n_rx, n_samples) complex
                    range_fft(beat, cfg)          -> (n_rx, n_range) complex
                    azimuth_beamform(rf, cfg)     -> (n_azimuth, n_range) real power
                    cfar_2d(ra_map, cfg)          -> (n_azimuth, n_range) bool mask
                    cluster_detections(mask, ra_map, cfg) -> (ranges, azimuths)
                    detections_to_scan(ranges, azimuths, cfg) -> Scan  [monostatic polar->Cartesian]
  sensor.py       SionnaRadarSensor: monostatic TX co-located with the vehicle RX +
                  diffuse scattering -> paths (tau, a, phi_r) -> the chain above -> Scan.
                  make_sensor seam: radar_sensor(built, cfg, rng) -> (pose -> Scan)
eval/drift.py     KITTI-protocol drift %: translational error over sub-trajectories
                  (standard lengths 100-800 m) and rotational deg/100 m. `lengths` is a
                  parameter; returns NaN / n_segments=0 when the trajectory is too short
                  rather than fabricating a value. REQUIRED: it is the accepted radar
                  protocol, and ATE alone would be marked down.
```

**Why `(n_rx, n_samples)` and not the `(n_rx, n_chirps, n_samples)` cube:** the scenes are static, so
every chirp in the CPI carries an identical signal and differs only in noise. Coherently integrating
`n_chirps` chirps is therefore analytically identical to generating the signal once with the noise
standard deviation scaled by `1/sqrt(n_chirps)` — which is what `beat_matrix` does. This is exact for
a static scene, is `n_chirps`x cheaper, and it **retires pitfall #4** (Sionna's synthetic within-CPI
time evolution) outright, because we never depend on within-CPI evolution at all.

- [ ] **Step 3: Fix the drift acceptance bullet**

Under `## Acceptance`, replace the `eval/drift.py` bullet with:

```
- `eval/drift.py`: KITTI-protocol drift %, unit-tested. Applied with **standard 100-800 m
  sub-sequence lengths on the real-radar anchor** (sub-project 2), where it is directly
  comparable to the cited CFEAR/DRO rows. Our simulated trajectories are 30-60 m, so
  standard drift is undefined there: the simulated cells report ATE/RPE + the four map
  metrics (as in paper 2), and any reduced-length drift figure is labelled as such and
  never tabulated beside a published KITTI/Oxford number.
```

- [ ] **Step 4: Commit**

```bash
git add docs/superpowers/specs/2026-07-12-paper3-wifi-vs-radar-design.md
git commit -m "paper3(spec): range-azimuth chain signatures; scope KITTI drift to the real-data anchor

The architecture block predated the range-azimuth decision and still described a
range-Doppler cube. Static scenes make the chirp axis analytic, which also retires
the synthetic-time-evolution pitfall. Separately: our 30-60 m simulated trajectories
cannot support KITTI's 100-800 m sub-sequences, so drift is scoped to the real-radar
anchor and drift() reports NaN rather than fabricating a value on short tracks."
```

---

### Task 1: `RadarConfig` and the cell presets

**Files:**
- Create: `src/wifi_radar_slam/radar/__init__.py`
- Create: `src/wifi_radar_slam/radar/config.py`
- Test: `tests/test_radar_config.py`

**Interfaces:**
- Consumes: nothing.
- Produces: `RadarConfig` (frozen dataclass) with fields `carrier_hz, bandwidth_hz, chirp_time_s, n_samples, n_chirps, n_rx, rx_spacing_frac, n_azimuth, fov_deg, max_range_m, min_range_m, cfar_guard_range, cfar_train_range, cfar_guard_azimuth, cfar_train_azimuth, pfa, noise_sigma`; properties `wavelength_m, sweep_slope_hz_per_s, sample_rate_hz, range_resolution_m, max_beat_range_m, n_range`; methods `range_bins() -> (n_range,)`, `azimuth_grid() -> (n_azimuth,)`. Presets `RADAR_77G_4G`, `RADAR_77G_160M`, `WIFI_5G2_160M`. Every later task depends on these exact names.

- [ ] **Step 1: Write the failing test**

Create `tests/test_radar_config.py`:

```python
import numpy as np
import pytest
from wifi_radar_slam.radar.config import (RadarConfig, RADAR_77G_4G,
                                          RADAR_77G_160M, WIFI_5G2_160M)

C = 299792458.0


def test_range_resolution_is_c_over_2b():
    # The textbook FMCW range resolution. 4 GHz -> ~3.7 cm; 160 MHz -> ~94 cm.
    assert RADAR_77G_4G.range_resolution_m == pytest.approx(C / (2 * 4e9))
    assert RADAR_77G_4G.range_resolution_m == pytest.approx(0.0375, abs=1e-3)
    assert RADAR_77G_160M.range_resolution_m == pytest.approx(0.937, abs=1e-2)


def test_narrowband_radar_and_wifi_cell_share_a_range_resolution():
    # This is the whole point of the ablation: cells C and B differ ONLY in carrier,
    # so their range resolution must be identical.
    assert RADAR_77G_160M.range_resolution_m == pytest.approx(
        WIFI_5G2_160M.range_resolution_m)


def test_carrier_separates_the_cells():
    assert RADAR_77G_4G.carrier_hz == 77e9
    assert RADAR_77G_160M.carrier_hz == 77e9
    assert WIFI_5G2_160M.carrier_hz == 5.2e9


def test_wavelength():
    assert RADAR_77G_4G.wavelength_m == pytest.approx(C / 77e9)
    assert RADAR_77G_4G.wavelength_m == pytest.approx(0.0039, abs=1e-4)


def test_sweep_slope_and_sample_rate():
    cfg = RadarConfig(carrier_hz=77e9, bandwidth_hz=4e9, chirp_time_s=40e-6,
                      n_samples=256, n_chirps=128, n_rx=16, rx_spacing_frac=0.5,
                      n_azimuth=181, fov_deg=180.0, max_range_m=100.0, min_range_m=1.0,
                      cfar_guard_range=2, cfar_train_range=8,
                      cfar_guard_azimuth=2, cfar_train_azimuth=4,
                      pfa=1e-4, noise_sigma=0.0)
    assert cfg.sweep_slope_hz_per_s == pytest.approx(4e9 / 40e-6)
    assert cfg.sample_rate_hz == pytest.approx(256 / 40e-6)


def test_range_bins_are_monotone_and_start_at_zero():
    bins = RADAR_77G_4G.range_bins()
    assert bins.shape == (RADAR_77G_4G.n_range,)
    assert bins[0] == pytest.approx(0.0)
    assert np.all(np.diff(bins) > 0)


def test_range_bin_spacing_equals_range_resolution():
    # An n_samples-point FFT over the sweep gives bins spaced exactly c/(2B).
    bins = RADAR_77G_4G.range_bins()
    assert np.diff(bins)[0] == pytest.approx(RADAR_77G_4G.range_resolution_m, rel=1e-9)


def test_max_beat_range_covers_the_configured_max_range():
    # If the ADC cannot sample the beat frequency of a target at max_range_m,
    # that target aliases. The presets must not be self-contradictory.
    for cfg in (RADAR_77G_4G, RADAR_77G_160M, WIFI_5G2_160M):
        assert cfg.max_beat_range_m >= cfg.max_range_m


def test_azimuth_grid_spans_the_fov_symmetrically():
    grid = RADAR_77G_4G.azimuth_grid()
    assert grid.shape == (RADAR_77G_4G.n_azimuth,)
    assert grid[0] == pytest.approx(-np.deg2rad(RADAR_77G_4G.fov_deg) / 2)
    assert grid[-1] == pytest.approx(np.deg2rad(RADAR_77G_4G.fov_deg) / 2)
    assert np.all(np.diff(grid) > 0)


def test_config_is_frozen():
    with pytest.raises(Exception):
        RADAR_77G_4G.carrier_hz = 1.0
```

- [ ] **Step 2: Run the test and watch it fail**

```bash
cd /mnt/data/projects/wifi-radar
python3 -m pytest tests/test_radar_config.py -v
```

Expected: collection error — `ModuleNotFoundError: No module named 'wifi_radar_slam.radar'`.

- [ ] **Step 3: Implement**

Create `src/wifi_radar_slam/radar/__init__.py`:

```python
"""77 GHz FMCW automotive-radar sensor model (paper 3).

`config` and `processing` are pure NumPy/SciPy and test locally. `sensor` is the only
module that touches Sionna, and it imports it lazily inside methods — so importing this
package never requires Sionna.
"""
from .config import RadarConfig, RADAR_77G_4G, RADAR_77G_160M, WIFI_5G2_160M

__all__ = ["RadarConfig", "RADAR_77G_4G", "RADAR_77G_160M", "WIFI_5G2_160M"]
```

Create `src/wifi_radar_slam/radar/config.py`:

```python
from __future__ import annotations
from dataclasses import dataclass
import numpy as np

C = 299792458.0


@dataclass(frozen=True)
class RadarConfig:
    """FMCW radar parameters for the range-azimuth detection chain (BEV plane).

    One config type serves every ablation cell: a cell is just a (carrier, bandwidth)
    pair on a fixed detection chain, which is exactly what makes the ablation clean.

    `n_chirps` is the **coherent-integration factor**, not a processed axis. The scenes
    are static, so all chirps in the CPI carry the same signal and differ only in noise;
    integrating them coherently is analytically identical to scaling the noise sigma by
    1/sqrt(n_chirps), which is what `processing.beat_matrix` does. See the plan's
    "Deviation from the spec's architecture block".
    """
    carrier_hz: float          # f_c
    bandwidth_hz: float        # B, the sweep bandwidth -- sets range resolution
    chirp_time_s: float        # T_c, sweep duration
    n_samples: int             # ADC samples per chirp (fast time) -> range bins
    n_chirps: int              # chirps per CPI == coherent-integration factor
    n_rx: int                  # (virtual) ULA elements
    rx_spacing_frac: float     # element spacing in wavelengths (0.5 = half-wavelength)
    n_azimuth: int             # beamforming grid size
    fov_deg: float             # total azimuth field of view
    max_range_m: float         # detections beyond this are discarded
    min_range_m: float         # blind zone: TX/RX are co-located, so near returns are self-clutter
    cfar_guard_range: int      # CA-CFAR guard cells, range axis
    cfar_train_range: int      # CA-CFAR training cells, range axis
    cfar_guard_azimuth: int    # CA-CFAR guard cells, azimuth axis
    cfar_train_azimuth: int    # CA-CFAR training cells, azimuth axis
    pfa: float                 # CFAR design probability of false alarm (per cell)
    noise_sigma: float         # per-sample complex-noise std BEFORE coherent integration

    # --- derived RF quantities -------------------------------------------------
    @property
    def wavelength_m(self) -> float:
        return C / self.carrier_hz

    @property
    def sweep_slope_hz_per_s(self) -> float:
        """S = B / T_c. A target at delay tau beats at f_b = S * tau."""
        return self.bandwidth_hz / self.chirp_time_s

    @property
    def sample_rate_hz(self) -> float:
        return self.n_samples / self.chirp_time_s

    @property
    def range_resolution_m(self) -> float:
        """c / 2B -- set by bandwidth alone. This is the ablation's C->D axis."""
        return C / (2.0 * self.bandwidth_hz)

    @property
    def n_range(self) -> int:
        """Range bins kept: the positive half of the real-input FFT, minus DC's mirror."""
        return self.n_samples // 2

    @property
    def max_beat_range_m(self) -> float:
        """Range whose beat frequency sits at the top kept FFT bin.

        f_b = 2*R*S/c, and the highest kept bin is (n_range-1)*fs/n_samples, so
        R_max = (n_range-1) * fs * c / (n_samples * 2 * S). Anything past this aliases.
        """
        f_b_max = (self.n_range - 1) * self.sample_rate_hz / self.n_samples
        return f_b_max * C / (2.0 * self.sweep_slope_hz_per_s)

    # --- grids -----------------------------------------------------------------
    def range_bins(self) -> np.ndarray:
        """Range (m) of each kept FFT bin. Spacing == range_resolution_m exactly."""
        k = np.arange(self.n_range)
        f_b = k * self.sample_rate_hz / self.n_samples
        return f_b * C / (2.0 * self.sweep_slope_hz_per_s)

    def azimuth_grid(self) -> np.ndarray:
        """Beamforming steering angles (rad), spanning the FOV symmetrically about 0
        (local +x forward, positive toward +y)."""
        half = np.deg2rad(self.fov_deg) / 2.0
        return np.linspace(-half, half, self.n_azimuth)


# --- the ablation cells ---------------------------------------------------------
# Chirp/ADC numbers follow a TI AWR-class automotive front end: ~40 us sweep, 256 ADC
# samples, 128 chirps per CPI, a 16-element virtual (MIMO) ULA at half-wavelength.
# Only carrier_hz and bandwidth_hz change between cells -- that is the ablation.
_COMMON = dict(chirp_time_s=40e-6, n_samples=256, n_chirps=128, n_rx=16,
               rx_spacing_frac=0.5, n_azimuth=181, fov_deg=180.0,
               max_range_m=100.0, min_range_m=1.0,
               cfar_guard_range=2, cfar_train_range=8,
               cfar_guard_azimuth=2, cfar_train_azimuth=4,
               pfa=1e-4, noise_sigma=1e-3)

# Cell D: full-bandwidth 77 GHz automotive radar. 4 GHz -> 3.75 cm range resolution.
RADAR_77G_4G = RadarConfig(carrier_hz=77e9, bandwidth_hz=4e9, **_COMMON)

# Cell C: the SAME radar crippled to WiFi's bandwidth. Isolates what the carrier buys.
RADAR_77G_160M = RadarConfig(carrier_hz=77e9, bandwidth_hz=160e6, **_COMMON)

# Cell B: an active monostatic WiFi radar (5.2 GHz, 160 MHz). Isolates bistatic geometry.
WIFI_5G2_160M = RadarConfig(carrier_hz=5.2e9, bandwidth_hz=160e6, **_COMMON)
```

- [ ] **Step 4: Run the tests and watch them pass**

```bash
python3 -m pytest tests/test_radar_config.py -v
```

Expected: 9 passed.

If `test_max_beat_range_covers_the_configured_max_range` fails, the presets are physically
self-contradictory (the ADC cannot sample a 100 m target's beat frequency) — do **not** relax
the test. Raise `n_samples`, or shorten `chirp_time_s` to raise the slope-to-sample-rate ratio,
until the config is consistent. A radar that aliases its own max range is a bug, not a test problem.

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/radar/__init__.py src/wifi_radar_slam/radar/config.py tests/test_radar_config.py
git commit -m "paper3(radar): RadarConfig + the ablation cell presets

Cells B/C/D differ ONLY in (carrier, bandwidth) on one fixed detection chain --
which is what makes the bandwidth-vs-carrier-vs-geometry decomposition clean."
```

---

### Task 2: Beat-signal synthesis

**Files:**
- Create: `src/wifi_radar_slam/radar/processing.py`
- Test: `tests/test_radar_processing.py`

**Interfaces:**
- Consumes: `RadarConfig` (Task 1).
- Produces: `beat_matrix(taus, amps, azimuths, cfg, rng=None) -> np.ndarray` of shape `(cfg.n_rx, cfg.n_samples)`, complex. `taus` are **absolute one-way** delays in seconds? **No — see below.** The contract: `taus` are the **round-trip propagation delays** (s) exactly as a monostatic radar measures them, `amps` are complex path amplitudes, `azimuths` are **sensor-local** arrival azimuths (rad, +x forward). Later tasks rely on this signature verbatim.

- [ ] **Step 1: Write the failing test**

Create `tests/test_radar_processing.py`:

```python
import numpy as np
import pytest
from wifi_radar_slam.radar.config import RadarConfig
from wifi_radar_slam.radar.processing import beat_matrix

C = 299792458.0


def cfg_small(noise=0.0):
    """A small, fast config for unit tests. 1 GHz -> 15 cm range resolution."""
    return RadarConfig(carrier_hz=77e9, bandwidth_hz=1e9, chirp_time_s=40e-6,
                       n_samples=128, n_chirps=1, n_rx=8, rx_spacing_frac=0.5,
                       n_azimuth=91, fov_deg=180.0, max_range_m=60.0, min_range_m=1.0,
                       cfar_guard_range=2, cfar_train_range=6,
                       cfar_guard_azimuth=2, cfar_train_azimuth=3,
                       pfa=1e-3, noise_sigma=noise)


def tau_of(range_m):
    """Monostatic round-trip delay for a target at `range_m`."""
    return 2.0 * range_m / C


def test_beat_matrix_shape_and_dtype():
    cfg = cfg_small()
    b = beat_matrix([tau_of(20.0)], [1.0 + 0j], [0.0], cfg)
    assert b.shape == (cfg.n_rx, cfg.n_samples)
    assert np.iscomplexobj(b)


def test_beat_frequency_matches_the_target_range():
    # THE core physical check: a target at range R must produce a beat tone at
    # f_b = 2*R*S/c. We read the tone straight off an FFT of one antenna's row.
    cfg = cfg_small()
    R = 20.0
    b = beat_matrix([tau_of(R)], [1.0 + 0j], [0.0], cfg)
    spec = np.abs(np.fft.fft(b[0]))[: cfg.n_range]
    peak_bin = int(np.argmax(spec))
    f_b_expected = 2.0 * R * cfg.sweep_slope_hz_per_s / C
    bin_expected = f_b_expected * cfg.n_samples / cfg.sample_rate_hz
    assert peak_bin == pytest.approx(bin_expected, abs=1.0)


def test_a_farther_target_beats_higher():
    cfg = cfg_small()
    peaks = []
    for R in (10.0, 40.0):
        b = beat_matrix([tau_of(R)], [1.0 + 0j], [0.0], cfg)
        peaks.append(int(np.argmax(np.abs(np.fft.fft(b[0]))[: cfg.n_range])))
    assert peaks[1] > peaks[0]


def test_azimuth_appears_as_a_linear_phase_ramp_across_the_array():
    # A target at azimuth theta imposes phase 2*pi*d/lambda*m*sin(theta) on element m.
    # With half-wavelength spacing that is pi*m*sin(theta).
    cfg = cfg_small()
    theta = np.deg2rad(30.0)
    b = beat_matrix([tau_of(20.0)], [1.0 + 0j], [theta], cfg)
    # phase difference between adjacent elements, averaged over fast time
    d_phase = np.angle(b[1:, :] * np.conj(b[:-1, :]))
    expected = 2 * np.pi * cfg.rx_spacing_frac * np.sin(theta)
    assert np.mean(d_phase) == pytest.approx(expected, abs=1e-6)


def test_boresight_target_gives_a_flat_phase_front():
    cfg = cfg_small()
    b = beat_matrix([tau_of(20.0)], [1.0 + 0j], [0.0], cfg)
    d_phase = np.angle(b[1:, :] * np.conj(b[:-1, :]))
    assert np.allclose(d_phase, 0.0, atol=1e-9)


def test_paths_superpose_linearly():
    cfg = cfg_small()
    p1 = ([tau_of(10.0)], [1.0 + 0j], [0.0])
    p2 = ([tau_of(30.0)], [0.5 + 0j], [np.deg2rad(20)])
    both = beat_matrix(p1[0] + p2[0], p1[1] + p2[1], p1[2] + p2[2], cfg)
    assert np.allclose(both, beat_matrix(*p1, cfg) + beat_matrix(*p2, cfg))


def test_empty_path_list_gives_noise_only():
    cfg = cfg_small()
    b = beat_matrix([], [], [], cfg)
    assert b.shape == (cfg.n_rx, cfg.n_samples)
    assert np.allclose(b, 0.0)


def test_coherent_integration_scales_noise_by_sqrt_n_chirps():
    # n_chirps is the coherent-integration factor: 100x the chirps must cut the
    # noise std by 10x. This is the analytic stand-in for the chirp axis.
    base = cfg_small(noise=1.0)
    many = RadarConfig(**{**base.__dict__, "n_chirps": 100})
    rng = np.random.default_rng(0)
    n1 = beat_matrix([], [], [], base, rng=np.random.default_rng(0))
    n2 = beat_matrix([], [], [], many, rng=np.random.default_rng(0))
    # same noise draw, scaled: std(n2) ~= std(n1)/10
    assert np.std(n2) == pytest.approx(np.std(n1) / 10.0, rel=1e-9)
```

- [ ] **Step 2: Run the test and watch it fail**

```bash
python3 -m pytest tests/test_radar_processing.py -v
```

Expected: collection error — `ModuleNotFoundError: No module named 'wifi_radar_slam.radar.processing'`.

- [ ] **Step 3: Implement**

Create `src/wifi_radar_slam/radar/processing.py` (this file grows over Tasks 2–6; write only
`beat_matrix` and the header now):

```python
"""The FMCW radar detection chain: beat signal -> range FFT -> azimuth beamforming -> CFAR.

PURE NumPy/SciPy. This module MUST NOT import Sionna or Mitsuba, so the whole chain is
unit-testable without the simulator.

Why the chain exists at all, in full: paper 3's headline question is whether the ~89 %
phantom-detection rate paper 2 measured on WiFi is a WiFi pathology or a property of RF
sensing. Reading ray-traced paths out of the simulator directly -- as the LiDAR model does
-- would hand radar **zero ghosts by construction** and rig that comparison. Ghosts and
false alarms have to *emerge* from finite bandwidth, a finite aperture and a CFAR
threshold, exactly as they do on real hardware. Hence: no shortcuts.
"""
from __future__ import annotations
import numpy as np

C = 299792458.0


def beat_matrix(taus, amps, azimuths, cfg, rng=None) -> np.ndarray:
    """Synthesize the dechirped (beat) signal across the RX array.

    Args:
        taus:     round-trip propagation delays (s), one per ray, as a monostatic radar
                  measures them (i.e. 2*R/c for a single-bounce target at range R).
        amps:     complex path amplitudes, one per ray.
        azimuths: sensor-local arrival azimuths (rad), +x forward, positive toward +y.
        cfg:      RadarConfig.
        rng:      numpy Generator for receiver noise (None -> no noise).

    Returns:
        (cfg.n_rx, cfg.n_samples) complex beat matrix.

    An FMCW sweep of slope S, mixed with its own echo at delay tau, leaves the beat tone

        s(t) = a * exp(j*2*pi*(S*tau*t + f_c*tau - 0.5*S*tau**2))

    -- a tone whose frequency S*tau encodes range, plus a carrier phase f_c*tau. The
    residual-video-phase term -0.5*S*tau**2 is small but kept: it is free, and dropping it
    would be a silent approximation.

    There is no chirp axis. The scenes are static, so every chirp in the CPI carries an
    identical signal and differs only in noise; coherently integrating cfg.n_chirps of them
    is analytically identical to generating the signal once with the noise standard
    deviation divided by sqrt(cfg.n_chirps), which is what we do. This is exact here, is
    cfg.n_chirps times cheaper, and it means we never rely on Sionna's synthetic
    within-CPI time evolution (a documented pitfall) at all.
    """
    taus = np.asarray(taus, dtype=float).ravel()
    amps = np.asarray(amps, dtype=complex).ravel()
    azimuths = np.asarray(azimuths, dtype=float).ravel()
    if not (taus.shape == amps.shape == azimuths.shape):
        raise ValueError(f"ragged rays: {taus.shape}, {amps.shape}, {azimuths.shape}")

    n = np.arange(cfg.n_samples)
    t = n / cfg.sample_rate_hz                      # fast time (n_samples,)
    m = np.arange(cfg.n_rx)                         # array elements (n_rx,)

    beat = np.zeros((cfg.n_rx, cfg.n_samples), dtype=complex)
    if taus.size:
        S = cfg.sweep_slope_hz_per_s
        # (n_paths, n_samples): the beat tone of each ray over fast time, amplitude folded in
        phase_t = 2 * np.pi * (S * taus[:, None] * t[None, :]
                               + cfg.carrier_hz * taus[:, None]
                               - 0.5 * S * taus[:, None] ** 2)
        rng_mat = amps[:, None] * np.exp(1j * phase_t)
        # (n_rx, n_paths): the ULA steering phase of each ray
        steer = np.exp(2j * np.pi * cfg.rx_spacing_frac
                       * m[:, None] * np.sin(azimuths)[None, :])
        beat = steer @ rng_mat                      # (n_rx, n_samples)

    if rng is not None and cfg.noise_sigma > 0:
        # Coherent integration over the CPI: sigma -> sigma / sqrt(n_chirps).
        sigma = cfg.noise_sigma / np.sqrt(cfg.n_chirps)
        beat = beat + (sigma / np.sqrt(2)) * (rng.normal(size=beat.shape)
                                              + 1j * rng.normal(size=beat.shape))
    return beat
```

- [ ] **Step 4: Run the tests and watch them pass**

```bash
python3 -m pytest tests/test_radar_processing.py -v
```

Expected: 8 passed.

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/radar/processing.py tests/test_radar_processing.py
git commit -m "paper3(radar): FMCW beat-signal synthesis

Range enters as a beat tone S*tau, azimuth as a ULA phase ramp, and the chirp axis is
folded analytically into the noise sigma (static scenes)."
```

---

### Task 3: Range FFT

**Files:**
- Modify: `src/wifi_radar_slam/radar/processing.py` (append)
- Test: `tests/test_radar_processing.py` (append)

**Interfaces:**
- Consumes: `beat_matrix` (Task 2), `cfg.range_bins()` (Task 1).
- Produces: `range_fft(beat, cfg) -> np.ndarray` of shape `(cfg.n_rx, cfg.n_range)`, complex. Bin *i* corresponds to range `cfg.range_bins()[i]`.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_radar_processing.py`:

```python
from wifi_radar_slam.radar.processing import range_fft


def test_range_fft_shape():
    cfg = cfg_small()
    b = beat_matrix([tau_of(20.0)], [1.0 + 0j], [0.0], cfg)
    rf = range_fft(b, cfg)
    assert rf.shape == (cfg.n_rx, cfg.n_range)
    assert np.iscomplexobj(rf)


def test_range_fft_peak_lands_on_the_true_range():
    # The end-to-end range check: put a target at 20 m, read 20 m back out.
    cfg = cfg_small()
    R = 20.0
    rf = range_fft(beat_matrix([tau_of(R)], [1.0 + 0j], [0.0], cfg), cfg)
    power = np.abs(rf).sum(axis=0)
    peak_range = cfg.range_bins()[int(np.argmax(power))]
    # within one range cell (c/2B = 15 cm here)
    assert peak_range == pytest.approx(R, abs=cfg.range_resolution_m)


def test_range_fft_resolves_two_targets_separated_by_more_than_a_cell():
    cfg = cfg_small()                       # 15 cm range cells
    rf = range_fft(beat_matrix([tau_of(20.0), tau_of(25.0)],
                               [1.0 + 0j, 1.0 + 0j], [0.0, 0.0], cfg), cfg)
    power = np.abs(rf).sum(axis=0)
    bins = cfg.range_bins()
    # the two strongest local maxima should sit at 20 m and 25 m
    order = np.argsort(power)[::-1]
    found = sorted(bins[order[:2]])
    # peaks may be adjacent bins; check the two clusters exist by nearest-bin lookup
    assert min(abs(bins[np.argmax(power)] - 20.0),
               abs(bins[np.argmax(power)] - 25.0)) < 0.5
    near20 = power[np.argmin(np.abs(bins - 20.0))]
    near25 = power[np.argmin(np.abs(bins - 25.0))]
    assert near20 > 0.5 * power.max() and near25 > 0.5 * power.max()


def test_windowing_suppresses_sidelobes():
    # A Hann window trades main-lobe width for sidelobe suppression -- essential,
    # because a strong target's rectangular-window sidelobes would otherwise trip
    # CFAR and masquerade as ghosts, contaminating the very rate we are measuring.
    cfg = cfg_small()
    b = beat_matrix([tau_of(20.0)], [1.0 + 0j], [0.0], cfg)
    windowed = np.abs(range_fft(b, cfg)).sum(axis=0)
    raw = np.abs(np.fft.fft(b, axis=1)[:, : cfg.n_range]).sum(axis=0)
    peak = int(np.argmax(windowed))
    far = np.r_[0:max(peak - 10, 0), min(peak + 11, cfg.n_range):cfg.n_range]
    # sidelobe floor far from the peak, relative to the peak, must be lower with the window
    assert (windowed[far].max() / windowed[peak]) < (raw[far].max() / raw[peak])
```

- [ ] **Step 2: Run the tests and watch them fail**

```bash
python3 -m pytest tests/test_radar_processing.py -v
```

Expected: `ImportError: cannot import name 'range_fft'`.

- [ ] **Step 3: Implement**

Append to `src/wifi_radar_slam/radar/processing.py`:

```python
def range_fft(beat: np.ndarray, cfg) -> np.ndarray:
    """Windowed FFT along fast time -> a complex range profile per array element.

    Returns (cfg.n_rx, cfg.n_range); bin i is at range cfg.range_bins()[i].

    The Hann window is not cosmetic. With a rectangular window, a strong target's
    spectral sidelobes (-13 dB, decaying slowly) sit well above the noise floor and trip
    CFAR at ranges where nothing exists -- manufacturing "ghosts" that are artifacts of
    our own processing rather than of the physics. Since the phantom rate is the paper's
    headline measurement, that contamination is disqualifying. Hann drops the first
    sidelobe to -31 dB and rolls off fast, at the cost of a ~2x wider main lobe.
    """
    w = np.hanning(cfg.n_samples)
    return np.fft.fft(beat * w[None, :], axis=1)[:, : cfg.n_range]
```

- [ ] **Step 4: Run the tests and watch them pass**

```bash
python3 -m pytest tests/test_radar_processing.py -v
```

Expected: 12 passed.

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/radar/processing.py tests/test_radar_processing.py
git commit -m "paper3(radar): windowed range FFT

Hann windowing is load-bearing: rectangular-window sidelobes would trip CFAR and
inflate the very phantom rate the paper measures."
```

---

### Task 4: Azimuth beamforming

**Files:**
- Modify: `src/wifi_radar_slam/radar/processing.py` (append)
- Test: `tests/test_radar_processing.py` (append)

**Interfaces:**
- Consumes: `range_fft` (Task 3), `cfg.azimuth_grid()` (Task 1).
- Produces: `azimuth_beamform(rf, cfg) -> np.ndarray` of shape `(cfg.n_azimuth, cfg.n_range)`, **real, non-negative power**. Row *j* is steering angle `cfg.azimuth_grid()[j]`; column *i* is range `cfg.range_bins()[i]`. This 2-D array is "the range–azimuth map" everywhere downstream.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_radar_processing.py`:

```python
from wifi_radar_slam.radar.processing import azimuth_beamform


def _ra_map(cfg, ranges, azimuths, amps=None, rng=None):
    amps = amps if amps is not None else [1.0 + 0j] * len(ranges)
    b = beat_matrix([tau_of(r) for r in ranges], amps, azimuths, cfg, rng=rng)
    return azimuth_beamform(range_fft(b, cfg), cfg)


def test_beamform_shape_and_realness():
    cfg = cfg_small()
    ra = _ra_map(cfg, [20.0], [0.0])
    assert ra.shape == (cfg.n_azimuth, cfg.n_range)
    assert np.isrealobj(ra)
    assert np.all(ra >= 0)


def test_beamform_peak_lands_on_the_true_range_and_azimuth():
    # THE end-to-end check for the whole front half of the chain.
    cfg = cfg_small()
    R, th = 25.0, np.deg2rad(20.0)
    ra = _ra_map(cfg, [R], [th])
    j, i = np.unravel_index(int(np.argmax(ra)), ra.shape)
    assert cfg.range_bins()[i] == pytest.approx(R, abs=cfg.range_resolution_m)
    # azimuth resolution of an 8-element half-wavelength ULA is coarse (~13 deg at
    # boresight, worse off-boresight), so allow a beamwidth of slack
    assert np.rad2deg(cfg.azimuth_grid()[j]) == pytest.approx(20.0, abs=8.0)


def test_beamform_separates_two_targets_at_the_same_range():
    cfg = cfg_small()
    ra = _ra_map(cfg, [25.0, 25.0], [np.deg2rad(-40.0), np.deg2rad(40.0)])
    i = int(np.argmin(np.abs(cfg.range_bins() - 25.0)))
    col = ra[:, i]
    az = np.rad2deg(cfg.azimuth_grid())
    left = col[az < 0].max()
    right = col[az > 0].max()
    middle = col[np.abs(az) < 10].max()
    assert left > 3 * middle and right > 3 * middle     # two lobes, a null between


def test_beamform_is_symmetric_in_azimuth():
    cfg = cfg_small()
    up = _ra_map(cfg, [25.0], [np.deg2rad(30.0)])
    dn = _ra_map(cfg, [25.0], [np.deg2rad(-30.0)])
    assert np.allclose(up, dn[::-1, :], atol=1e-9)
```

- [ ] **Step 2: Run the tests and watch them fail**

```bash
python3 -m pytest tests/test_radar_processing.py -v
```

Expected: `ImportError: cannot import name 'azimuth_beamform'`.

- [ ] **Step 3: Implement**

Append to `src/wifi_radar_slam/radar/processing.py`:

```python
def steering_matrix(cfg) -> np.ndarray:
    """(n_azimuth, n_rx) conventional ULA steering vectors over the azimuth grid."""
    m = np.arange(cfg.n_rx)
    th = cfg.azimuth_grid()
    return np.exp(2j * np.pi * cfg.rx_spacing_frac
                  * th[:, None] * 0 + 2j * np.pi * cfg.rx_spacing_frac
                  * m[None, :] * np.sin(th)[:, None])


def azimuth_beamform(rf: np.ndarray, cfg) -> np.ndarray:
    """Conventional (Bartlett) beamforming across the array -> a range-azimuth power map.

    Returns (cfg.n_azimuth, cfg.n_range), real and non-negative.

    Deliberately NOT a superresolution beamformer (MUSIC/Capon). Papers 1-2 used MUSIC on
    WiFi, and the whole point of this ablation is to hold the *detection algorithm* fixed
    across cells so that any difference between them is **physical** -- carrier, bandwidth,
    geometry -- and not an artifact of a different estimator. FFT+CFAR is also what real
    automotive radar actually runs. MUSIC-on-WiFi survives separately as the 5th reference
    row, which is precisely where the superresolution-vs-FFT axis becomes visible instead
    of silently confounded with the physics.
    """
    A = steering_matrix(cfg)                 # (n_azimuth, n_rx)
    return np.abs(A.conj() @ rf) ** 2        # (n_azimuth, n_range)
```

- [ ] **Step 4: Run the tests and watch them pass**

```bash
python3 -m pytest tests/test_radar_processing.py -v
```

Expected: 16 passed.

If `test_beamform_is_symmetric_in_azimuth` fails, `steering_matrix` has a sign error in
`sin(th)` — that would silently mirror every map left-right and put every reflector on the
wrong side of the road, so fix it here rather than compensating downstream.

- [ ] **Step 5: Simplify `steering_matrix`**

The expression above contains a vestigial `* 0` term. Replace the function body with the clean
form and re-run the tests to confirm they still pass:

```python
def steering_matrix(cfg) -> np.ndarray:
    """(n_azimuth, n_rx) conventional ULA steering vectors over the azimuth grid."""
    m = np.arange(cfg.n_rx)
    th = cfg.azimuth_grid()
    return np.exp(2j * np.pi * cfg.rx_spacing_frac * m[None, :] * np.sin(th)[:, None])
```

```bash
python3 -m pytest tests/test_radar_processing.py -v
```

Expected: 16 passed.

- [ ] **Step 6: Commit**

```bash
git add src/wifi_radar_slam/radar/processing.py tests/test_radar_processing.py
git commit -m "paper3(radar): conventional azimuth beamforming -> range-azimuth map

Bartlett, not MUSIC, on purpose: the detection algorithm is held fixed across every
ablation cell so differences are physical, not algorithmic."
```

---

### Task 5: 2-D CA-CFAR and detection clustering

**Files:**
- Modify: `src/wifi_radar_slam/radar/processing.py` (append)
- Test: `tests/test_radar_processing.py` (append)

**Interfaces:**
- Consumes: `azimuth_beamform` (Task 4).
- Produces:
  - `cfar_2d(ra_map, cfg) -> np.ndarray` — bool mask, shape `(cfg.n_azimuth, cfg.n_range)`.
  - `cluster_detections(mask, ra_map, cfg) -> tuple[np.ndarray, np.ndarray]` — `(ranges_m, azimuths_rad)`, one entry per connected component of the mask, at its power-weighted centroid.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_radar_processing.py`:

```python
from wifi_radar_slam.radar.processing import cfar_2d, cluster_detections


def test_cfar_fires_on_a_target_and_not_on_empty_noise():
    cfg = cfg_small(noise=0.05)
    rng = np.random.default_rng(0)
    ra = _ra_map(cfg, [25.0], [0.0], rng=rng)
    mask = cfar_2d(ra, cfg)
    assert mask.dtype == bool
    assert mask.shape == ra.shape
    assert mask.any(), "CFAR must detect a clean, strong target"
    # the detection sits at the target
    j, i = np.unravel_index(int(np.argmax(np.where(mask, ra, 0))), ra.shape)
    assert cfg.range_bins()[i] == pytest.approx(25.0, abs=1.0)


def test_cfar_false_alarm_rate_on_pure_noise_is_near_pfa():
    # An empty scene: every detection is by definition a false alarm. This is the
    # test that makes the phantom-rate measurement trustworthy -- if the CFAR
    # threshold were miscalibrated, RQ1's headline number would be meaningless.
    cfg = cfg_small(noise=1.0)
    rng = np.random.default_rng(1)
    ra = _ra_map(cfg, [], [], rng=rng)
    mask = cfar_2d(ra, cfg)
    rate = mask.mean()
    assert rate < 20 * cfg.pfa, f"CFAR false-alarm rate {rate:.2e} >> Pfa {cfg.pfa:.0e}"


def test_cfar_adapts_to_a_raised_noise_floor():
    # CA-CFAR's defining property: scale the whole map and the mask is unchanged.
    cfg = cfg_small(noise=0.05)
    ra = _ra_map(cfg, [25.0], [0.0], rng=np.random.default_rng(0))
    assert np.array_equal(cfar_2d(ra, cfg), cfar_2d(100.0 * ra, cfg))


def test_cluster_detections_collapses_one_target_to_one_detection():
    # A target spreads over several range/azimuth cells; without clustering it would
    # be counted as many detections and every downstream rate would be wrong.
    cfg = cfg_small(noise=0.02)
    ra = _ra_map(cfg, [25.0], [0.0], rng=np.random.default_rng(0))
    ranges, azimuths = cluster_detections(cfar_2d(ra, cfg), ra, cfg)
    assert len(ranges) == 1
    assert ranges[0] == pytest.approx(25.0, abs=1.0)
    assert np.rad2deg(azimuths[0]) == pytest.approx(0.0, abs=8.0)


def test_cluster_detections_finds_two_separated_targets():
    cfg = cfg_small(noise=0.02)
    ra = _ra_map(cfg, [15.0, 40.0], [np.deg2rad(-30.0), np.deg2rad(30.0)],
                 rng=np.random.default_rng(0))
    ranges, azimuths = cluster_detections(cfar_2d(ra, cfg), ra, cfg)
    assert len(ranges) == 2
    order = np.argsort(ranges)
    assert ranges[order][0] == pytest.approx(15.0, abs=1.5)
    assert ranges[order][1] == pytest.approx(40.0, abs=1.5)
    assert np.rad2deg(azimuths[order][0]) == pytest.approx(-30.0, abs=10.0)
    assert np.rad2deg(azimuths[order][1]) == pytest.approx(30.0, abs=10.0)


def test_cluster_detections_on_an_empty_mask_returns_empty_arrays():
    cfg = cfg_small()
    ra = _ra_map(cfg, [], [])
    ranges, azimuths = cluster_detections(np.zeros_like(ra, dtype=bool), ra, cfg)
    assert len(ranges) == 0 and len(azimuths) == 0
```

- [ ] **Step 2: Run the tests and watch them fail**

```bash
python3 -m pytest tests/test_radar_processing.py -v
```

Expected: `ImportError: cannot import name 'cfar_2d'`.

- [ ] **Step 3: Implement**

Append to `src/wifi_radar_slam/radar/processing.py` (note the new import at the top of the file —
add `from scipy import ndimage` beside the existing `import numpy as np`):

```python
def cfar_2d(ra_map: np.ndarray, cfg) -> np.ndarray:
    """2-D cell-averaging CFAR over the range-azimuth map. Returns a bool detection mask.

    Each cell under test is compared against the mean power of a rectangular ring of
    training cells surrounding it, with a guard band excluded so that a target's own
    energy cannot inflate its noise estimate. The threshold multiplier for an
    N-training-cell CA-CFAR at design false-alarm probability Pfa is the standard

        alpha = N * (Pfa**(-1/N) - 1)

    which makes the false-alarm rate *constant* regardless of the absolute noise level --
    the whole reason a radar uses CFAR instead of a fixed threshold. That property is
    load-bearing here: the phantom rate we report in RQ1 is only meaningful if the
    threshold is calibrated, not tuned.

    Implemented with two box filters (a full window minus the guard window) rather than a
    per-cell loop -- an exact, vectorized identity, and ~1000x faster on a 181x128 map.
    """
    gr, tr = cfg.cfar_guard_range, cfg.cfar_train_range
    ga, ta = cfg.cfar_guard_azimuth, cfg.cfar_train_azimuth
    full = (2 * (ga + ta) + 1, 2 * (gr + tr) + 1)      # (azimuth, range)
    guard = (2 * ga + 1, 2 * gr + 1)

    n_full = full[0] * full[1]
    n_guard = guard[0] * guard[1]
    n_train = n_full - n_guard
    if n_train <= 0:
        raise ValueError("CFAR training region is empty; increase cfar_train_*")

    sum_full = ndimage.uniform_filter(ra_map, size=full, mode="nearest") * n_full
    sum_guard = ndimage.uniform_filter(ra_map, size=guard, mode="nearest") * n_guard
    noise = (sum_full - sum_guard) / n_train

    alpha = n_train * (cfg.pfa ** (-1.0 / n_train) - 1.0)
    return ra_map > alpha * noise


def cluster_detections(mask: np.ndarray, ra_map: np.ndarray, cfg):
    """Collapse each connected blob of CFAR hits to one power-weighted centroid detection.

    Returns (ranges_m, azimuths_rad), both 1-D, one entry per blob.

    A single physical target lights up several range and azimuth cells (the main lobe is
    wider than one cell by construction -- see the Hann window in range_fft). Counting
    those cells individually would multiply every detection count by the beam footprint
    and corrupt the phantom rate, so blobs are merged before anything downstream sees them.
    """
    if not mask.any():
        return np.empty(0), np.empty(0)
    labels, n = ndimage.label(mask)
    idx = np.arange(1, n + 1)
    weights = np.where(mask, ra_map, 0.0)
    cent = ndimage.center_of_mass(weights, labels, idx)    # [(az_idx, rng_idx), ...]
    az_i = np.array([c[0] for c in cent])
    rg_i = np.array([c[1] for c in cent])
    # fractional bin indices -> physical units by linear interpolation on the grids
    ranges = np.interp(rg_i, np.arange(cfg.n_range), cfg.range_bins())
    azimuths = np.interp(az_i, np.arange(cfg.n_azimuth), cfg.azimuth_grid())
    return ranges, azimuths
```

- [ ] **Step 4: Run the tests and watch them pass**

```bash
python3 -m pytest tests/test_radar_processing.py -v
```

Expected: 22 passed.

If `test_cfar_false_alarm_rate_on_pure_noise_is_near_pfa` fails high, the threshold is wrong —
do **not** raise `pfa` to make the test go green. A miscalibrated CFAR threshold would directly
corrupt RQ1's phantom rate, which is the paper's headline. Debug `alpha` and the box-filter
identity instead.

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/radar/processing.py tests/test_radar_processing.py
git commit -m "paper3(radar): 2-D CA-CFAR + connected-component detection clustering

The CFAR threshold is calibrated (alpha = N*(Pfa^(-1/N)-1)), not tuned -- RQ1's phantom
rate is only meaningful if it is. Clustering stops one target counting as many."
```

---

### Task 6: Detections → `Scan`, and the end-to-end chain test

**Files:**
- Modify: `src/wifi_radar_slam/radar/processing.py` (append)
- Test: `tests/test_radar_processing.py` (append)

**Interfaces:**
- Consumes: `cluster_detections` (Task 5), `lidar.pointcloud.Scan`.
- Produces:
  - `detections_to_scan(ranges, azimuths, cfg) -> Scan` — **monostatic** polar → Cartesian, range-gated to `[cfg.min_range_m, cfg.max_range_m]`.
  - `radar_scan(taus, amps, azimuths, cfg, rng=None) -> Scan` — the whole chain in one call. This is what `sensor.py` (Task 7) invokes.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_radar_processing.py`:

```python
from wifi_radar_slam.radar.processing import detections_to_scan, radar_scan
from wifi_radar_slam.lidar.pointcloud import Scan


def test_detections_to_scan_is_a_plain_polar_projection():
    # THE monostatic geometry. Contrast this with the bistatic case (cell A), where the
    # measured range is an AP->reflector->vehicle path length and recovering the
    # reflector needs an ill-conditioned ellipse solve. That asymmetry IS the geometry
    # ablation (A->B), which is why it lives in a named function and not inline.
    cfg = cfg_small()
    scan = detections_to_scan(np.array([10.0]), np.array([0.0]), cfg)
    assert isinstance(scan, Scan)
    assert np.allclose(scan.points, [[10.0, 0.0]])       # +x forward


def test_detections_to_scan_places_azimuth_correctly():
    cfg = cfg_small()
    scan = detections_to_scan(np.array([10.0]), np.array([np.pi / 2]), cfg)
    assert np.allclose(scan.points, [[0.0, 10.0]], atol=1e-9)   # +y is +90 deg


def test_detections_to_scan_gates_the_blind_zone_and_max_range():
    cfg = cfg_small()                       # min 1 m, max 60 m
    scan = detections_to_scan(np.array([0.3, 25.0, 200.0]),
                              np.array([0.0, 0.0, 0.0]), cfg)
    assert len(scan) == 1
    assert np.allclose(scan.points, [[25.0, 0.0]])


def test_detections_to_scan_on_no_detections_returns_an_empty_scan():
    cfg = cfg_small()
    scan = detections_to_scan(np.empty(0), np.empty(0), cfg)
    assert isinstance(scan, Scan) and len(scan) == 0


def test_radar_scan_end_to_end_recovers_two_known_reflectors():
    # THE acceptance test for the whole pure chain: two reflectors in, two points out,
    # each within a range cell and a beamwidth of the truth.
    cfg = cfg_small(noise=0.02)
    truth = [(15.0, np.deg2rad(-30.0)), (40.0, np.deg2rad(25.0))]
    scan = radar_scan([tau_of(r) for r, _ in truth],
                      [1.0 + 0j, 1.0 + 0j],
                      [a for _, a in truth], cfg,
                      rng=np.random.default_rng(0))
    assert len(scan) == 2
    for r, a in truth:
        expected = np.array([r * np.cos(a), r * np.sin(a)])
        d = np.linalg.norm(scan.points - expected, axis=1).min()
        assert d < 0.15 * r, f"no detection near ({r} m, {np.rad2deg(a):.0f} deg)"


def test_radar_scan_on_an_empty_scene_returns_few_or_no_points():
    # Sanity floor: pure noise must not manufacture a point cloud.
    cfg = cfg_small(noise=1.0)
    scan = radar_scan([], [], [], cfg, rng=np.random.default_rng(2))
    assert len(scan) <= 3


def test_a_multi_bounce_ray_becomes_a_ghost_at_the_wrong_range():
    # THE mechanism the paper is about. A double-bounce ray arrives with a LONGER delay
    # than the true reflector distance, so the chain -- which can only assume a
    # round trip -- places a detection too far away. Radar cannot tell the difference,
    # and neither could WiFi's MUSIC. Ghosts are not a bug in the chain; they are the
    # physics the chain is built to expose.
    cfg = cfg_small(noise=0.01)
    true_range = 20.0
    bounce_path_len = 55.0                 # a longer folded path off a second surface
    scan = radar_scan([tau_of(true_range), tau_of(bounce_path_len)],
                      [1.0 + 0j, 0.7 + 0j], [0.0, 0.0], cfg,
                      rng=np.random.default_rng(0))
    r = np.linalg.norm(scan.points, axis=1)
    assert np.any(np.abs(r - true_range) < 1.0), "the real reflector must be detected"
    assert np.any(np.abs(r - bounce_path_len) < 1.5), "the ghost must appear, uncorrected"
```

- [ ] **Step 2: Run the tests and watch them fail**

```bash
python3 -m pytest tests/test_radar_processing.py -v
```

Expected: `ImportError: cannot import name 'detections_to_scan'`.

- [ ] **Step 3: Implement**

Append to `src/wifi_radar_slam/radar/processing.py` (and add `from ..lidar.pointcloud import Scan`
to the imports at the top of the file):

```python
def detections_to_scan(ranges: np.ndarray, azimuths: np.ndarray, cfg) -> Scan:
    """Monostatic polar -> Cartesian: place each detection in the sensor-local frame.

    Returns a Scan (+x forward), range-gated to [cfg.min_range_m, cfg.max_range_m].

    This is trivially simple *because the geometry is monostatic*: TX and RX are
    co-located, so the measured delay is an honest round trip and range = tau*c/2 projects
    straight out along the measured bearing.

    The bistatic case (cell A -- ambient WiFi) has no such luxury: the measured quantity is
    a **path length** AP -> reflector -> vehicle, whose locus is an *ellipse* with the AP
    and the vehicle at its foci, and pinning the reflector on that ellipse needs an angle
    estimate whose error is amplified by the ellipse's eccentricity. That asymmetry is not
    an implementation detail -- it is precisely what the A->B ablation isolates, and it is
    why paper 2's WiFi maps carried a 6.45 m range bias while the resolution limit was
    0.94 m.
    """
    ranges = np.asarray(ranges, dtype=float).ravel()
    azimuths = np.asarray(azimuths, dtype=float).ravel()
    if ranges.size == 0:
        return Scan.empty()
    keep = (ranges >= cfg.min_range_m) & (ranges <= cfg.max_range_m)
    r, a = ranges[keep], azimuths[keep]
    if r.size == 0:
        return Scan.empty()
    return Scan(np.stack([r * np.cos(a), r * np.sin(a)], axis=1))


def radar_scan(taus, amps, azimuths, cfg, rng=None) -> Scan:
    """The complete chain: rays -> beat signal -> range FFT -> beamform -> CFAR -> Scan.

    This is the single entry point sensor.py uses. Every ablation cell calls exactly this
    function with exactly this detection chain -- only cfg (carrier, bandwidth) and the ray
    set differ -- which is what makes the decomposition attributable to physics.
    """
    beat = beat_matrix(taus, amps, azimuths, cfg, rng=rng)
    ra = azimuth_beamform(range_fft(beat, cfg), cfg)
    ranges, az = cluster_detections(cfar_2d(ra, cfg), ra, cfg)
    return detections_to_scan(ranges, az, cfg)
```

- [ ] **Step 4: Run the tests and watch them pass**

```bash
python3 -m pytest tests/test_radar_processing.py -v
```

Expected: 29 passed.

- [ ] **Step 5: Run the full suite — nothing existing may break**

```bash
python3 -m pytest -q
```

Expected: 85 prior tests + the new radar tests, all passing, 0 failures.

- [ ] **Step 6: Commit**

```bash
git add src/wifi_radar_slam/radar/processing.py tests/test_radar_processing.py
git commit -m "paper3(radar): detections -> Scan, and the end-to-end pure chain

Monostatic projection is a plain polar->Cartesian; the bistatic ellipse solve it is NOT
is exactly what the A->B geometry ablation isolates. Includes the ghost test: a
multi-bounce ray lands at the wrong range, uncorrected, by construction."
```

---

### Task 7: `SionnaRadarSensor` on the `make_sensor` seam

**Files:**
- Create: `src/wifi_radar_slam/radar/sensor.py`
- Test: `tests/test_radar_sensor.py`

**Interfaces:**
- Consumes: `radar_scan` (Task 6), `RadarConfig` (Task 1), `geometry.RX_HEIGHT_M`.
- Produces:
  - `paths_to_rays(tau, a, phi_r, yaw) -> tuple[np.ndarray, np.ndarray, np.ndarray]` — pure, locally tested: `(taus, amps, local_azimuths)` ready for `radar_scan`.
  - `SionnaRadarSensor(built, cfg, rng, scattering=0.7, max_depth=3)` — callable `(pose) -> Scan`.
  - `radar_sensor(built, cfg, rng) -> SionnaRadarSensor` — the `make_sensor` factory, matching the signature `lidar/runner.py:8` expects.

**On the three Sionna pitfalls this task must handle:**

- **Pitfall #1 (`normalize_delays=True` is the default and destroys absolute range).** We **never call `cfr()`, `cir()` or `taps()`.** We read `paths.tau` and `paths.a` directly and synthesize the beat signal ourselves — and `paths.tau` is *absolute* (verified 2026-07-12: LOS delay × c matched true distance to 3 dp). So the pitfall is structurally impossible here rather than merely avoided. Do not "fix" this by switching to `cfr(normalize_delays=False)`; that would be a regression.
- **Pitfall #2 (specular-only surfaces make monostatic 77 GHz radar blind).** `diffuse_reflection=True` on the solver **and** a non-zero `scattering_coefficient` on every material are both required. Paper 2 measured the difference on this exact scene: specular-only → **1** return; diffuse → **8,417**. Without this the sensor returns an empty scan and looks "broken" for a reason that is physical, not a bug.
- **Pitfall #5 (monostatic co-located TX/RX changes the angle convention).** This one **cannot be resolved by reading the docs** — it is a known Sionna quirk (NVlabs/sionna-rt#5). It is resolved *empirically* by Task 9, which runs a known geometry on the server and checks that the detected bearing matches the true bearing to a real wall. Until Task 9 passes, `paths_to_rays`'s azimuth convention is a **hypothesis**, and the code says so.

- [ ] **Step 1: Write the failing test**

Create `tests/test_radar_sensor.py`:

```python
import numpy as np
import pytest
from wifi_radar_slam.radar.sensor import paths_to_rays

C = 299792458.0


def test_paths_to_rays_passes_delays_and_amplitudes_through_untouched():
    # tau must stay ABSOLUTE. Sionna's cfr()/cir() would zero the first-path delay
    # (normalize_delays defaults to True) and destroy range; we read paths.tau instead,
    # so the delays that arrive here are already absolute and must not be rescaled.
    tau = np.array([1e-7, 2e-7])
    a = np.array([1 + 0j, 0.5 + 0.5j])
    taus, amps, az = paths_to_rays(tau, a, np.array([0.0, 0.0]), yaw=0.0)
    assert np.allclose(taus, tau)
    assert np.allclose(amps, a)


def test_paths_to_rays_rotates_world_azimuth_into_the_sensor_frame():
    # Sionna reports arrival azimuth in the WORLD frame; a Scan is sensor-local
    # (+x forward). The vehicle's yaw must therefore be subtracted.
    tau = np.array([1e-7])
    a = np.array([1 + 0j])
    phi_world = np.array([np.deg2rad(90.0)])
    _, _, az = paths_to_rays(tau, a, phi_world, yaw=np.deg2rad(30.0))
    assert np.rad2deg(az[0]) == pytest.approx(60.0)


def test_paths_to_rays_wraps_azimuth_to_minus_pi_pi():
    tau = np.array([1e-7])
    a = np.array([1 + 0j])
    _, _, az = paths_to_rays(tau, a, np.array([np.deg2rad(170.0)]),
                             yaw=np.deg2rad(-170.0))
    # 170 - (-170) = 340 deg, which must wrap to -20 deg, not stay at 340
    assert np.rad2deg(az[0]) == pytest.approx(-20.0, abs=1e-6)
    assert -np.pi <= az[0] <= np.pi


def test_paths_to_rays_drops_invalid_and_nonfinite_paths():
    tau = np.array([1e-7, np.nan, 3e-7])
    a = np.array([1 + 0j, 1 + 0j, 0 + 0j])          # third has zero amplitude
    phi = np.array([0.0, 0.0, 0.0])
    taus, amps, az = paths_to_rays(tau, a, phi, yaw=0.0)
    assert len(taus) == 1                            # nan dropped, zero-amp dropped
    assert taus[0] == pytest.approx(1e-7)


def test_paths_to_rays_on_an_empty_path_set():
    taus, amps, az = paths_to_rays(np.empty(0), np.empty(0, dtype=complex),
                                   np.empty(0), yaw=0.0)
    assert len(taus) == 0 and len(amps) == 0 and len(az) == 0


def test_sensor_module_imports_without_sionna():
    # The whole point of the lazy-import pattern: this module must be importable on a
    # machine that has never seen Sionna (this laptop). Only *running* the sensor needs it.
    import wifi_radar_slam.radar.sensor as s
    assert hasattr(s, "SionnaRadarSensor") and hasattr(s, "radar_sensor")
```

- [ ] **Step 2: Run the tests and watch them fail**

```bash
python3 -m pytest tests/test_radar_sensor.py -v
```

Expected: `ModuleNotFoundError: No module named 'wifi_radar_slam.radar.sensor'`.

- [ ] **Step 3: Implement**

Create `src/wifi_radar_slam/radar/sensor.py`:

```python
"""Sionna-ray-traced 77 GHz FMCW radar sensor (monostatic + diffuse scattering).

`paths_to_rays` is pure NumPy and tests locally. `SionnaRadarSensor` lazily imports
sionna.rt/mitsuba *inside* its methods, so this module imports fine without Sionna and
only *running* the sensor needs the amd server -- the same pattern as
lidar/sensor_sionna.py.
"""
from __future__ import annotations
import numpy as np
from ..geometry import RX_HEIGHT_M
from ..lidar.pointcloud import Scan
from .processing import radar_scan


def paths_to_rays(tau, a, phi_r, yaw: float):
    """Sionna path arrays -> (taus, amps, sensor-local azimuths) for the radar chain.

    Args:
        tau:   absolute propagation delays (s), one per path. **Absolute**: these come from
               `paths.tau`, NOT from cfr()/cir()/taps(), whose `normalize_delays` defaults
               to True and would zero the first-path delay -- destroying the very quantity
               a radar measures. Reading paths.tau makes that pitfall structurally
               impossible rather than merely avoided.
        a:     complex path amplitudes.
        phi_r: azimuth of arrival at the receiver (rad), in the WORLD frame.
        yaw:   vehicle heading (rad).

    Returns:
        (taus, amps, azimuths) with azimuths in the SENSOR-LOCAL frame (+x forward),
        wrapped to [-pi, pi], with non-finite and zero-amplitude paths dropped.

    NOTE (unresolved until the server validation in experiments/validate_radar_sensor.py):
    Sionna is known to change its angle convention when TX and RX are co-located, as they
    are in a monostatic radar (NVlabs/sionna-rt#5). The `phi_r - yaw` mapping below is the
    documented convention and is our working hypothesis; it is *verified empirically*
    against a known wall geometry by that script. If the validation shows a mirrored or
    offset bearing, fix it HERE -- never compensate downstream, or the ablation's geometry
    axis becomes uninterpretable.
    """
    tau = np.asarray(tau, dtype=float).ravel()
    a = np.asarray(a, dtype=complex).ravel()
    phi_r = np.asarray(phi_r, dtype=float).ravel()
    ok = np.isfinite(tau) & np.isfinite(phi_r) & np.isfinite(a) & (np.abs(a) > 0)
    tau, a, phi_r = tau[ok], a[ok], phi_r[ok]
    az = np.arctan2(np.sin(phi_r - yaw), np.cos(phi_r - yaw))     # world -> local, wrapped
    return tau, a, az


class SionnaRadarSensor:
    """Monostatic 77 GHz FMCW radar: a TX co-located with the vehicle RX, diffuse material
    backscatter, and the FULL detection chain (beat -> range FFT -> beamform -> CFAR).

    The chain is not an implementation choice, it is the experiment. Reading interaction
    vertices straight out of the ray tracer -- which is what the LiDAR model B does -- would
    give radar zero ghosts by construction and rig RQ1, the paper's headline question. Every
    ghost and false alarm this sensor produces must earn its way through finite bandwidth, a
    finite aperture and a calibrated CFAR threshold.
    """

    def __init__(self, built, cfg, rng, scattering: float = 0.7, max_depth: int = 3):
        import sionna.rt as rt                  # lazy: server only
        self.built, self.cfg, self.rng = built, cfg, rng
        self.max_depth = max_depth
        self.scene = built.scene
        self.scene.frequency = cfg.carrier_hz   # the ablation's carrier axis
        if "radar_tx" not in self.scene.transmitters:
            self.scene.add(rt.Transmitter("radar_tx",
                                          position=[0.0, 0.0, RX_HEIGHT_M]))
        # Diffuse backscatter is REQUIRED. With specular-only materials a monostatic
        # 77 GHz radar is very nearly blind: on this exact scene paper 2 measured
        # 1 return specular vs 8,417 with diffuse enabled.
        for m in self.scene.radio_materials.values():
            try:
                m.scattering_coefficient = scattering
            except Exception:
                pass
        self.solver = rt.PathSolver()
        self.tidx = list(self.scene.transmitters.keys()).index("radar_tx")
        self.rx = self.scene.receivers["veh"]

    def __call__(self, pose) -> Scan:
        import os
        import mitsuba as mi
        px, py = float(pose[0]), float(pose[1])
        yaw = float(pose[2]) if len(pose) > 2 else 0.0
        # co-locate TX and RX: this is what makes the sensor monostatic
        self.scene.transmitters["radar_tx"].position = mi.Point3f(px, py, RX_HEIGHT_M)
        self.rx.position = mi.Point3f(px, py, RX_HEIGHT_M)
        ns = int(os.environ.get("WRS_NUM_SAMPLES", "1000000"))
        paths = self.solver(self.scene, max_depth=self.max_depth, samples_per_src=ns,
                            diffuse_reflection=True,
                            seed=int(self.rng.integers(1, 2**31 - 1)))
        # absolute delays and complex amplitudes -- NOT cfr()/cir(), see paths_to_rays
        tau = np.asarray(paths.tau.numpy())[0, self.tidx]        # (n_paths,)
        a = np.asarray(paths.a.numpy())
        a = (a[0] + 1j * a[1])[0, 0, self.tidx, 0] if a.ndim > 4 else a.ravel()
        phi = np.asarray(paths.phi_r.numpy())[0, self.tidx]      # (n_paths,) world azimuth
        taus, amps, az = paths_to_rays(tau, a, phi, yaw)
        return radar_scan(taus, amps, az, self.cfg, rng=self.rng)


def radar_sensor(built, cfg, rng) -> "SionnaRadarSensor":
    """make_sensor factory: matches the seam lidar/runner.run_lidar already expects,
    so the radar drops into the SAME scan-to-map ICP back-end with no changes to it."""
    return SionnaRadarSensor(built, cfg, rng)
```

- [ ] **Step 4: Run the tests and watch them pass**

```bash
python3 -m pytest tests/test_radar_sensor.py -v
```

Expected: 6 passed.

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/radar/sensor.py tests/test_radar_sensor.py
git commit -m "paper3(radar): Sionna monostatic radar sensor on the make_sensor seam

Reads paths.tau/paths.a directly -- never cfr() -- so the normalize_delays pitfall is
structurally impossible. Diffuse scattering is mandatory (specular-only: 1 return vs
8,417). The monostatic angle convention is a HYPOTHESIS until the server validation."
```

**Note on the `paths.a` unpacking:** the shape of `paths.a` in Sionna RT 2.0.1 is the one thing
here that cannot be confirmed without the simulator. The line above handles the documented
`(2, ...)` real/imag layout with a fallback. **Task 9 is what confirms it** — expect to correct
this line once, against the real array shape printed by the validation script. That is a normal
part of this task, not a failure of it.

---

### Task 8: KITTI-protocol drift

**Files:**
- Create: `src/wifi_radar_slam/eval/drift.py`
- Test: `tests/test_drift.py`

**Interfaces:**
- Consumes: nothing (pure NumPy).
- Produces:
  - `path_lengths(traj) -> np.ndarray` — cumulative arc length, shape `(n,)`, starting at 0.
  - `drift(est_traj, gt_traj, lengths=(100,200,300,400,500,600,700,800), step=10) -> dict` with keys `trans_pct` (float, % — NaN if no segments), `rot_deg_per_100m` (float — NaN if no segments), `n_segments` (int), `per_length` (dict `{length: (trans_pct, rot_deg_per_100m)}`).

- [ ] **Step 1: Write the failing test**

Create `tests/test_drift.py`:

```python
import numpy as np
import pytest
from wifi_radar_slam.eval.drift import path_lengths, drift


def straight(n=200, dx=5.0):
    """A 1 km straight-line trajectory: n frames, dx metres apart, heading +x."""
    t = np.zeros((n, 3))
    t[:, 0] = np.arange(n) * dx
    return t


def test_path_lengths_starts_at_zero_and_accumulates():
    cum = path_lengths(straight(n=5, dx=5.0))
    assert np.allclose(cum, [0.0, 5.0, 10.0, 15.0, 20.0])


def test_perfect_estimate_has_zero_drift():
    gt = straight()
    d = drift(gt, gt)
    assert d["n_segments"] > 0
    assert d["trans_pct"] == pytest.approx(0.0, abs=1e-9)
    assert d["rot_deg_per_100m"] == pytest.approx(0.0, abs=1e-9)


def test_a_constant_scale_error_gives_the_expected_drift_percent():
    # An estimate that travels 1% too far accrues exactly 1% translational drift,
    # at EVERY sub-sequence length. This is the calibration test for the whole metric.
    gt = straight()
    est = gt.copy()
    est[:, 0] *= 1.01
    d = drift(gt=gt, est_traj=est)
    assert d["trans_pct"] == pytest.approx(1.0, abs=1e-6)


def test_drift_is_invariant_to_a_global_rigid_transform():
    # Drift is a metric on RELATIVE motion, which is exactly why radar odometry uses it:
    # unlike ATE, it does not care where the trajectory sits in the world.
    gt = straight()
    th = np.deg2rad(37.0)
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    moved = gt.copy()
    moved[:, :2] = gt[:, :2] @ R.T + np.array([100.0, -50.0])
    moved[:, 2] = gt[:, 2] + th
    d_ref = drift(gt, gt)
    d_moved = drift(moved, moved)
    assert d_moved["trans_pct"] == pytest.approx(d_ref["trans_pct"], abs=1e-9)


def test_a_yaw_bias_shows_up_as_rotational_drift():
    gt = straight()
    est = gt.copy()
    est[:, 2] += np.deg2rad(1.0)              # constant 1 deg heading error
    d = drift(gt, est)
    assert d["rot_deg_per_100m"] > 0.0


def test_short_trajectory_returns_nan_not_a_fabricated_number():
    # OUR SIMULATED TRAJECTORIES ARE 30-60 m. KITTI's shortest sub-sequence is 100 m, so
    # standard drift is UNDEFINED on them. It must say so, loudly, rather than quietly
    # averaging over zero segments or silently shrinking the window.
    gt = straight(n=10, dx=5.0)               # 45 m total
    d = drift(gt, gt)
    assert d["n_segments"] == 0
    assert np.isnan(d["trans_pct"])
    assert np.isnan(d["rot_deg_per_100m"])


def test_reduced_lengths_work_on_short_trajectories_when_asked_for_explicitly():
    # Sub-project 3 may report drift at reduced lengths on the simulated cells -- but
    # only because it passed `lengths` explicitly and labelled the result. The default
    # never silently degrades.
    gt = straight(n=10, dx=5.0)               # 45 m total
    d = drift(gt, gt, lengths=(10, 20, 30))
    assert d["n_segments"] > 0
    assert d["trans_pct"] == pytest.approx(0.0, abs=1e-9)


def test_per_length_breakdown_is_reported():
    gt = straight()
    d = drift(gt, gt)
    assert set(d["per_length"]) <= {100, 200, 300, 400, 500, 600, 700, 800}
    assert all(len(v) == 2 for v in d["per_length"].values())
```

- [ ] **Step 2: Run the tests and watch them fail**

```bash
python3 -m pytest tests/test_drift.py -v
```

Expected: `ModuleNotFoundError: No module named 'wifi_radar_slam.eval.drift'`.

- [ ] **Step 3: Implement**

Create `src/wifi_radar_slam/eval/drift.py`:

```python
"""KITTI-protocol odometry drift: translational % and rotational deg/100 m.

This is the metric the radar-odometry literature is actually scored on -- CFEAR reports
1.09 % on Oxford, DRO 0.26 % on Boreas -- and a radar paper that reported only ATE would be
marked down. It measures error over sub-sequences of fixed *length*, so it is invariant to
where the trajectory sits in the world and does not let one early blunder dominate, which is
exactly the failure mode ATE has on long drives.

SCOPE, AND A HARD EDGE. The standard sub-sequence lengths are 100-800 m. Our SIMULATED
trajectories are 30-60 m, so standard drift is mathematically undefined on them: there is
not one 100 m window to measure. `drift()` therefore reports NaN with n_segments=0 rather
than fabricating a value. The standard lengths are used on the real-radar credibility anchor
(Oxford/Boreas), where they are directly comparable to the published CFEAR/DRO numbers;
simulated cells report ATE/RPE + the map metrics, as in paper 2.
"""
from __future__ import annotations
import numpy as np


def path_lengths(traj: np.ndarray) -> np.ndarray:
    """Cumulative arc length (m) along a trajectory (n, >=2). Starts at 0."""
    xy = np.asarray(traj, dtype=float)[:, :2]
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


def _se2(pose):
    """(x, y, yaw) -> 3x3 homogeneous SE(2) matrix."""
    x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def _se2_inv(T):
    R, t = T[:2, :2], T[:2, 2]
    out = np.eye(3)
    out[:2, :2] = R.T
    out[:2, 2] = -R.T @ t
    return out


def _last_frame_from(cum: np.ndarray, first: int, length: float) -> int:
    """First index j > first with cum[j] - cum[first] >= length, or -1 if none."""
    reach = np.nonzero(cum[first:] - cum[first] >= length)[0]
    return int(first + reach[0]) if reach.size else -1


def drift(gt: np.ndarray, est_traj: np.ndarray,
          lengths=(100, 200, 300, 400, 500, 600, 700, 800),
          step: int = 10) -> dict:
    """KITTI-protocol drift of `est_traj` against ground truth `gt`, both (n, 3) (x,y,yaw).

    For every start frame (each `step`-th) and every sub-sequence `length`, compare the
    relative motion the estimate claims against the relative motion the ground truth made:

        E = inv(gt_delta) @ est_delta

    translational error = ||E_t|| / length          -> reported as a percentage
    rotational error    = |E_yaw| / length          -> reported as deg / 100 m

    and average over every segment. Returns
        {"trans_pct", "rot_deg_per_100m", "n_segments", "per_length": {L: (t%, r)}}
    with NaN for the two averages when no segment of any requested length fits in the
    trajectory (see this module's docstring -- that is a real case for us, not a corner one).
    """
    gt = np.asarray(gt, dtype=float)
    est_traj = np.asarray(est_traj, dtype=float)
    if gt.shape != est_traj.shape or gt.shape[1] < 3:
        raise ValueError(f"need matching (n,3) trajectories, got {gt.shape}, {est_traj.shape}")

    cum = path_lengths(gt)
    per_length: dict[int, tuple[float, float]] = {}
    all_t: list[float] = []
    all_r: list[float] = []

    for L in lengths:
        t_errs: list[float] = []
        r_errs: list[float] = []
        for first in range(0, len(gt), step):
            last = _last_frame_from(cum, first, float(L))
            if last < 0:
                continue
            gt_d = _se2_inv(_se2(gt[first])) @ _se2(gt[last])
            est_d = _se2_inv(_se2(est_traj[first])) @ _se2(est_traj[last])
            E = _se2_inv(gt_d) @ est_d
            t_errs.append(float(np.linalg.norm(E[:2, 2])) / L)
            r_errs.append(abs(float(np.arctan2(E[1, 0], E[0, 0]))) / L)
        if t_errs:
            per_length[int(L)] = (100.0 * float(np.mean(t_errs)),
                                  100.0 * np.rad2deg(float(np.mean(r_errs))))
            all_t.extend(t_errs)
            all_r.extend(r_errs)

    if not all_t:                       # trajectory too short for any requested length
        return {"trans_pct": float("nan"), "rot_deg_per_100m": float("nan"),
                "n_segments": 0, "per_length": {}}

    return {
        "trans_pct": 100.0 * float(np.mean(all_t)),                  # % of distance travelled
        "rot_deg_per_100m": 100.0 * np.rad2deg(float(np.mean(all_r))),
        "n_segments": len(all_t),
        "per_length": per_length,
    }
```

- [ ] **Step 4: Run the tests and watch them pass**

```bash
python3 -m pytest tests/test_drift.py -v
```

Expected: 8 passed.

Note the argument order: `drift(gt, est_traj)` — ground truth first, matching `eval/metrics.py`'s
convention of `(est, gt)`… **no, it does not.** `metrics.ate(est_traj, gt_traj)` puts *est* first.
Make `drift` match: change the signature to `drift(est_traj, gt, ...)` and update the tests'
call sites accordingly. Consistency across the eval package matters more than any local
preference, and a silently swapped pair of trajectories is the kind of bug that produces a
plausible-looking wrong number.

- [ ] **Step 5: Fix the argument order to match `eval/metrics.py`, then re-run**

Signature becomes `drift(est_traj, gt, lengths=..., step=10)`. Update every call in
`tests/test_drift.py` (they currently pass `gt` first).

```bash
python3 -m pytest tests/test_drift.py -v
```

Expected: 8 passed.

- [ ] **Step 6: Commit**

```bash
git add src/wifi_radar_slam/eval/drift.py tests/test_drift.py
git commit -m "paper3(eval): KITTI-protocol drift (translational %, rotational deg/100m)

The metric radar odometry is actually scored on (CFEAR 1.09%, DRO 0.26%). Reports NaN
rather than a fabricated number when the trajectory is shorter than the shortest
sub-sequence -- which is the case for our 30-60 m simulated runs, so drift is scoped to
the real-data anchor."
```

---

### Task 9: Server validation — the sub-project's gate

**Files:**
- Create: `experiments/validate_radar_sensor.py`
- Create: `docs/results-paper3-radar-substrate.md` (the artifact the numbers land in)

This is the task that turns three hypotheses into facts: that the monostatic angle convention is
what we assumed (pitfall #5), that `paths.a`'s array layout is what we assumed, and that diffuse
scattering makes a monostatic 77 GHz radar see anything at all (pitfall #2). **None of it can be
tested locally.** Run on `amd`.

- [ ] **Step 1: Write the validation script**

Create `experiments/validate_radar_sensor.py`:

```python
"""Validate the Sionna radar sensor against a KNOWN geometry. Server-only (needs Sionna).

Three things are unverifiable without the simulator, and each would silently corrupt the
paper if wrong:

  1. Sionna's angle convention when TX and RX are co-located (NVlabs/sionna-rt#5). If the
     bearing is mirrored, every reflector lands on the wrong side of the road and the A->B
     geometry ablation becomes uninterpretable.
  2. The array layout of `paths.a` in Sionna RT 2.0.1.
  3. That diffuse scattering is genuinely required at 77 GHz monostatic (paper 2 measured
     1 specular return vs 8,417 diffuse -- confirm it reproduces for radar).

Run:  WRS_NUM_SAMPLES=1000000 python3 experiments/validate_radar_sensor.py
"""
from __future__ import annotations
import json
import numpy as np
from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.radar.config import RADAR_77G_4G
from wifi_radar_slam.radar.sensor import SionnaRadarSensor, paths_to_rays
from wifi_radar_slam.radar.processing import radar_scan

C = 299792458.0


def main() -> None:
    cfg_run = load_config("configs/nominal.yaml")
    built = build_scene(cfg_run)
    rng = np.random.default_rng(0)
    pose = built.trajectory[len(built.trajectory) // 2]     # mid-trajectory
    print(f"pose = {pose}")

    sensor = SionnaRadarSensor(built, RADAR_77G_4G, rng)

    # --- raw path inspection: print the shapes BEFORE trusting any unpacking ----------
    import mitsuba as mi
    import os
    px, py = float(pose[0]), float(pose[1])
    from wifi_radar_slam.geometry import RX_HEIGHT_M
    sensor.scene.transmitters["radar_tx"].position = mi.Point3f(px, py, RX_HEIGHT_M)
    sensor.rx.position = mi.Point3f(px, py, RX_HEIGHT_M)
    ns = int(os.environ.get("WRS_NUM_SAMPLES", "1000000"))

    for diffuse in (False, True):
        paths = sensor.solver(sensor.scene, max_depth=3, samples_per_src=ns,
                              diffuse_reflection=diffuse, seed=1)
        tau_raw = np.asarray(paths.tau.numpy())
        a_raw = np.asarray(paths.a.numpy())
        phi_raw = np.asarray(paths.phi_r.numpy())
        n_valid = int(np.count_nonzero(np.asarray(paths.valid.numpy())))
        print(f"\n--- diffuse_reflection={diffuse} ---")
        print(f"  tau   shape {tau_raw.shape}")
        print(f"  a     shape {a_raw.shape}  dtype {a_raw.dtype}")
        print(f"  phi_r shape {phi_raw.shape}")
        print(f"  valid paths: {n_valid}")          # PITFALL #2: expect ~0 vs thousands

    # --- the geometry check ----------------------------------------------------------
    # The nearest ground-truth surface point tells us where the strongest return SHOULD be.
    gt = built.ground_truth_map[:, :2]
    rel = gt - np.array([px, py])
    d = np.linalg.norm(rel, axis=1)
    near = int(np.argmin(d))
    true_range = float(d[near])
    yaw = float(pose[2]) if len(pose) > 2 else 0.0
    true_bearing = float(np.arctan2(rel[near, 1], rel[near, 0]) - yaw)
    true_bearing = float(np.arctan2(np.sin(true_bearing), np.cos(true_bearing)))
    print(f"\nnearest GT surface: range {true_range:.2f} m, "
          f"bearing {np.rad2deg(true_bearing):+.1f} deg")

    scan = sensor(pose)
    print(f"radar scan: {len(scan)} detections")
    if len(scan) == 0:
        raise SystemExit("FAIL: no detections. Check diffuse scattering (pitfall #2).")

    r = np.linalg.norm(scan.points, axis=1)
    b = np.arctan2(scan.points[:, 1], scan.points[:, 0])
    # is ANY detection near the true nearest surface?
    err = np.hypot(r * np.cos(b) - true_range * np.cos(true_bearing),
                   r * np.sin(b) - true_range * np.sin(true_bearing))
    best = int(np.argmin(err))
    print(f"closest detection to that surface: range {r[best]:.2f} m, "
          f"bearing {np.rad2deg(b[best]):+.1f} deg, error {err[best]:.2f} m")

    # PITFALL #5: if the bearing is MIRRORED, the error against -true_bearing will be
    # far smaller than against +true_bearing. Check explicitly, do not guess.
    err_mirror = np.hypot(r * np.cos(b) - true_range * np.cos(-true_bearing),
                          r * np.sin(b) - true_range * np.sin(-true_bearing))
    print(f"  best error, assumed convention : {err[best]:.2f} m")
    print(f"  best error, MIRRORED convention: {err_mirror.min():.2f} m")
    if err_mirror.min() < 0.5 * err[best]:
        print("  *** ANGLE CONVENTION IS MIRRORED -- fix paths_to_rays, "
              "not anything downstream ***")

    out = {
        "true_range_m": true_range,
        "true_bearing_deg": float(np.rad2deg(true_bearing)),
        "n_detections": int(len(scan)),
        "best_range_m": float(r[best]),
        "best_bearing_deg": float(np.rad2deg(b[best])),
        "best_error_m": float(err[best]),
        "best_error_mirrored_m": float(err_mirror.min()),
    }
    print("\n" + json.dumps(out, indent=2))
    with open("results/radar_substrate_validation.json", "w") as f:
        json.dump(out, f, indent=2)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Sync to the server and run it**

```bash
# from the repo root, with the branch pushed
git push -u origin paper3-sub1-radar-substrate
```

Then on `amd` (`/home/dev/mulham/wifi-radar-slam`), throttled so other services are undisturbed:

```bash
cd /home/dev/mulham/wifi-radar-slam
git fetch origin && git checkout paper3-sub1-radar-substrate && git pull --ff-only
mkdir -p results
nice -n 19 ionice -c3 .venv/bin/python experiments/validate_radar_sensor.py 2>&1 | tee results/radar_substrate_validation.log
```

- [ ] **Step 3: Act on what it printed — this is the point of the task**

Three outcomes, each with a required action:

| What the script shows | What it means | What to do |
|---|---|---|
| `diffuse=False` gives ~0 valid paths, `diffuse=True` gives thousands | Pitfall #2 confirmed for radar | Record the two counts in the results doc. Nothing to fix. |
| The `a` / `tau` / `phi_r` shapes differ from what `sensor.py` unpacks | Our unpacking guess was wrong | **Fix `SionnaRadarSensor.__call__`** to match the real shapes, re-run. Expected — see the note at the end of Task 7. |
| `best error, MIRRORED convention` is much smaller than the assumed one | Pitfall #5 has bitten; Sionna mirrors azimuth for co-located TX/RX | **Fix `paths_to_rays`** (negate the azimuth). Never compensate downstream — the geometry axis of the ablation would become uninterpretable. |

The gate: **a detection must land within ~1 m of the true nearest ground-truth surface.** If it
does not, and the mirror check does not explain it, stop and diagnose before building anything on
top of this sensor. A radar that points the wrong way would produce a beautifully self-consistent,
completely wrong paper.

- [ ] **Step 4: Record the numbers in a committed artifact**

Create `docs/results-paper3-radar-substrate.md` with the real printed values — the specular vs
diffuse path counts, the true vs detected range and bearing, the resolved angle convention, and
the `paths.a` layout. Every one of these is a fact a later task or a reviewer will need, and none
of them are re-derivable without the server.

- [ ] **Step 5: Run the full suite one last time**

```bash
python3 -m pytest -q
```

Expected: all tests pass (85 prior + ~43 new).

- [ ] **Step 6: Commit and push**

```bash
git add experiments/validate_radar_sensor.py docs/results-paper3-radar-substrate.md \
        src/wifi_radar_slam/radar/sensor.py
git commit -m "paper3(radar): server validation of the monostatic sensor

Resolves empirically what the docs cannot: the co-located TX/RX angle convention
(sionna-rt#5), the paths.a array layout, and that diffuse scattering is required for a
77 GHz monostatic radar to see anything at all."
git push
```

---

## Definition of done for sub-project 1

- [ ] `radar/` exists: `RadarConfig` + the three cell presets, the full pure detection chain, the Sionna monostatic sensor on the `make_sensor` seam.
- [ ] `eval/drift.py` exists, KITTI-protocol, honest about short trajectories.
- [ ] The pure chain is unit-tested locally end to end, including **the ghost mechanism** (a multi-bounce ray lands at the wrong range, uncorrected).
- [ ] The Sionna sensor is validated on the server against a known wall: a detection lands within ~1 m of the true nearest surface, and the angle convention is *measured*, not assumed.
- [ ] The full suite is green.
- [ ] Merge `paper3-sub1-radar-substrate` → `paper3-wifi-vs-radar`, tag `paper3-v0.1.0`.

**Then, and only then, sub-project 2 — the credibility anchor — which is a GATE on the whole paper.**

---

## Self-review of this plan

**Spec coverage.** The spec's sub-project-1 row asks for: `radar/` (RadarConfig, pure-NumPy chain, Sionna monostatic sensor on the `make_sensor` seam) + `eval/drift.py`, unit-tested, sensor gated. Task 1 → RadarConfig; Tasks 2–6 → the chain; Task 7 → the sensor; Task 8 → drift; Task 9 → the server gate. The spec's six pitfalls: #1 (`normalize_delays`) is handled structurally in Task 7 by never calling `cfr()`; #2 (diffuse required) in Tasks 7 and 9; #3 (`doppler` is zero without velocity vectors) is moot — the range–azimuth decision means we never read `doppler`; #4 (synthetic time evolution) is *retired* by the analytic chirp axis in Task 2 and this is written down rather than assumed; #5 (monostatic angle convention) is an explicit hypothesis in Task 7 and is *measured* in Task 9; #6 (frequency-flat amplitudes) is a stated paper limitation, not code, and needs no task. The bistatic-vs-monostatic asymmetry the spec insists must be "reported as a mechanism, not hidden inside a helper function" is documented at length in `detections_to_scan`.

**Two amendments to the spec, both made in Task 0 rather than left to drift:** the architecture block's `beat_cube`/`range_doppler` predated the range–azimuth decision, and KITTI drift is undefined on our 30–60 m simulated trajectories.

**Type consistency.** `RadarConfig` field names are used identically in Tasks 2–7. `radar_scan(taus, amps, azimuths, cfg, rng)` is defined in Task 6 and called in Task 7 with that exact signature. `Scan` is the existing `lidar.pointcloud.Scan` throughout — never a new type. `drift`'s argument order is deliberately corrected in Task 8 Step 5 to match `eval/metrics.py`'s `(est, gt)`.
