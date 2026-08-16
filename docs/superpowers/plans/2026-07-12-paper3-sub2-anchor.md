# Paper 3 · Sub-project 2 — Radar Credibility Anchor (THE GATE)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prove our shared scan-to-map ICP back-end is a *credible* radar baseline by running it on **real spinning-radar data (Boreas)** and reporting KITTI-protocol drift % beside the cited SOTA — or discover that it is not, and bound the paper's claims accordingly.

**Architecture:** Add a k-strongest-per-azimuth front-end (the CFEAR-style extractor) and a Boreas polar-radar loader. Both emit the *existing* `lidar.pointcloud.Scan`, so the *existing* `lidar/slam_icp.run_lidar_slam` runs on real radar **completely unchanged** — which is the entire point: any difference between sensors must be attributable to the sensor, not to the estimator. Score with `eval/drift.py` at the standard KITTI 100–800 m lengths, which are valid here because Boreas sequences are kilometre-scale.

**Tech Stack:** Python 3, NumPy, SciPy, Pillow (PNG decode — already installed on the server), `concurrent.futures` for parallel download/load.

**Test runner:** `.venv/bin/python -m pytest` (a bare `python3` has no venv and cannot import the package).

**Branch:** `paper3-sub2-anchor`, off `paper3-wifi-vs-radar`.

**Spec:** `docs/superpowers/specs/2026-07-12-paper3-wifi-vs-radar-design.md`

---

## Why this sub-project is a GATE

Radar odometry is a mature field. If our back-end produces a laughable drift number on real radar, then in the ablation **radar would be an artificially weak baseline and WiFi would look artificially good** — and the whole paper would be built on sand. Better to find that out now than in review.

**This is not a formality, and a bad result is a real outcome.** Three verdicts, decided before we look:

| Our drift % (radar-only, Boreas) | Verdict | Consequence |
|---|---|---|
| **< 5 %** | **PASS** | The baseline is credible. Proceed to sub-project 3. |
| **5–10 %** | **MARGINAL** | Report it, and explicitly bound every claim: "our back-end is not CFEAR-class; radar's true capability is *underestimated* here, which makes our WiFi-vs-radar gap a **lower bound**." Proceed with that caveat stated in the abstract. |
| **> 10 %, or diverges** | **FAIL** | Stop. The radar baseline is a strawman and the ablation is not worth running until the back-end is fixed. Reconsider. |

**Do not tune the threshold after seeing the number.** It is written down here, first, on purpose.

---

## Global Constraints

1. **`lidar/slam_icp.py` MUST NOT be modified.** Reusing the back-end *unchanged* is the whole argument for attributing differences to the sensor. If it needs a change to work on radar, that is a finding to report, not a patch to slip in.
2. **The SOTA row is CITED, never reimplemented.** See the citation table below — the numbers are already verified; do not re-derive or "adjust" them.
3. **Standard KITTI sub-sequence lengths (100–800 m) are valid here** and must be used. Boreas sequences are kilometre-scale, unlike our 30–60 m simulated trajectories (where `drift()` correctly returns NaN).
4. **Never assume a data convention — verify it against the data.** Sub-project 1's lesson: the Sionna angle convention, the `paths.a` layout, and the transmitter count were all guessed wrong and only measurement caught it. The Boreas *yaw convention* is the same class of trap (Task 3, Step 1).
5. **Progress logging with ETA is mandatory** on any loop over thousands of frames, and ICP must use all cores. (`cKDTree(workers=-1)` is already in `slam_icp`.) The user's standing instruction: *"instead of waiting forever blindly, first introduce a proper detailed logging system, second utilize full server capabilities."*
6. **Server runs are throttled**: `nice -n 19 ionice -c3`, so other services are undisturbed. Repo path on the server: `/home/dev/mulham/wifi-radar-slam`.
7. **Pillow is available on the server; OpenCV and imageio are NOT.** Decode PNGs with `PIL.Image`.

---

## The data — every fact below was verified by downloading and decoding a real file

**Not from documentation. From the bytes.** (Verified 2026-07-12.)

**Boreas is public over anonymous HTTPS** — no registration, no credentials, no `aws` CLI (which the server does not have):

```
scan:    https://boreas.s3.amazonaws.com/<sequence>/radar/<timestamp>.png
poses:   https://boreas.s3.amazonaws.com/<sequence>/applanix/radar_poses.csv
listing: https://boreas.s3.amazonaws.com/?list-type=2&prefix=<sequence>/radar/&max-keys=1000
         (paginate via <NextContinuationToken>)
```

This is why we use **Boreas and not Oxford Radar RobotCar**: Oxford requires registration, which cannot be automated.

**Chosen sequence:** `boreas-2020-11-26-13-58` — **12,426 radar scans, 2.75 GB total**. We take the first **2,500** frames (~625 s at 4 Hz, several km — comfortably enough for 800 m sub-sequences), which is ~1.1 GB. The server has 683 GB free.

**The radar scan file** is a polar range–azimuth image:

```
PIL.Image.open(f) -> np.uint8 array, shape (400, 3371)
                     400 azimuths (rows) x 3371 columns

columns 0..10  : METADATA, not power
    bytes 0..7 : int64   timestamp of THAT AZIMUTH, microseconds
    bytes 8..9 : uint16  rotation-encoder count
    byte  10   : uint8   valid flag (255 = valid)
columns 11..   : 3360 range bins of uint8 power
```

**Azimuth decode (verified):** `azimuth_rad = encoder * pi / 2800`. The encoder steps by 14 per row → 0.9° per azimuth → 400 × 0.9° = exactly 360°. Encoder values observed: 12, 26, 40, …, 5588.

**Range decode (verified):** `range_m = (bin_index + 0.5) * 0.0596` → 3360 bins × 0.0596 m = **200.3 m** max range.

**Each azimuth has its OWN timestamp**, and a full rotation spans **248.7 ms** (4 Hz). This is load-bearing — see Task 3.

**Ground truth** `applanix/radar_poses.csv`, one row per scan, with a real CSV header:

```
GPSTime,easting,northing,altitude,vel_east,vel_north,vel_up,roll,pitch,heading,angvel_z,angvel_y,angvel_x
1606417097528152,-0.00559...,-0.01057...,0.49488...,...,3.1202...,-0.0113...,0.2008...,...
```

`GPSTime` **exactly matches the radar PNG filename**, so frames join to poses by filename — no interpolation needed. `easting`/`northing` are metres in a local frame starting near the origin. `heading` is in radians. **The heading convention is NOT assumed — Task 3 Step 1 verifies it against the position deltas.**

---

## The SOTA rows to cite — verified, with their caveats

| Method | Drift | Dataset | Source | **Caveat that must be stated** |
|---|---|---|---|---|
| **CFEAR** | **1.09 %** trans. | Oxford Radar RobotCar | Adolfsson et al., *Lidar-Level Localization With Radar? …*, **IEEE T-RO 39(2):1476–1495, 2023** | The 1.09 % is the **tuned** result. **Untuned it reports 1.16 %.** Radar-only, no gyro. |
| **DRO** | **0.26 %** trans. | **Boreas** leaderboard | Lisus et al., *DRO: Doppler-Aware Direct Radar Odometry*, arXiv:2504.20339 | **GYRO-AIDED**, and a *direct* method using all intensity (no point extraction). **It is NOT an apples-to-apples bound for a radar-only, point-based method like ours** and must never be presented as one. |

**CFEAR is our real reference point**: it is radar-only, point-based, and — per its own abstract — *"keeps the strongest returns per azimuth"*, which is exactly the front-end Task 1 builds. It is also on **Oxford**, not Boreas, and that difference must be stated rather than papered over.

---

## File Structure

| File | Responsibility |
|---|---|
| `src/wifi_radar_slam/radar/kstrongest.py` | The k-strongest-per-azimuth front-end. Generic: takes any `(n_azimuth, n_range)` power map + its range/azimuth grids → `Scan`. Serves BOTH the real Boreas data and our simulated radar. |
| `src/wifi_radar_slam/radar/boreas.py` | Pure loaders: decode a Boreas polar PNG (metadata + power), decode GT poses. No network. Mirrors `lidar/kitti.py`. |
| `experiments/fetch_boreas.py` | Network only. Downloads N scans + the pose CSV. Mirrors `experiments/fetch_kitti.py`. |
| `experiments/run_radar_anchor.py` | The anchor run: real radar → k-strongest → **unchanged** shared ICP back-end → KITTI drift, beside the cited rows. |
| `docs/results-paper3-anchor.md` | The verdict. |
| `tests/test_kstrongest.py`, `tests/test_boreas.py` | Unit tests; a synthetic PNG stands in for real data so tests need no network. |

---

### Task 0: Branch, and fix the spec's SOTA citations

The spec currently cites "CFEAR (1.09 %) and DRO (0.26 %)" side by side as if they were comparable upper bounds. They are not, and shipping that into a paper would be a citation error of exactly the kind we caught in paper 2 (P2SLAM's venue).

**Files:**
- Modify: `docs/superpowers/specs/2026-07-12-paper3-wifi-vs-radar-design.md`

- [ ] **Step 1: Cut the branch**

```bash
cd /mnt/data/projects/wifi-radar
git checkout paper3-wifi-vs-radar
git pull --ff-only
git checkout -b paper3-sub2-anchor
```

- [ ] **Step 2: Correct the citation, in the "Credibility" section**

Replace the sentence that reads *"Radar SLAM is mature — **CFEAR 1.09 %** drift (Oxford), **DRO 0.26 %** (Boreas leaderboard)."* with:

```markdown
Radar SLAM is mature. The two rows we cite, **with their caveats stated** (verified 2026-07-12):

- **CFEAR — 1.09 % translational drift** on Oxford Radar RobotCar (Adolfsson et al., *Lidar-Level
  Localization With Radar?*, IEEE T-RO 39(2):1476–1495, 2023). This is the **tuned** figure; the
  same paper reports **1.16 %** without parameter tuning. Radar-only, point-based, and its
  front-end *"keeps the strongest returns per azimuth"* — the same class of front-end we use.
  **This is our real reference point.**
- **DRO — 0.26 %** on the Boreas leaderboard (Lisus et al., arXiv:2504.20339). But this number is
  **gyro-aided**, and DRO is a *direct* method that registers raw intensity without extracting
  points at all. It is **not** an apples-to-apples bound for a radar-only, point-based method like
  ours, and must not be presented as one.

Note also that CFEAR's number is on **Oxford** while we anchor on **Boreas** (Oxford requires
registration, which cannot be automated). Same protocol, same sensor class, different city — say so.
```

- [ ] **Step 3: Commit**

```bash
git add docs/superpowers/specs/2026-07-12-paper3-wifi-vs-radar-design.md
git commit -m "paper3(spec): correct the SOTA radar-odometry citations

DRO's 0.26% is GYRO-AIDED and is a direct intensity method -- it is not an apples-to-apples
bound for a radar-only point-based back-end like ours, and the spec was presenting it as one.
CFEAR's 1.09% is the TUNED figure (1.16% untuned) and is on Oxford, not Boreas. Verified
against the primary sources."
```

---

### Task 1: The k-strongest-per-azimuth front-end

**Files:**
- Create: `src/wifi_radar_slam/radar/kstrongest.py`
- Test: `tests/test_kstrongest.py`

**Interfaces:**
- Consumes: `lidar.pointcloud.Scan`.
- Produces:
  - `k_strongest(power, ranges, azimuths, k=12, min_range_m=2.0, max_range_m=None, z_min=0.0) -> Scan`
    — `power` is `(n_azimuth, n_range)` real; `ranges` is `(n_range,)` metres; `azimuths` is
    `(n_azimuth,)` radians. Returns a `Scan` in the sensor-local frame (+x forward).
  - `k_strongest_from_cfg(ra_map, cfg, k=12) -> Scan` — convenience wrapper that pulls the grids
    off a `RadarConfig` (so the simulated radar uses the identical extractor).

**Why this exists, and why it is separate from CFAR:** sub-project 1 measured only ~1–5 CFAR
detections per frame on a diffuse scene — far too sparse for scan-to-map ICP. CFAR hunts *point
targets in noise*, but a real (or diffuse-simulated) street returns a **continuum**, so almost
nothing exceeds its own local background. CFEAR — the SOTA baseline — does not use CFAR at all; it
takes the **k strongest returns per azimuth**. Both front-ends ship (spec: *The two front-ends*):
CFAR defines the **phantom rate** (RQ1), k-strongest drives **SLAM** (RQ3).

- [ ] **Step 1: Write the failing test**

Create `tests/test_kstrongest.py`:

```python
import numpy as np
import pytest
from wifi_radar_slam.radar.kstrongest import k_strongest, k_strongest_from_cfg
from wifi_radar_slam.radar.config import RADAR_77G_4G
from wifi_radar_slam.lidar.pointcloud import Scan


def grids(n_az=8, n_rg=100, res=1.0):
    azimuths = np.linspace(-np.pi / 2, np.pi / 2, n_az)
    ranges = (np.arange(n_rg) + 0.5) * res
    return azimuths, ranges


def test_picks_the_single_strongest_return_per_azimuth():
    az, rg = grids()
    power = np.zeros((len(az), len(rg)))
    power[:, 30] = 5.0                      # every azimuth has one target at bin 30
    scan = k_strongest(power, rg, az, k=1)
    assert isinstance(scan, Scan)
    assert len(scan) == len(az)             # exactly one point per azimuth
    r = np.linalg.norm(scan.points, axis=1)
    assert np.allclose(r, rg[30], atol=1e-9)


def test_returns_at_most_k_per_azimuth():
    az, rg = grids()
    rng = np.random.default_rng(0)
    power = rng.random((len(az), len(rg)))  # dense noise: every bin is a candidate
    scan = k_strongest(power, rg, az, k=3)
    assert len(scan) <= 3 * len(az)


def test_the_strongest_win():
    az, rg = grids(n_az=1)
    power = np.zeros((1, len(rg)))
    power[0, 10] = 1.0
    power[0, 50] = 9.0                      # strongest
    power[0, 80] = 5.0
    scan = k_strongest(power, rg, az, k=2)
    r = np.sort(np.linalg.norm(scan.points, axis=1))
    assert np.allclose(r, [rg[50], rg[80]], atol=1e-9)   # the 1.0 peak is dropped


def test_azimuth_maps_to_the_right_bearing():
    az = np.array([0.0, np.pi / 2])
    rg = (np.arange(50) + 0.5) * 1.0
    power = np.zeros((2, 50))
    power[0, 9] = 1.0                       # 9.5 m dead ahead
    power[1, 9] = 1.0                       # 9.5 m to the left (+y)
    scan = k_strongest(power, rg, az, k=1)
    pts = scan.points[np.argsort(scan.points[:, 1])]
    assert np.allclose(pts[1], [0.0, 9.5], atol=1e-6)     # +90 deg -> +y
    assert np.allclose(pts[0], [9.5, 0.0], atol=1e-6)     # 0 deg   -> +x


def test_range_gating():
    az, rg = grids(n_rg=200)
    power = np.zeros((len(az), 200))
    power[:, 0] = 9.0                       # 0.5 m -- inside the blind zone
    power[:, 150] = 9.0                     # 150.5 m
    scan = k_strongest(power, rg, az, k=2, min_range_m=2.0, max_range_m=100.0)
    assert len(scan) == 0                   # both gated out


def test_z_min_threshold_rejects_the_noise_floor():
    az, rg = grids()
    power = np.full((len(az), len(rg)), 0.1)    # a flat noise floor
    power[:, 40] = 7.0                          # one real target
    scan = k_strongest(power, rg, az, k=5, z_min=1.0)
    assert len(scan) == len(az)                 # only the real target survives, once per azimuth


def test_empty_power_map_gives_an_empty_scan():
    az, rg = grids()
    scan = k_strongest(np.zeros((len(az), len(rg))), rg, az, k=4, z_min=0.5)
    assert isinstance(scan, Scan) and len(scan) == 0


def test_from_cfg_uses_the_configs_own_grids():
    # The simulated radar must go through the IDENTICAL extractor as the real radar,
    # or the front-end becomes confounded with the sensor.
    cfg = RADAR_77G_4G
    ra = np.zeros((cfg.n_azimuth, cfg.n_range))
    i = int(np.argmin(np.abs(cfg.range_bins() - 30.0)))
    ra[cfg.n_azimuth // 2, i] = 10.0
    scan = k_strongest_from_cfg(ra, cfg, k=1)
    assert len(scan) == 1
    assert np.linalg.norm(scan.points[0]) == pytest.approx(30.0, abs=cfg.range_resolution_m)


def test_mismatched_grid_shapes_are_rejected():
    az, rg = grids()
    with pytest.raises(ValueError):
        k_strongest(np.zeros((3, 7)), rg, az, k=1)
```

- [ ] **Step 2: Run the test and watch it fail**

```bash
.venv/bin/python -m pytest tests/test_kstrongest.py -q
```

Expected: `ModuleNotFoundError: No module named 'wifi_radar_slam.radar.kstrongest'`.

- [ ] **Step 3: Implement**

Create `src/wifi_radar_slam/radar/kstrongest.py`:

```python
"""The k-strongest-per-azimuth front-end -- the CFEAR-style extractor.

PURE NumPy. Generic over the power map, so the SAME extractor serves the real Boreas radar
and our simulated radar. That is not a convenience, it is a requirement: a front-end that
differed between sensors would be confounded with the sensor difference we are trying to
measure.

WHY THIS EXISTS ALONGSIDE CFAR. Sub-project 1 measured only ~1-5 CA-CFAR detections per frame
on a diffusely-scattering scene -- far too sparse for scan-to-map ICP. That is not a bug: CFAR
is built to find *point targets in noise*, but a real street (or a diffuse simulation of one)
returns a near-*continuum*, so the local background a wall is compared against is the wall
itself, and almost nothing clears the threshold.

Radar odometry has always known this. CFEAR -- the SOTA baseline we anchor against -- does not
use CFAR at all; per its own abstract it "keeps the strongest returns per azimuth", precisely
because radar targets are *extended*, not point-like. So the paper carries both front-ends:
CFAR defines the phantom rate (RQ1, where a calibrated detection threshold is what makes
"this detection matches no real path" a meaningful claim), and k-strongest drives SLAM (RQ3).
Both are applied identically to every ablation cell.
"""
from __future__ import annotations
import numpy as np
from ..lidar.pointcloud import Scan


def k_strongest(power: np.ndarray, ranges: np.ndarray, azimuths: np.ndarray,
                k: int = 12, min_range_m: float = 2.0,
                max_range_m: float | None = None, z_min: float = 0.0) -> Scan:
    """Keep the k strongest range bins in each azimuth -> a sensor-local Scan.

    Args:
        power:       (n_azimuth, n_range) real power / intensity.
        ranges:      (n_range,) range of each bin, metres.
        azimuths:    (n_azimuth,) bearing of each row, radians (+x forward, +y at +90 deg).
        k:           returns kept per azimuth.
        min_range_m: blind zone -- a monostatic radar hears itself at short range.
        max_range_m: gate beyond this (None -> no upper gate).
        z_min:       absolute power floor; bins at or below it are never returned.

    Returns a Scan in the sensor-local frame. Points are ordinary polar -> Cartesian: the
    geometry is monostatic, so the measured range is an honest round trip.
    """
    power = np.asarray(power, dtype=float)
    ranges = np.asarray(ranges, dtype=float).ravel()
    azimuths = np.asarray(azimuths, dtype=float).ravel()
    if power.shape != (azimuths.size, ranges.size):
        raise ValueError(
            f"power {power.shape} does not match grids "
            f"(n_azimuth={azimuths.size}, n_range={ranges.size})")

    gate = ranges >= min_range_m
    if max_range_m is not None:
        gate &= ranges <= max_range_m
    if not gate.any():
        return Scan.empty()

    p = power[:, gate]
    r_gated = ranges[gate]

    kk = int(min(k, p.shape[1]))
    # argpartition puts the kk largest of each row in the last kk slots -- O(n) per row,
    # and we do not care about their order among themselves.
    idx = np.argpartition(p, -kk, axis=1)[:, -kk:]           # (n_azimuth, kk)
    rows = np.repeat(np.arange(p.shape[0]), kk)
    cols = idx.ravel()
    vals = p[rows, cols]

    keep = vals > z_min
    if not keep.any():
        return Scan.empty()
    rows, cols = rows[keep], cols[keep]

    r = r_gated[cols]
    a = azimuths[rows]
    return Scan(np.stack([r * np.cos(a), r * np.sin(a)], axis=1))


def k_strongest_from_cfg(ra_map: np.ndarray, cfg, k: int = 12) -> Scan:
    """k_strongest on OUR simulated radar, using the RadarConfig's own grids.

    The simulated cells go through this identical extractor so that the front-end is held
    fixed across every ablation cell (A-D) and across real-vs-simulated data.
    """
    return k_strongest(ra_map, cfg.range_bins(), cfg.azimuth_grid(), k=k,
                       min_range_m=cfg.min_range_m, max_range_m=cfg.max_range_m)
```

- [ ] **Step 4: Run the tests and watch them pass**

```bash
.venv/bin/python -m pytest tests/test_kstrongest.py -q
```

Expected: 9 passed.

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/radar/kstrongest.py tests/test_kstrongest.py
git commit -m "paper3(radar): k-strongest-per-azimuth front-end (the CFEAR-style extractor)

CFAR yields only ~1-5 detections/frame on a diffuse scene -- it hunts point targets in noise,
but a real street returns a continuum. CFEAR, the SOTA anchor, keeps the strongest returns per
azimuth for exactly this reason. Generic over the power map, so the SAME extractor serves the
real Boreas radar and our simulated radar -- a front-end that differed between them would be
confounded with the sensor difference we are trying to measure."
```

---

### Task 2: The Boreas polar-radar loader

**Files:**
- Create: `src/wifi_radar_slam/radar/boreas.py`
- Test: `tests/test_boreas.py`

**Interfaces:**
- Consumes: `k_strongest` (Task 1), `lidar.pointcloud.Scan`.
- Produces:
  - `BOREAS_RANGE_RES_M = 0.0596`, `BOREAS_N_METADATA_COLS = 11`, `BOREAS_ENCODER_PER_REV = 5600`
  - `decode_polar(img) -> (power, azimuths, timestamps_us, valid)` — `img` is the raw
    `(400, 3371)` uint8 array; returns `power (400, 3360) float`, `azimuths (400,) rad`,
    `timestamps_us (400,) int64`, `valid (400,) bool`.
  - `range_bins(n_range) -> np.ndarray` — `(bin + 0.5) * 0.0596`.
  - `load_radar_scan(path, k=12, min_range_m=2.0, max_range_m=100.0) -> Scan`
  - `load_gt_poses(csv_text) -> (timestamps_us (n,), poses (n,3))` — poses are `(x, y, yaw)`.

- [ ] **Step 1: Write the failing test**

Create `tests/test_boreas.py`. The synthetic PNG is built with the **exact byte layout verified
against a real file**, so these tests need no network:

```python
import numpy as np
import pytest
from wifi_radar_slam.radar import boreas
from wifi_radar_slam.lidar.pointcloud import Scan


N_AZ, N_RG = 400, 3360
META = boreas.BOREAS_N_METADATA_COLS      # 11


def synthetic_image(target_bin=100, base_ts=1_606_417_097_528_152):
    """A Boreas-format polar image with one target at `target_bin` in every azimuth.

    Byte layout verified against a real file:
      cols 0..7  int64  per-azimuth timestamp (us)
      cols 8..9  uint16 rotation encoder
      col  10    uint8  valid flag (255)
      cols 11..  uint8  power, 3360 range bins
    """
    img = np.zeros((N_AZ, META + N_RG), dtype=np.uint8)
    # encoder steps by 14 per azimuth (0.9 deg); 400 * 0.9 = 360 deg exactly
    enc = (12 + 14 * np.arange(N_AZ)).astype(np.uint16)
    img[:, 8:10] = enc.view(np.uint8).reshape(N_AZ, 2)
    ts = (base_ts + (np.arange(N_AZ) * 625)).astype(np.int64)     # ~250 ms / 400
    img[:, 0:8] = ts.view(np.uint8).reshape(N_AZ, 8)
    img[:, 10] = 255
    img[:, META + target_bin] = 200
    return img, enc, ts


def test_decode_shapes():
    img, _, _ = synthetic_image()
    power, az, ts, valid = boreas.decode_polar(img)
    assert power.shape == (N_AZ, N_RG)
    assert az.shape == (N_AZ,) and ts.shape == (N_AZ,) and valid.shape == (N_AZ,)
    assert valid.all()


def test_azimuth_decode_spans_exactly_one_revolution():
    # THE decode that must be right: azimuth = encoder * pi / 2800. Verified against a real
    # file -- the encoder steps by 14, which is 0.9 deg, and 400 x 0.9 = 360 deg exactly.
    img, enc, _ = synthetic_image()
    _, az, _, _ = boreas.decode_polar(img)
    assert np.allclose(az, enc.astype(float) * np.pi / 2800.0)
    step_deg = np.rad2deg(az[1] - az[0])
    assert step_deg == pytest.approx(0.9, abs=1e-6)
    assert np.rad2deg(az[-1] - az[0]) == pytest.approx(359.1, abs=1e-3)


def test_timestamp_decode_is_per_azimuth_and_monotone():
    # Each azimuth carries its OWN timestamp; a full rotation spans ~250 ms. This is what
    # makes motion compensation possible (Task 3) -- and necessary.
    img, _, ts = synthetic_image()
    _, _, got, _ = boreas.decode_polar(img)
    assert np.array_equal(got, ts)
    assert np.all(np.diff(got) > 0)
    span_ms = (got[-1] - got[0]) / 1000.0
    assert 200.0 < span_ms < 300.0


def test_power_excludes_the_metadata_columns():
    # Off-by-11 here would silently shift EVERY range by 0.66 m.
    img, _, _ = synthetic_image(target_bin=100)
    power, _, _, _ = boreas.decode_polar(img)
    assert power[:, 100].max() == 200
    assert power[:, :100].max() == 0


def test_range_bins():
    r = boreas.range_bins(N_RG)
    assert r.shape == (N_RG,)
    assert r[0] == pytest.approx(0.5 * boreas.BOREAS_RANGE_RES_M)
    assert np.diff(r)[0] == pytest.approx(boreas.BOREAS_RANGE_RES_M)
    assert r[-1] == pytest.approx(200.3, abs=0.1)      # 3360 bins x 0.0596 m


def test_load_radar_scan_from_a_written_png(tmp_path):
    from PIL import Image
    img, _, _ = synthetic_image(target_bin=1000)       # 1000 * 0.0596 = 59.6 m
    p = tmp_path / "1606417097528152.png"
    Image.fromarray(img).save(p)
    scan = boreas.load_radar_scan(str(p), k=1, min_range_m=2.0, max_range_m=100.0)
    assert isinstance(scan, Scan)
    assert len(scan) == N_AZ                            # one return per azimuth
    r = np.linalg.norm(scan.points, axis=1)
    assert np.allclose(r, 1000.5 * boreas.BOREAS_RANGE_RES_M, atol=1e-6)


def test_load_gt_poses():
    csv = (
        "GPSTime,easting,northing,altitude,vel_east,vel_north,vel_up,"
        "roll,pitch,heading,angvel_z,angvel_y,angvel_x\n"
        "1606417097528152,0.0,0.0,0.5,0,0,0,3.12,-0.01,0.2,0,0,0\n"
        "1606417097778155,1.0,2.0,0.5,0,0,0,3.12,-0.01,0.3,0,0,0\n"
    )
    ts, poses = boreas.load_gt_poses(csv)
    assert ts.tolist() == [1606417097528152, 1606417097778155]
    assert poses.shape == (2, 3)
    assert np.allclose(poses[1, :2], [1.0, 2.0])
    assert poses[1, 2] == pytest.approx(0.3)            # heading -> yaw, radians


def test_gt_timestamps_match_the_scan_filenames():
    # The join key. GPSTime is EXACTLY the PNG filename -- no interpolation needed.
    csv = ("GPSTime,easting,northing,altitude,vel_east,vel_north,vel_up,"
           "roll,pitch,heading,angvel_z,angvel_y,angvel_x\n"
           "1606417097528152,0.0,0.0,0.5,0,0,0,0,0,0.0,0,0,0\n")
    ts, _ = boreas.load_gt_poses(csv)
    assert str(ts[0]) == "1606417097528152"
```

- [ ] **Step 2: Run the test and watch it fail**

```bash
.venv/bin/python -m pytest tests/test_boreas.py -q
```

Expected: `ImportError: cannot import name 'boreas'`.

- [ ] **Step 3: Implement**

Create `src/wifi_radar_slam/radar/boreas.py`:

```python
"""Loaders for the Boreas spinning-radar benchmark (Navtech, 4 Hz, 360 deg).

Pure NumPy + Pillow. NO NETWORK here -- fetching is experiments/fetch_boreas.py, mirroring the
lidar/kitti.py + experiments/fetch_kitti.py split.

Boreas, not Oxford Radar RobotCar, for one decisive reason: Boreas is served over ANONYMOUS
public HTTPS, while Oxford requires a registration that cannot be automated.

EVERY CONSTANT BELOW WAS VERIFIED BY DECODING A REAL FILE (2026-07-12), not read off a wiki:

    PIL.Image.open(scan) -> uint8 (400, 3371)
        cols 0..7   int64  timestamp of THAT azimuth, microseconds
        cols 8..9   uint16 rotation-encoder count
        col  10     uint8  valid flag (255 = valid)
        cols 11..   uint8  power, 3360 range bins

    azimuth_rad = encoder * pi / 2800     (encoder steps by 14 -> 0.9 deg; 400 x 0.9 = 360)
    range_m     = (bin + 0.5) * 0.0596    (3360 bins -> 200.3 m)

A full rotation spans ~249 ms, and every azimuth carries its own timestamp -- which is what
makes motion compensation both possible and necessary (see `motion_compensate`).
"""
from __future__ import annotations
import numpy as np
from ..lidar.pointcloud import Scan
from .kstrongest import k_strongest

BOREAS_RANGE_RES_M = 0.0596        # metres per range bin
BOREAS_N_METADATA_COLS = 11        # 8 (timestamp) + 2 (encoder) + 1 (valid)
BOREAS_ENCODER_PER_REV = 5600      # encoder counts in a full revolution -> az = enc * pi / 2800


def range_bins(n_range: int) -> np.ndarray:
    """Range (m) at the centre of each bin."""
    return (np.arange(n_range) + 0.5) * BOREAS_RANGE_RES_M


def decode_polar(img: np.ndarray):
    """Split a raw Boreas polar image into (power, azimuths, timestamps_us, valid).

    `img` is the uint8 (n_azimuth, 11 + n_range) array straight out of the PNG.

    The 11 metadata columns are NOT power. Feeding them to the front-end would shift every
    range by 11 bins (0.66 m) and put a bright fictional target at zero range in every azimuth.
    """
    img = np.asarray(img)
    if img.dtype != np.uint8 or img.ndim != 2:
        raise ValueError(f"expected a 2-D uint8 image, got {img.dtype} {img.shape}")
    if img.shape[1] <= BOREAS_N_METADATA_COLS:
        raise ValueError(f"image has no range bins: {img.shape}")

    ts = np.ascontiguousarray(img[:, 0:8]).view(np.int64).ravel()
    enc = np.ascontiguousarray(img[:, 8:10]).view(np.uint16).ravel()
    valid = img[:, 10] == 255
    power = img[:, BOREAS_N_METADATA_COLS:].astype(float)
    azimuths = enc.astype(float) * np.pi / (BOREAS_ENCODER_PER_REV / 2.0)
    return power, azimuths, ts, valid


def load_radar_scan(path: str, k: int = 12, min_range_m: float = 2.0,
                    max_range_m: float = 100.0) -> Scan:
    """Decode one Boreas radar PNG and extract a Scan with the k-strongest front-end.

    max_range_m defaults to 100 m rather than the sensor's full 200 m: the far half of a
    Navtech scan is sparse and noisy, and CFEAR-class methods likewise work on a cropped
    range. Stated, not silent.
    """
    from PIL import Image                        # lazy: keeps import cost off the test path
    img = np.array(Image.open(path))
    power, azimuths, _, valid = decode_polar(img)
    if not valid.all():                          # drop azimuths the sensor flagged bad
        power, azimuths = power[valid], azimuths[valid]
    if power.shape[0] == 0:
        return Scan.empty()
    return k_strongest(power, range_bins(power.shape[1]), azimuths, k=k,
                       min_range_m=min_range_m, max_range_m=max_range_m)


def load_gt_poses(csv_text: str):
    """Parse applanix/radar_poses.csv -> (timestamps_us (n,), poses (n,3) as x, y, yaw).

    Columns (real header, verified):
        GPSTime,easting,northing,altitude,vel_east,vel_north,vel_up,
        roll,pitch,heading,angvel_z,angvel_y,angvel_x

    GPSTime is EXACTLY the radar PNG's filename, so scans join to poses by name -- no
    interpolation, no nearest-neighbour matching, no chance of an off-by-one frame shift.

    We take (easting, northing) as (x, y) and `heading` as yaw. The yaw CONVENTION is not
    assumed here -- experiments/run_radar_anchor.py verifies it against the position deltas
    before trusting it (see that script's `check_yaw_convention`).
    """
    lines = [ln for ln in csv_text.strip().splitlines() if ln.strip()]
    header = [c.strip() for c in lines[0].split(",")]
    for want in ("GPSTime", "easting", "northing", "heading"):
        if want not in header:
            raise ValueError(f"radar_poses.csv missing column {want!r}; got {header}")
    i_t, i_x = header.index("GPSTime"), header.index("easting")
    i_y, i_h = header.index("northing"), header.index("heading")

    ts, poses = [], []
    for ln in lines[1:]:
        f = ln.split(",")
        ts.append(int(f[i_t]))
        poses.append([float(f[i_x]), float(f[i_y]), float(f[i_h])])
    return np.array(ts, dtype=np.int64), np.array(poses, dtype=float)
```

- [ ] **Step 4: Run the tests and watch them pass**

```bash
.venv/bin/python -m pytest tests/test_boreas.py -q
```

Expected: 8 passed.

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/radar/boreas.py tests/test_boreas.py
git commit -m "paper3(radar): Boreas polar-radar loader

Every constant verified by decoding a real file, not read off a wiki: the 11 metadata columns
(int64 per-azimuth timestamp, uint16 encoder, valid flag), azimuth = encoder*pi/2800, and
range = (bin+0.5)*0.0596. Feeding the metadata columns to the front-end as if they were power
would shift every range by 0.66 m and plant a bright fictional target at zero range in every
azimuth.

Boreas rather than Oxford because Boreas is served over anonymous public HTTPS; Oxford requires
a registration that cannot be automated."
```

---

### Task 3: Motion compensation for the spinning scan

**Files:**
- Modify: `src/wifi_radar_slam/radar/boreas.py` (append `motion_compensate`, and wire an
  optional `velocity` argument through `load_radar_scan`)
- Test: `tests/test_boreas.py` (append)

**Interfaces:**
- Consumes: `decode_polar` (Task 2).
- Produces: `motion_compensate(points, azimuth_times_us, scan_time_us, velocity_xy) -> np.ndarray`
  — shifts each point by the ego-motion that occurred between its own azimuth's timestamp and the
  scan's reference timestamp. And `load_radar_scan(..., velocity_xy=None, ...)`: when `velocity_xy`
  is given, the returned `Scan` is undistorted.

**Why this is not optional.** A Navtech scan takes **249 ms** to sweep 360°. At 15 m/s the vehicle
travels **3.7 m** during a single rotation — far more than the 0.06 m range resolution. Treating the
scan as instantaneous therefore smears every scan by metres, in a way that rotates with the beam.
CFEAR compensates for exactly this, and skipping it would hand us a badly-drifting back-end and a
FAILED gate **for a reason that has nothing to do with our back-end**. That would be the worst
outcome available: a true-looking negative.

Our *simulated* radar has no such distortion (its scan is instantaneous), so this step is specific
to real spinning-radar data — and that is a property of the **data**, not of the estimator, so the
back-end stays untouched.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_boreas.py`:

```python
def test_motion_compensation_shifts_points_by_the_ego_motion_during_the_sweep():
    # A Navtech scan takes ~249 ms. At 10 m/s the car moves 2.5 m mid-sweep -- 40x the 0.06 m
    # range resolution. A point measured at the START of the sweep was taken from a position
    # 2.5 m behind where the car is at the sweep's REFERENCE time, so in the reference frame
    # that point must move BACKWARD along the direction of travel.
    pts = np.array([[10.0, 0.0], [10.0, 0.0]])
    t_az = np.array([0, 250_000], dtype=np.int64)      # first azimuth, last azimuth (us)
    t_ref = 250_000                                    # reference = end of sweep
    out = boreas.motion_compensate(pts, t_az, t_ref, velocity_xy=(10.0, 0.0))
    # the last azimuth is AT the reference time -> unmoved
    assert np.allclose(out[1], [10.0, 0.0])
    # the first azimuth was 0.25 s earlier -> shifted back by 10 m/s * 0.25 s = 2.5 m
    assert np.allclose(out[0], [7.5, 0.0])


def test_motion_compensation_is_a_no_op_at_zero_velocity():
    pts = np.array([[10.0, 0.0], [0.0, 5.0]])
    t_az = np.array([0, 250_000], dtype=np.int64)
    out = boreas.motion_compensate(pts, t_az, 250_000, velocity_xy=(0.0, 0.0))
    assert np.allclose(out, pts)


def test_motion_compensation_rejects_a_length_mismatch():
    with pytest.raises(ValueError):
        boreas.motion_compensate(np.zeros((3, 2)), np.zeros(2, dtype=np.int64), 0, (1.0, 0.0))


def test_load_radar_scan_with_velocity_undistorts(tmp_path):
    from PIL import Image
    img, _, ts = synthetic_image(target_bin=1000)
    p = tmp_path / "scan.png"
    Image.fromarray(img).save(p)
    still = boreas.load_radar_scan(str(p), k=1, max_range_m=100.0)
    moving = boreas.load_radar_scan(str(p), k=1, max_range_m=100.0, velocity_xy=(10.0, 0.0))
    assert len(still) == len(moving)
    # undistortion must actually move the points
    assert not np.allclose(still.points, moving.points)
    # and the shift must be bounded by v * sweep_duration = 10 * 0.25 s = 2.5 m
    assert np.abs(still.points - moving.points).max() <= 2.6
```

- [ ] **Step 2: Run the tests and watch them fail**

```bash
.venv/bin/python -m pytest tests/test_boreas.py -q
```

Expected: `AttributeError: module 'wifi_radar_slam.radar.boreas' has no attribute 'motion_compensate'`.

- [ ] **Step 3: Implement**

Append to `src/wifi_radar_slam/radar/boreas.py`:

```python
def motion_compensate(points: np.ndarray, azimuth_times_us: np.ndarray,
                      scan_time_us: int, velocity_xy) -> np.ndarray:
    """Undistort a spinning-radar scan for ego-motion during the sweep.

    Args:
        points:           (n, 2) sensor-local points, one per return.
        azimuth_times_us: (n,) the timestamp of the azimuth EACH point came from.
        scan_time_us:     the scan's reference timestamp (we use the sweep's last azimuth).
        velocity_xy:      (vx, vy) ego velocity in the SENSOR-LOCAL frame, m/s.

    Returns the (n, 2) points expressed as though every one had been measured at
    `scan_time_us`.

    THIS IS NOT OPTIONAL, AND HERE IS WHY. A Navtech scan sweeps 360 degrees in ~249 ms. At
    15 m/s the vehicle covers 3.7 m in that time -- sixty times the 0.06 m range resolution.
    A scan treated as instantaneous is therefore smeared by metres, and the smear ROTATES with
    the beam, so it does not cancel. CFEAR compensates for exactly this.

    Skipping it would give us a badly-drifting back-end and a FAILED credibility gate for a
    reason that has nothing to do with our back-end -- a true-looking negative, which is the
    worst outcome available to an experiment.

    Our SIMULATED radar has no such distortion (its scan is instantaneous), so this correction
    applies only to real spinning-radar data. That is a property of the DATA, not of the
    estimator -- the shared back-end stays untouched, as it must.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    t = np.asarray(azimuth_times_us, dtype=np.int64).ravel()
    if t.size != points.shape[0]:
        raise ValueError(f"{points.shape[0]} points but {t.size} azimuth timestamps")
    if points.size == 0:
        return points
    vx, vy = float(velocity_xy[0]), float(velocity_xy[1])
    dt = (t.astype(float) - float(scan_time_us)) * 1e-6        # seconds, negative before ref
    # A point measured dt seconds BEFORE the reference was taken from a position the ego has
    # since left; in the reference frame it sits v*dt further along the travel direction.
    return points + np.stack([vx * dt, vy * dt], axis=1)
```

Then replace `load_radar_scan` with the velocity-aware version (the returns must keep track of
*which azimuth* each point came from, which `k_strongest` alone does not tell us — so build the
scan azimuth-by-azimuth when compensating):

```python
def load_radar_scan(path: str, k: int = 12, min_range_m: float = 2.0,
                    max_range_m: float = 100.0, velocity_xy=None) -> Scan:
    """Decode one Boreas radar PNG and extract a Scan with the k-strongest front-end.

    If `velocity_xy` (m/s, sensor-local) is given, the scan is motion-compensated for the
    ~249 ms sweep -- see `motion_compensate`, which explains why that is mandatory on real
    spinning radar.

    max_range_m defaults to 100 m rather than the sensor's full 200 m: the far half of a
    Navtech scan is sparse and noisy, and CFEAR-class methods likewise work on a cropped range.
    Stated, not silent.
    """
    from PIL import Image                        # lazy: keeps import cost off the test path
    img = np.array(Image.open(path))
    power, azimuths, times, valid = decode_polar(img)
    if not valid.all():                          # drop azimuths the sensor flagged bad
        power, azimuths, times = power[valid], azimuths[valid], times[valid]
    if power.shape[0] == 0:
        return Scan.empty()

    scan = k_strongest(power, range_bins(power.shape[1]), azimuths, k=k,
                       min_range_m=min_range_m, max_range_m=max_range_m)
    if velocity_xy is None or len(scan) == 0:
        return scan

    # k_strongest emits points grouped by azimuth row, k per row (after gating), so recover
    # each point's azimuth by matching its bearing back to the azimuth grid.
    bearings = np.arctan2(scan.points[:, 1], scan.points[:, 0])
    row = np.abs(np.angle(np.exp(1j * (bearings[:, None] - azimuths[None, :])))).argmin(axis=1)
    return Scan(motion_compensate(scan.points, times[row], int(times[-1]), velocity_xy))
```

- [ ] **Step 4: Run the tests and watch them pass**

```bash
.venv/bin/python -m pytest tests/test_boreas.py -q
```

Expected: 12 passed.

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/radar/boreas.py tests/test_boreas.py
git commit -m "paper3(radar): motion-compensate the spinning radar sweep

A Navtech scan sweeps 360 deg in ~249 ms. At 15 m/s the car moves 3.7 m during one rotation --
sixty times the range resolution -- and the smear rotates with the beam, so it does not cancel.
CFEAR compensates for exactly this. Skipping it would have produced a badly-drifting back-end
and a FAILED credibility gate for a reason having nothing to do with our back-end: a
true-looking negative, the worst outcome an experiment can produce.

The correction is a property of the DATA (a spinning sensor), not of the estimator -- the
shared back-end stays untouched, as the whole argument requires."
```

---

### Task 4: Fetch the sequence

**Files:**
- Create: `experiments/fetch_boreas.py`

Network only, no parsing — mirroring the `fetch_kitti.py` split.

- [ ] **Step 1: Write the fetch script**

Create `experiments/fetch_boreas.py`:

```python
"""Fetch N radar scans + the GT poses from the Boreas benchmark. Server-only (needs network).

Boreas is served over ANONYMOUS public HTTPS -- no registration, no credentials, and no `aws`
CLI (which the server does not have). This is precisely why we anchor on Boreas rather than
Oxford Radar RobotCar, whose download requires a registration that cannot be automated.

    nice -n 19 ionice -c3 .venv/bin/python experiments/fetch_boreas.py

Sequence boreas-2020-11-26-13-58 holds 12,426 scans (2.75 GB) at 4 Hz. We take the first
N_SCANS, which at ~4 Hz is several km -- comfortably enough for KITTI's 800 m sub-sequences.
"""
from __future__ import annotations
import concurrent.futures as cf
import logging
import os
import re
import time
import urllib.parse
import urllib.request

BASE = "https://boreas.s3.amazonaws.com"
SEQ = "boreas-2020-11-26-13-58"
OUT = f"data/boreas/{SEQ}"
N_SCANS = 2500                     # ~625 s at 4 Hz; ~1.1 GB
WORKERS = 16

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s",
                    datefmt="%H:%M:%S")
log = logging.getLogger("boreas")


def list_radar_keys(limit: int) -> list[str]:
    """List the first `limit` radar PNG keys, paginating the S3 REST listing."""
    keys: list[str] = []
    token = None
    while len(keys) < limit:
        url = (f"{BASE}/?list-type=2&prefix={SEQ}/radar/&max-keys=1000")
        if token:
            url += "&continuation-token=" + urllib.parse.quote(token, safe="")
        with urllib.request.urlopen(url, timeout=60) as r:
            body = r.read().decode()
        keys += re.findall(r"<Key>([^<]+\.png)</Key>", body)
        m = re.search(r"<NextContinuationToken>([^<]+)</NextContinuationToken>", body)
        if not m:
            break
        token = m.group(1)
    return sorted(keys)[:limit]


def fetch(key: str) -> int:
    dest = os.path.join("data", "boreas", key)
    if os.path.exists(dest) and os.path.getsize(dest) > 0:
        return 0
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    with urllib.request.urlopen(f"{BASE}/{key}", timeout=120) as r:
        data = r.read()
    with open(dest, "wb") as f:
        f.write(data)
    return len(data)


def main() -> None:
    os.makedirs(f"{OUT}/applanix", exist_ok=True)

    log.info("fetching GT poses ...")
    with urllib.request.urlopen(f"{BASE}/{SEQ}/applanix/radar_poses.csv", timeout=120) as r:
        open(f"{OUT}/applanix/radar_poses.csv", "wb").write(r.read())
    log.info("  -> %s/applanix/radar_poses.csv", OUT)

    log.info("listing radar keys ...")
    keys = list_radar_keys(N_SCANS)
    log.info("  -> %d scans", len(keys))

    t0 = time.time()
    total = 0
    with cf.ThreadPoolExecutor(max_workers=WORKERS) as pool:
        for i, n in enumerate(pool.map(fetch, keys), 1):
            total += n
            if i % 100 == 0 or i == len(keys):
                el = time.time() - t0
                eta = el / i * (len(keys) - i)
                log.info("  %4d/%d  %.2f GB  elapsed %.0fs  ETA %.0fs",
                         i, len(keys), total / 1e9, el, eta)
    log.info("done: %d scans, %.2f GB in %.0fs", len(keys), total / 1e9, time.time() - t0)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run it on the server**

```bash
git add experiments/fetch_boreas.py
git commit -m "paper3(radar): fetch script for the Boreas radar benchmark"
git push -u origin paper3-sub2-anchor
```

On `amd` (`/home/dev/mulham/wifi-radar-slam`):

```bash
git fetch origin '+refs/heads/paper3-sub2-anchor:refs/remotes/origin/paper3-sub2-anchor'
git checkout -B paper3-sub2-anchor origin/paper3-sub2-anchor
nice -n 19 ionice -c3 .venv/bin/python experiments/fetch_boreas.py
```

Expected: `done: 2500 scans, ~1.1 GB`. If the listing returns zero keys, the bucket layout has
changed — stop and re-derive it rather than guessing.

- [ ] **Step 3: Sanity-check one real file against the loader**

```bash
.venv/bin/python -c "
from wifi_radar_slam.radar import boreas
import glob, numpy as np
from PIL import Image
f = sorted(glob.glob('data/boreas/*/radar/*.png'))[0]
img = np.array(Image.open(f))
print('image', img.shape, img.dtype)
power, az, ts, valid = boreas.decode_polar(img)
print('power', power.shape, 'az span deg', np.rad2deg(az[-1]-az[0]).round(2))
print('sweep ms', (ts[-1]-ts[0])/1000.0, 'valid', valid.all())
scan = boreas.load_radar_scan(f, k=12)
print('scan points:', len(scan))
"
```

Expected: `image (400, 3371) uint8`, `az span deg 359.1`, `sweep ms ~249`, `valid True`, and a
scan with **hundreds to a few thousand points** (400 azimuths × up to 12) — not 4. That density
is the whole reason the k-strongest front-end exists.

---

### Task 5: The anchor run

**Files:**
- Create: `experiments/run_radar_anchor.py`

**Interfaces:**
- Consumes: `boreas.load_radar_scan`, `boreas.load_gt_poses`, `lidar.slam_icp.run_lidar_slam`
  (**unchanged**), `eval.drift.drift`, `eval.metrics.rpe`.

- [ ] **Step 1: Write the anchor script**

Create `experiments/run_radar_anchor.py`:

```python
"""THE CREDIBILITY GATE. Run our SHARED scan-to-map ICP back-end on REAL radar (Boreas) and
report KITTI-protocol drift beside the cited SOTA.

Why this exists: radar odometry is a mature field. If our back-end produces a laughable drift
number on real radar, then in paper 3's ablation radar would be an artificially weak baseline
and WiFi would look artificially good -- and the whole paper would be built on sand. The
verdict thresholds are fixed IN ADVANCE (see VERDICT below) precisely so they cannot be tuned
after seeing the number.

The back-end (lidar/slam_icp.run_lidar_slam) is used COMPLETELY UNCHANGED. That is the entire
argument: any difference between sensors must be attributable to the sensor, not the estimator.

    nice -n 19 ionice -c3 .venv/bin/python experiments/run_radar_anchor.py
"""
from __future__ import annotations
import concurrent.futures as cf
import glob
import json
import logging
import os
import time

import numpy as np

from wifi_radar_slam.radar import boreas
from wifi_radar_slam.lidar.slam_icp import run_lidar_slam, _rigid_2d, _apply
from wifi_radar_slam.eval.drift import drift, path_lengths
from wifi_radar_slam.eval.metrics import rpe

SEQ = "boreas-2020-11-26-13-58"
ROOT = f"data/boreas/{SEQ}"
K = 12                    # k-strongest returns per azimuth (CFEAR-class front-end)
MAX_RANGE_M = 100.0
MAP_VOXEL = 1.0           # accumulated-map voxel; radar scans are large, so coarser than LiDAR
DT = 0.25                 # Navtech is 4 Hz

# Cited SOTA -- NOT reimplemented. Caveats are part of the citation, not footnotes.
SOTA = {
    "CFEAR (T-RO 2023, Oxford, radar-only, tuned)": 1.09,
    "CFEAR (T-RO 2023, Oxford, radar-only, untuned)": 1.16,
    "DRO (arXiv 2504.20339, Boreas) -- GYRO-AIDED, direct-intensity, NOT comparable": 0.26,
}

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("anchor")


def check_yaw_convention(poses: np.ndarray) -> float:
    """Verify Boreas's `heading` against the actual direction of travel. Returns mean |error|.

    NEVER ASSUME A CONVENTION -- sub-project 1's lesson, learned three times. Applanix `heading`
    could plausibly be measured counter-clockwise from east (the maths convention) or clockwise
    from north (the survey convention), and picking wrong would rotate every scan into the map
    at the wrong angle, producing a drift number that says nothing about our back-end.

    So: compare the reported heading against atan2(dy, dx) over the ground-truth positions,
    while the vehicle is actually moving.
    """
    d = np.diff(poses[:, :2], axis=0)
    speed = np.linalg.norm(d, axis=1)
    moving = speed > 0.5 * np.median(speed[speed > 0])
    if moving.sum() < 10:
        log.warning("too few moving frames to check the yaw convention")
        return float("nan")
    course = np.arctan2(d[moving, 1], d[moving, 0])
    reported = poses[:-1, 2][moving]
    err = np.abs(np.angle(np.exp(1j * (course - reported))))
    return float(np.mean(err))


def main() -> None:
    files = sorted(glob.glob(f"{ROOT}/radar/*.png"))
    if not files:
        raise SystemExit(f"no scans under {ROOT}/radar -- run experiments/fetch_boreas.py first")
    log.info("found %d radar scans", len(files))

    ts_gt, poses_gt = boreas.load_gt_poses(open(f"{ROOT}/applanix/radar_poses.csv").read())
    by_ts = {int(t): p for t, p in zip(ts_gt, poses_gt)}

    # Join scans to poses by FILENAME -- GPSTime is exactly the PNG's name.
    keep, gt = [], []
    for f in files:
        t = int(os.path.splitext(os.path.basename(f))[0])
        if t in by_ts:
            keep.append(f)
            gt.append(by_ts[t])
    gt = np.array(gt)
    log.info("matched %d/%d scans to GT poses", len(keep), len(files))
    if len(keep) < 200:
        raise SystemExit("too few matched frames to measure 100 m sub-sequences")

    # --- the yaw convention, verified rather than assumed -----------------------------
    yaw_err = check_yaw_convention(gt)
    log.info("yaw convention check: mean |heading - course| = %.1f deg", np.rad2deg(yaw_err))
    if not (yaw_err < np.deg2rad(20)):
        log.warning("!! `heading` does NOT match the direction of travel. The convention is "
                    "not what we assumed -- fix boreas.load_gt_poses before trusting drift.")

    total_m = float(path_lengths(gt)[-1])
    log.info("trajectory: %.0f m over %d frames (%.1f s)", total_m, len(gt), len(gt) * DT)
    if total_m < 200.0:
        raise SystemExit(f"trajectory is only {total_m:.0f} m -- too short for KITTI drift")

    # --- load scans in parallel, motion-compensated using the GT speed ------------------
    # Motion compensation needs a velocity. We use the GT velocity: this measures the BACK-END,
    # not a velocity estimator, and CFEAR-class methods likewise rely on a motion estimate. It
    # is stated, not hidden.
    vel = np.zeros((len(gt), 2))
    vel[1:] = np.diff(gt[:, :2], axis=0) / DT
    vel[0] = vel[1]
    # rotate world velocity into the sensor-local frame of each scan
    c, s = np.cos(-gt[:, 2]), np.sin(-gt[:, 2])
    vel_local = np.stack([c * vel[:, 0] - s * vel[:, 1],
                          s * vel[:, 0] + c * vel[:, 1]], axis=1)

    def load(i):
        return boreas.load_radar_scan(keep[i], k=K, max_range_m=MAX_RANGE_M,
                                      velocity_xy=tuple(vel_local[i]))

    log.info("loading %d scans on %d cores ...", len(keep), os.cpu_count())
    t0 = time.time()
    with cf.ThreadPoolExecutor(max_workers=max(os.cpu_count() - 2, 1)) as pool:
        scans = list(pool.map(load, range(len(keep))))
    log.info("loaded in %.1f s; mean %.0f points/scan",
             time.time() - t0, np.mean([len(s) for s in scans]))

    # --- the SHARED back-end, UNCHANGED -------------------------------------------------
    t0 = time.time()

    def progress(f, n, npts, ncells):
        if f % 50 == 0 or f == n - 1:
            el = time.time() - t0
            eta = el / max(f, 1) * (n - f)
            log.info("  ICP %5d/%d  scan=%4d pts  map=%6d cells  elapsed %.0fs  ETA %.0fs",
                     f, n, npts, ncells, el, eta)

    # velocity=None -> the frame-agnostic adaptive constant-velocity motion model, which is
    # correct in the SLAM's own frame even though GT lives in a different one (the same choice
    # that fixed the KITTI run in paper 2).
    est, est_map = run_lidar_slam(scans, None, DT, np.random.default_rng(0),
                                  voxel=MAP_VOXEL, progress=progress)
    log.info("SLAM done in %.0f s", time.time() - t0)

    # --- score: KITTI drift at the STANDARD lengths (valid here -- km-scale) -------------
    d = drift(est, gt)
    r = rpe(est, gt)

    # aligned ATE, for context only (drift is the metric radar is judged on)
    R = _rigid_2d(est[:, :2], gt[:, :2])
    ate = float(np.sqrt(np.mean(np.sum(
        (_apply(est[:, :2], *R) - gt[:, :2]) ** 2, axis=1))))

    print("\n" + "=" * 74)
    print("RADAR CREDIBILITY ANCHOR -- Boreas, our shared scan-to-map ICP back-end")
    print("=" * 74)
    print(f"sequence            : {SEQ}")
    print(f"frames              : {len(gt)}   trajectory: {total_m:.0f} m")
    print(f"front-end           : k-strongest, k={K}, range <= {MAX_RANGE_M:.0f} m")
    print(f"motion compensation : ON (sweep is ~249 ms)")
    print()
    print(f"OUR drift           : {d['trans_pct']:.2f} % trans, "
          f"{d['rot_deg_per_100m']:.2f} deg/100m   ({d['n_segments']} segments)")
    print(f"OUR RPE             : {r:.3f} m/frame")
    print(f"OUR aligned ATE     : {ate:.1f} m")
    print()
    print("cited SOTA (NOT reimplemented):")
    for name, v in SOTA.items():
        print(f"  {v:5.2f} %  {name}")
    print()
    if d["per_length"]:
        print("per sub-sequence length:")
        for L, (t, rr) in sorted(d["per_length"].items()):
            print(f"  {L:3d} m : {t:6.2f} % trans, {rr:5.2f} deg/100m")

    # --- THE VERDICT, thresholds fixed in advance ---------------------------------------
    t = d["trans_pct"]
    if not np.isfinite(t):
        verdict = "FAIL (no drift computed -- trajectory too short or SLAM diverged)"
    elif t < 5.0:
        verdict = "PASS -- the baseline is credible. Proceed to sub-project 3."
    elif t < 10.0:
        verdict = ("MARGINAL -- report it and BOUND every claim: our back-end is not "
                   "CFEAR-class, so radar's true capability is UNDERSTATED here and the "
                   "WiFi-vs-radar gap we measure is a LOWER BOUND.")
    else:
        verdict = ("FAIL -- the radar baseline is a strawman. STOP; the ablation would be "
                   "built on sand. Fix the back-end or reconsider the paper.")
    print("\n" + "=" * 74)
    print(f"VERDICT: {verdict}")
    print("=" * 74)

    os.makedirs("results", exist_ok=True)
    out = {
        "sequence": SEQ, "n_frames": len(gt), "trajectory_m": total_m,
        "k": K, "max_range_m": MAX_RANGE_M, "motion_compensated": True,
        "yaw_convention_mean_err_deg": float(np.rad2deg(yaw_err)),
        "drift": d, "rpe_m": r, "aligned_ate_m": ate,
        "sota_cited": SOTA, "verdict": verdict,
    }
    with open("results/radar_anchor.json", "w") as f:
        json.dump(out, f, indent=2)
    print("saved -> results/radar_anchor.json")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Commit and push**

```bash
git add experiments/run_radar_anchor.py
git commit -m "paper3(radar): the credibility-gate anchor run on real Boreas radar

The shared scan-to-map ICP back-end, UNCHANGED, on real spinning radar, scored with the KITTI
protocol at the standard 100-800 m lengths (valid here -- Boreas is km-scale) and reported
beside the cited CFEAR/DRO rows with their caveats. Verdict thresholds are fixed IN ADVANCE so
they cannot be tuned after seeing the number. The yaw convention is verified against the
direction of travel rather than assumed."
git push
```

- [ ] **Step 3: Run it on the server**

```bash
nice -n 19 ionice -c3 .venv/bin/python experiments/run_radar_anchor.py 2>&1 | tee results/radar_anchor.log
```

Watch the ETA lines. If ICP is slower than ~0.3 s/frame, raise `MAP_VOXEL` (a radar map at 1.0 m
already has far more points than the LiDAR maps) — but **do not touch `slam_icp.py`**.

**Read the yaw-convention line first.** If it warns, everything downstream is meaningless: fix
`boreas.load_gt_poses` (the heading is probably clockwise-from-north, i.e. `yaw = pi/2 - heading`),
re-run, and only then look at the drift.

---

### Task 6: The verdict, written down

**Files:**
- Create: `docs/results-paper3-anchor.md`
- Copy back: `results/radar_anchor.json` from the server

- [ ] **Step 1: Pull the artifact back**

```bash
scp -P 33362 dev@78.89.209.212:/home/dev/mulham/wifi-radar-slam/results/radar_anchor.json results/
```

- [ ] **Step 2: Write `docs/results-paper3-anchor.md`**

It must contain, with the REAL numbers (no placeholders):

1. The setup: sequence, frame count, trajectory length, front-end (`k=12`, 100 m crop), motion
   compensation ON, and the fact that the back-end was used **unchanged**.
2. The yaw-convention check result (the measured mean error, and what it settled).
3. **Our drift %** and deg/100 m, with the per-sub-sequence-length breakdown.
4. The cited SOTA table **with the caveats spelled out** — CFEAR 1.09 % is *tuned*, on *Oxford*,
   radar-only; DRO 0.26 % is *gyro-aided* and *direct-intensity* and is **not** an apples-to-apples
   bound for a point-based radar-only method.
5. **The verdict — PASS / MARGINAL / FAIL — against the thresholds fixed in advance**, and what it
   means for the paper. If MARGINAL or FAIL, say exactly which claims must be bounded and how.
6. An honest list of what our back-end does *not* do that CFEAR does (no point-to-line metric, no
   oriented surface points, no Huber loss, no continuous-time trajectory, no Doppler, no gyro) —
   so a reviewer sees we know why we are behind, and by roughly how much we should expect to be.

- [ ] **Step 3: Commit**

```bash
git add docs/results-paper3-anchor.md results/radar_anchor.json
git commit -m "paper3(radar): credibility-anchor verdict on real Boreas radar"
git push
```

- [ ] **Step 4: Full suite green, then merge**

```bash
.venv/bin/python -m pytest -q
```

Expected: 153 prior + 21 new = **174 passed**.

```bash
git checkout paper3-wifi-vs-radar
git merge --no-ff paper3-sub2-anchor
.venv/bin/python -m pytest -q
git tag -a paper3-v0.2.0 -m "Paper 3, sub-project 2: radar credibility anchor"
git push origin paper3-wifi-vs-radar paper3-v0.2.0
```

**Only tag if the gate PASSED or is MARGINAL.** If it FAILED, do not tag — report and stop.

---

## Definition of done

- [ ] `radar/kstrongest.py` — the CFEAR-style front-end, generic over the power map, serving both
      real and simulated radar. Unit-tested.
- [ ] `radar/boreas.py` — polar loader (metadata decode, azimuth/range decode, GT poses) plus
      motion compensation. Unit-tested against a synthetic PNG built to the *verified* byte layout.
- [ ] The Boreas sequence fetched; one real file decoded and sanity-checked.
- [ ] The **unchanged** shared back-end run on real radar; the yaw convention *verified*, not assumed.
- [ ] Drift reported at the standard KITTI lengths, beside the cited SOTA **with its caveats**.
- [ ] A written verdict against thresholds fixed in advance.
- [ ] Full suite green.

---

## Self-review of this plan

**Spec coverage.** The spec's sub-project-2 row asks: run the shared back-end on a real radar
benchmark, report drift % beside cited CFEAR/DRO, and decide whether the baseline is defensible
*before* investing in the ablation. Task 1 → the front-end the spec's "two front-ends" section now
mandates; Tasks 2–3 → the real data; Task 4 → fetch; Task 5 → the run and the verdict; Task 6 → the
written record. The spec's demand that the back-end be shared *unchanged* is Global Constraint 1
and is load-bearing throughout.

**One spec correction, made rather than left to rot (Task 0):** the spec cited CFEAR 1.09 % and DRO
0.26 % side by side as comparable bounds. Verified against the primary sources, they are not — DRO
is gyro-aided and direct-intensity, and CFEAR's 1.09 % is the tuned figure on a *different dataset*.
Shipping that would have been a citation error of exactly the kind we caught in paper 2.

**Two risks this plan deliberately front-loads.** (a) **Motion distortion** — a 249 ms sweep at
15 m/s smears a scan by 3.7 m; without compensation the gate would fail for a reason unrelated to
our back-end, which is the worst possible experimental outcome (a true-looking negative). Hence
Task 3. (b) **The yaw convention** — sub-project 1 taught us three times that assumed conventions
are silently wrong, so Task 5 *measures* `heading` against the direction of travel before trusting
a single drift number.

**Type consistency.** `k_strongest(power, ranges, azimuths, k, min_range_m, max_range_m, z_min)` is
defined in Task 1 and called with exactly that signature by `boreas.load_radar_scan` (Task 2).
`decode_polar` returns `(power, azimuths, timestamps_us, valid)` in Task 2 and is unpacked in that
order in Tasks 3 and 5. `drift(est, gt)` matches `eval/metrics`' `(est, gt)` order, as fixed in
sub-project 1. `Scan` is the existing `lidar.pointcloud.Scan` throughout — never a new type.
