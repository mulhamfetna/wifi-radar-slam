# Paper 4 · Sub-projects 1+2 — the $40 self-contained ESP32 WiFi radar

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a vehicle-mounted, infrastructure-free WiFi radar from two ESP32s, and measure the **phantom rate on a real channel** — the measurement paper 2 named as the single most valuable thing missing from this whole programme.

**Architecture:** Two ESP32s ride on the vehicle: one **illuminates**, one **receives CSI** 30–50 cm away. The CSI's delay taps *are* echo ranges — a 1-D radar. The ESP32s only **stream**; all processing is pure Python offline, reusing the **same** detection chain and the **same** phantom definition as papers 2–3, so the numbers are directly comparable. Nothing is installed in the building.

**Tech Stack:** ESP-IDF (C) for two tiny firmwares; Python 3 + NumPy/SciPy for the offline pipeline; pytest.

**Test runner:** `.venv/bin/python -m pytest` (a bare `python3` has no venv).

**Branch:** `paper4-sub1-esp32-sensor`, **off `paper3-sub3-ablation`** — see Task 0, this is not a typo.

**Spec:** `docs/superpowers/specs/2026-07-13-paper4-hardware-testbed-design.md`

---

## Global Constraints

1. **This is a SLAM sensor, not a positioning service.** No anchors, no towers, no surveyed beacon mesh. **Everything rides on the vehicle.** Any design that needs transmitters installed and surveyed in the environment is **disqualified** — that is infrastructure-based localisation, not SLAM.
2. **The bistatic control costs nothing.** On the same drive, also log an access point **that already exists in the building** (position measured once). One drive, two geometries — paper 3's A-vs-B geometry ablation, physically.
3. **We validate the READING, not a SLAM system.** Poses **measured along a physical track**; map **surveyed**. No estimator — exactly as paper 3 scores its ablation under ground-truth poses.

   **NOT wheel encoders.** Verified (Borenstein & Feng, UMBmark, Table I read directly):
   uncalibrated odometry drifts **1.9 % of distance travelled** — *"the robot's internal position
   estimate is **totally wrong after as little as 10 m of travel**"* — and a single **10 mm bump
   under one wheel is 0.6° of PERMANENT yaw error**. Use a **taped/measured straight track** (or a
   rail): the pose is then a number read off a tape, with **zero** accumulating drift.

   **And the thing odometry is worst at is the thing we do not need.** With one RF chain there is
   **no AoA**, so the sensor measures **range only** — and the excess path length depends on
   **position alone, not heading**. Yaw enters solely through the 0.4 m TX offset, where a 5° error
   displaces the transmitter by **3.5 cm** against a **3.75 m** resolution cell. **Yaw is
   irrelevant here**, which is precisely why a tape measure beats a robot.

   **The admissibility rule, with a citation** (*Benchmarking Egocentric Visual-Inertial SLAM at
   City Scale*, arXiv:2509.26639): a reference is valid when it is **several times better than the
   errors it must resolve** — that work accepts a ~20 cm reference to resolve >50 cm errors, a
   **2.5x** margin. A tape-measured track at **~1 cm** against a WiFi map of **3.75 m** resolution
   is a **~375x** margin. The objection is closed before a reviewer raises it.
4. **REUSE the existing chain.** `eval/phantom.py` (the *identical* phantom definition), `sensing/superres.estimate_delays` (the *identical* MUSIC), and the same CA-CFAR statistic. **A different chain would confound the hardware result with an algorithm change** — which is the one thing this experiment must not do.
5. **The Python side must be fully testable with SYNTHETIC CSI.** Development and CI require **no hardware**. Hardware-touching code is gated, exactly as the Sionna sensors are.
6. **Phase 1 streams to a laptop. That is the correct development order, not a dependency.** You cannot tune an algorithm you cannot see, and this phase must compare MUSIC against CFAR. Phase 2 (a later sub-project) ports the winner on-chip.

---

## The physics that decides everything (verified — see the spec)

| | |
|---|---|
| ESP32 | **2.4 GHz only**, HT40 = **40 MHz** CSI, **one RF chain** (so **no AoA, ever**, from one chip) |
| **Monostatic** round-trip range resolution | **c/2B = 3.75 m** |
| **Bistatic** path-length resolution | **c/B = 7.5 m** |
| Absolute time-of-flight | **NOT measurable** — packet-detection delay swamps it |
| **Excess** delay (relative to the first/LOS arrival) | **PRESERVED** — STO/SFO enter as a phase ramp **common to all paths in a packet** |

> **The last row is the single fact this entire programme rests on.** Our method needs the *excess* path length, not the absolute one. That is why a $5 chip can run it.

**Why 2.4 GHz is legitimate:** paper 3 proved the **carrier does nothing**. We are spending a result we earned, not cutting a corner.

**Why the monostatic geometry is twice as forgiving:** a round trip traverses the distance twice, so 3.75 m vs 7.5 m. A *second*, independent reason the transmitter belongs on the vehicle.

## 🔴 The make-or-break constraint: the site must be BIG

At 3.75 m resolution, a reflector must sit **beyond ~3.75 m** to leave the direct-path bin at all.

> **A small lab collapses every echo into the direct-path bin and measures literally nothing.** The site must be a corridor, sports hall, or car park, with large surveyed reflectors well off-axis. **This is geometry, not budget, and no amount of money fixes it.**

---

## File Structure

| File | Responsibility |
|---|---|
| `firmware/esp32_csi_rx/main/main.c` | RX: enable HT40 CSI, stream `(timestamp, rssi, csi_bytes)` as CSV over serial. Minimal. |
| `firmware/esp32_tx_beacon/main/main.c` | TX: emit packets at a steady rate. That is all it does. |
| `src/wifi_radar_slam/hw/csi.py` | Parse the ESP32 CSI stream → complex CSI matrix. **The byte layout is a HYPOTHESIS until validated on real hardware** (Task 7). |
| `src/wifi_radar_slam/hw/delay.py` | CSI → delay profile (IFFT) → echo excess-path-lengths. **Two front-ends: CFAR and MUSIC.** |
| `src/wifi_radar_slam/hw/truth.py` | Surveyed reflectors + measured pose → **predicted** excess path lengths (monostatic *and* bistatic). |
| `experiments/run_hw_phantom.py` | The measurement: phantom rate + range bias · MUSIC vs CFAR · monostatic vs bistatic. |
| `experiments/validate_esp32_csi.py` | **The gate.** Verify the byte layout against a real capture before trusting a single number. |
| `docs/hardware-build.md` | Bill of materials, wiring, survey procedure, **the site-size requirement**. |
| `tests/test_hw_csi.py`, `tests/test_hw_delay.py`, `tests/test_hw_truth.py` | Unit tests, all on **synthetic** CSI. |

---

### Task 0: Branch — off `paper3-sub3-ablation`, and this matters

**Why not off `paper4-hardware-testbed`:** `eval/phantom.py` — the phantom definition we are *required* to reuse verbatim — lives on `paper3-sub3-ablation`, which is **not yet merged** (its experiments are still running). Branching from `paper4-hardware-testbed` would leave us without it, and re-implementing the definition would silently break comparability with paper 2's ~89 % and paper 3's numbers. **That is the whole point of this experiment, so we branch where the code is.**

- [ ] **Step 1: Cut the branch and confirm the dependency is present**

```bash
cd /mnt/data/projects/wifi-radar
git fetch origin
git checkout paper3-sub3-ablation
git pull --ff-only
git checkout -b paper4-sub1-esp32-sensor

# the two things we MUST inherit:
test -f src/wifi_radar_slam/eval/phantom.py && echo "OK phantom.py (the identical definition)"
grep -q "def estimate_delays" src/wifi_radar_slam/sensing/superres.py && echo "OK estimate_delays (the identical MUSIC)"
.venv/bin/python -m pytest -q
```

Expected: both `OK` lines, and the suite green (200 passed).

If `phantom.py` is missing, **stop** — you are on the wrong branch, and proceeding would fork the phantom definition.

---

### Task 1: Parse the ESP32 CSI stream

**Files:**
- Create: `src/wifi_radar_slam/hw/__init__.py`
- Create: `src/wifi_radar_slam/hw/csi.py`
- Test: `tests/test_hw_csi.py`

**Interfaces:**
- Consumes: nothing (pure NumPy).
- Produces:
  - `ESP32_HT40_SUBCARRIERS = 128`, `ESP32_HT40_BANDWIDTH_HZ = 40e6`
  - `parse_csi_bytes(raw: bytes | np.ndarray, imag_first: bool = True) -> np.ndarray` — `(n_subcarriers,)` complex.
  - `parse_csi_csv(text: str, imag_first: bool = True) -> tuple[np.ndarray, np.ndarray]` — `(timestamps_us (n,), csi (n, n_sub) complex)`.

**⚠ The byte layout is a HYPOTHESIS.** ESP-IDF packs CSI as **signed int8 pairs**, and the pairs are believed to be **(imag, real)** — *not* (real, imag). Getting that backwards conjugates the channel, which **mirrors the delay profile** and would look like a physics result. `imag_first` is a parameter, defaulted to the hypothesis, and **Task 7 settles it against a real capture.** Do not hard-code it.

- [ ] **Step 1: Write the failing test**

Create `tests/test_hw_csi.py`:

```python
import numpy as np
import pytest
from wifi_radar_slam.hw.csi import (parse_csi_bytes, parse_csi_csv,
                                    ESP32_HT40_SUBCARRIERS, ESP32_HT40_BANDWIDTH_HZ)


def test_constants_match_the_esp32_ht40_mode():
    # HT40 CSI: 128 subcarriers over 40 MHz. This is what licenses a 3.75 m monostatic
    # range resolution (c/2B) -- the number the whole experiment lives or dies on.
    assert ESP32_HT40_SUBCARRIERS == 128
    assert ESP32_HT40_BANDWIDTH_HZ == 40e6


def test_bytes_are_int8_PAIRS_and_imag_comes_FIRST():
    # THE trap. ESP-IDF packs (imag, real), not (real, imag). Reading it backwards
    # CONJUGATES the channel, which MIRRORS the delay profile -- and a mirrored delay
    # profile looks like a physics result rather than a parsing bug.
    raw = np.array([3, 4, -1, 2], dtype=np.int8)      # (im=3,re=4), (im=-1,re=2)
    got = parse_csi_bytes(raw, imag_first=True)
    assert got.shape == (2,)
    assert got[0] == pytest.approx(4 + 3j)
    assert got[1] == pytest.approx(2 - 1j)


def test_the_imag_first_flag_actually_swaps():
    raw = np.array([3, 4], dtype=np.int8)
    assert parse_csi_bytes(raw, imag_first=False)[0] == pytest.approx(3 + 4j)


def test_odd_byte_count_is_rejected():
    with pytest.raises(ValueError, match="pairs"):
        parse_csi_bytes(np.array([1, 2, 3], dtype=np.int8))


def test_values_are_signed_not_unsigned():
    # int8, so 0xFF is -1, not 255. Reading it unsigned would put every subcarrier in the
    # wrong half-plane and destroy the phase -- which is the only thing we actually use.
    raw = np.array([0xFF, 0x01], dtype=np.uint8).view(np.int8)
    got = parse_csi_bytes(raw, imag_first=True)
    assert got[0] == pytest.approx(1 - 1j)


def test_parse_csv_extracts_timestamps_and_csi():
    # The ESP32-CSI-Tool line format: fields, then a bracketed CSI array.
    line = "CSI_DATA,1234567,-42,[3 4 -1 2]"
    ts, csi = parse_csi_csv(line)
    assert ts.tolist() == [1234567]
    assert csi.shape == (1, 2)
    assert csi[0, 0] == pytest.approx(4 + 3j)


def test_parse_csv_handles_many_rows_and_skips_junk():
    text = "\n".join([
        "garbage line that is not CSI",
        "CSI_DATA,100,-40,[1 2 3 4]",
        "",
        "CSI_DATA,200,-41,[5 6 7 8]",
    ])
    ts, csi = parse_csi_csv(text)
    assert ts.tolist() == [100, 200]
    assert csi.shape == (2, 2)


def test_rows_of_differing_length_are_rejected_not_silently_truncated():
    # A short row means a dropped/corrupt packet. Silently padding or truncating it would
    # shift every subcarrier and corrupt the delay profile without any error.
    text = "CSI_DATA,100,-40,[1 2 3 4]\nCSI_DATA,200,-41,[1 2]"
    with pytest.raises(ValueError, match="inconsistent"):
        parse_csi_csv(text)


def test_empty_input_gives_empty_arrays():
    ts, csi = parse_csi_csv("")
    assert ts.shape == (0,) and csi.shape == (0, 0)
```

- [ ] **Step 2: Run the test and watch it fail**

```bash
.venv/bin/python -m pytest tests/test_hw_csi.py -q
```

Expected: `ModuleNotFoundError: No module named 'wifi_radar_slam.hw'`.

- [ ] **Step 3: Implement**

Create `src/wifi_radar_slam/hw/__init__.py`:

```python
"""The real-hardware arm: a $40 self-contained ESP32 WiFi radar (paper 4).

Two ESP32s ride on the vehicle -- one illuminates, one receives CSI 30-50 cm away. The CSI's
delay taps ARE echo ranges: a 1-D radar. NOTHING is installed in the building; this is a SLAM
sensor, not a positioning service.

Everything here is pure NumPy/SciPy and tests on SYNTHETIC CSI, so the pipeline develops with
no hardware at all.
"""
```

Create `src/wifi_radar_slam/hw/csi.py`:

```python
"""Parse the ESP32's CSI stream into a complex channel matrix.

THE BYTE LAYOUT IS A HYPOTHESIS UNTIL experiments/validate_esp32_csi.py CONFIRMS IT.

ESP-IDF packs CSI as **signed int8 pairs**, and the pairs are believed to be **(imag, real)** --
NOT (real, imag). Reading them backwards CONJUGATES the channel, which MIRRORS the delay
profile. A mirrored delay profile does not look like a bug; it looks like a physics result. So
`imag_first` is a parameter, not a constant, and it is settled by measurement.

HT40 gives 128 subcarriers across 40 MHz. That is what licenses a **3.75 m monostatic range
resolution** (c/2B) -- the number this whole experiment lives or dies on.
"""
from __future__ import annotations
import re
import numpy as np

ESP32_HT40_SUBCARRIERS = 128
ESP32_HT40_BANDWIDTH_HZ = 40e6

_ROW = re.compile(r"CSI_DATA,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*\[([^\]]*)\]")


def parse_csi_bytes(raw, imag_first: bool = True) -> np.ndarray:
    """int8 pairs -> (n_subcarriers,) complex.

    `imag_first=True` reads each pair as (imag, real), which is the ESP-IDF convention we
    believe holds. It is VERIFIED, not assumed -- see the module docstring.
    """
    a = np.asarray(raw, dtype=np.int8).ravel()
    if a.size % 2 != 0:
        raise ValueError(f"CSI must be int8 pairs; got {a.size} bytes (odd)")
    first, second = a[0::2].astype(float), a[1::2].astype(float)
    return (second + 1j * first) if imag_first else (first + 1j * second)


def parse_csi_csv(text: str, imag_first: bool = True):
    """The ESP32-CSI-Tool serial stream -> (timestamps_us (n,), csi (n, n_sub) complex).

    Lines that are not CSI rows are ignored (the ESP32 also prints boot chatter). Rows of
    DIFFERING length are an ERROR, never silently padded: a short row is a dropped or corrupt
    packet, and padding it would shift every subcarrier and corrupt the delay profile with no
    error at all.
    """
    ts, rows = [], []
    for m in _ROW.finditer(text):
        vals = m.group(3).split()
        if not vals:
            continue
        ts.append(int(m.group(1)))
        rows.append(np.array([int(v) for v in vals], dtype=np.int8))
    if not rows:
        return np.empty(0, dtype=np.int64), np.empty((0, 0), dtype=complex)

    n = rows[0].size
    if any(r.size != n for r in rows):
        sizes = sorted({int(r.size) for r in rows})
        raise ValueError(f"inconsistent CSI row lengths {sizes} -- dropped/corrupt packets; "
                         f"they must not be silently padded")

    csi = np.stack([parse_csi_bytes(r, imag_first=imag_first) for r in rows])
    return np.array(ts, dtype=np.int64), csi
```

- [ ] **Step 4: Run the tests and watch them pass**

```bash
.venv/bin/python -m pytest tests/test_hw_csi.py -q
```

Expected: 8 passed.

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/hw/ tests/test_hw_csi.py
git commit -m "paper4(hw): parse the ESP32 CSI stream

The byte layout is a HYPOTHESIS until validated against a real capture. ESP-IDF packs signed
int8 pairs believed to be (imag, real) -- reading them backwards CONJUGATES the channel, which
MIRRORS the delay profile, and a mirrored profile looks like a physics result rather than a
parsing bug. So imag_first is a parameter, not a constant.

Rows of differing length are an ERROR, never padded: a short row is a dropped packet, and
padding it would shift every subcarrier and corrupt the delay profile silently."
```

---

### Task 2: CSI → delay profile → echo ranges (BOTH front-ends)

**Files:**
- Create: `src/wifi_radar_slam/hw/delay.py`
- Test: `tests/test_hw_delay.py`

**Interfaces:**
- Consumes: `hw.csi` constants; `sensing.superres.estimate_delays` (the **identical** MUSIC papers 1–2 use — signature verified: `estimate_delays(block, bandwidth_hz, n_paths, max_range_m=None) -> np.ndarray` of delays in seconds).
- Produces:
  - `path_length_bins(n_sub, bandwidth_hz) -> np.ndarray` — the excess-path-length axis (m).
  - `delay_profile(csi, window=True) -> np.ndarray` — real power vs delay bin.
  - `cfar_excess_lengths(csi, bandwidth_hz, pfa=1e-3, guard=2, train=8) -> np.ndarray` — detected **excess** path lengths (m), CFAR front-end.
  - `music_excess_lengths(csi, bandwidth_hz, n_paths=3, max_path_m=60.0) -> np.ndarray` — detected **excess** path lengths (m), MUSIC front-end.

**Both front-ends return the SAME quantity — excess path length in metres — so `eval/phantom.py` scores them identically.** That is what makes MUSIC-vs-CFAR a clean single-variable comparison.

- [ ] **Step 1: Write the failing test**

Create `tests/test_hw_delay.py`:

```python
import numpy as np
import pytest
from wifi_radar_slam.hw.csi import ESP32_HT40_BANDWIDTH_HZ as B
from wifi_radar_slam.hw.delay import (path_length_bins, delay_profile,
                                      cfar_excess_lengths, music_excess_lengths)

C = 299792458.0
N = 128


def synth_csi(excess_m, amps=None, n_sub=N, bandwidth_hz=B, rng=None, noise=0.0,
              global_phase=0.0):
    """Synthetic CSI: H(f) = sum_k a_k exp(-j 2 pi f tau_k), plus the direct path at tau=0.

    `global_phase` fakes the per-packet STO/CFO offset that real CSI carries -- a phase ramp
    COMMON to every path. Our tests assert it does not move the ANSWER, because the answer is
    an EXCESS delay.
    """
    f = np.arange(n_sub) * (bandwidth_hz / n_sub)
    h = np.exp(-1j * 2 * np.pi * f * 0.0)                      # the direct TX->RX path
    for i, d in enumerate(np.atleast_1d(excess_m)):
        a = 0.5 if amps is None else np.atleast_1d(amps)[i]
        h = h + a * np.exp(-1j * 2 * np.pi * f * (d / C))
    h = h * np.exp(1j * global_phase)
    if noise and rng is not None:
        h = h + (noise / np.sqrt(2)) * (rng.normal(size=n_sub) + 1j * rng.normal(size=n_sub))
    return h


def test_path_length_bin_spacing_is_c_over_B():
    # THE resolution fact. A path-length estimate at bandwidth B resolves to c/B -- NOT c/2B,
    # which is the MONOSTATIC RANGE resolution (range = path length / 2). Confusing the two is
    # exactly the factor-of-2 error we had to correct in paper 2.
    bins = path_length_bins(N, B)
    assert bins.shape == (N,)
    assert bins[0] == pytest.approx(0.0)
    assert np.diff(bins)[0] == pytest.approx(C / B)
    assert np.diff(bins)[0] == pytest.approx(7.49, abs=0.01)


def test_delay_profile_peaks_at_the_direct_path_when_there_are_no_echoes():
    p = delay_profile(synth_csi([]))
    assert int(np.argmax(p)) == 0


def test_delay_profile_shows_an_echo_at_its_excess_path_length():
    p = delay_profile(synth_csi([30.0], amps=[0.8]))
    bins = path_length_bins(N, B)
    # the echo is at 30 m of EXCESS path -- within one 7.5 m bin
    peak = bins[int(np.argmax(p[1:])) + 1]
    assert peak == pytest.approx(30.0, abs=C / B)


def test_CFAR_finds_a_strong_echo_and_reports_its_EXCESS_path_length():
    rng = np.random.default_rng(0)
    got = cfar_excess_lengths(synth_csi([30.0], amps=[1.0], rng=rng, noise=0.02), B)
    assert len(got) >= 1
    assert np.min(np.abs(got - 30.0)) < C / B


def test_CFAR_on_a_direct_path_ONLY_reports_no_echo():
    # A clean LOS-only channel must produce NO echo detections. If it does, the detector is
    # manufacturing phantoms out of its own sidelobes and RQ1 is uninterpretable.
    rng = np.random.default_rng(1)
    got = cfar_excess_lengths(synth_csi([], rng=rng, noise=0.02), B)
    assert len(got) == 0


def test_MUSIC_finds_the_same_echo():
    got = music_excess_lengths(synth_csi([30.0], amps=[1.0]), B, n_paths=2)
    assert np.min(np.abs(got - 30.0)) < C / B


def test_MUSIC_EMITS_A_FIXED_NUMBER_OF_PEAKS_EVEN_WITH_NO_ECHOES():
    # THE MECHANISM THE WHOLE PAPER IS ABOUT. MUSIC has a FIXED MODEL ORDER: asked for 3 paths
    # it returns 3 peaks WHETHER OR NOT 3 PATHS EXIST. On a direct-path-only channel it must
    # therefore invent phantoms -- while CFAR (above) correctly returns none.
    #
    # This is paper 2's ~89% in a single test, and it is why the cheap detector beats the
    # expensive one.
    music = music_excess_lengths(synth_csi([]), B, n_paths=3)
    cfar = cfar_excess_lengths(synth_csi([]), B)
    assert len(music) == 3        # three peaks conjured from a channel with one path
    assert len(cfar) == 0         # the calibrated detector reports nothing, correctly


def test_a_PER_PACKET_PHASE_OFFSET_does_not_move_the_excess_delays():
    # THE FACT THE WHOLE PROGRAMME RESTS ON. Real CSI carries an unknown per-packet phase
    # (STO/CFO), so ABSOLUTE time-of-flight is unmeasurable. But that offset is COMMON to every
    # path in the packet, so delays RELATIVE to the direct arrival survive -- and the excess is
    # exactly what our method needs.
    clean = cfar_excess_lengths(synth_csi([30.0], amps=[1.0]), B)
    offset = cfar_excess_lengths(synth_csi([30.0], amps=[1.0], global_phase=1.234), B)
    assert len(clean) == len(offset)
    assert np.allclose(np.sort(clean), np.sort(offset), atol=1e-9)


def test_empty_csi_gives_no_detections():
    assert len(cfar_excess_lengths(np.empty(0), B)) == 0
```

- [ ] **Step 2: Run the test and watch it fail**

```bash
.venv/bin/python -m pytest tests/test_hw_delay.py -q
```

Expected: `ModuleNotFoundError: No module named 'wifi_radar_slam.hw.delay'`.

- [ ] **Step 3: Implement**

Create `src/wifi_radar_slam/hw/delay.py`:

```python
"""CSI -> delay profile -> echo EXCESS path lengths. Two front-ends: CFAR and MUSIC.

BOTH front-ends return the SAME quantity -- excess path length in metres -- so eval/phantom.py
scores them identically. That is what makes MUSIC-vs-CFAR a clean single-variable comparison,
and it is the whole reason this module exists rather than two separate ones.

WHY "EXCESS" AND NOT ABSOLUTE. Commodity CSI carries an unknown per-packet sampling/carrier
offset, so ABSOLUTE time-of-flight is not measurable at all. But that offset enters as a phase
ramp COMMON to every path in the packet, so delays RELATIVE to the first (direct) arrival
survive intact -- and the excess path length is exactly what our bistatic ellipse and our
monostatic range both need. This single fact is why a $5 chip can run the method.

RESOLUTION. A path-length estimate at bandwidth B resolves to c/B = 7.5 m at 40 MHz. That is
NOT c/2B = 3.75 m, which is the MONOSTATIC RANGE resolution (range = path length / 2).
Confusing the two is exactly the factor-of-2 error that had to be corrected in paper 2.
"""
from __future__ import annotations
import numpy as np
from scipy.ndimage import uniform_filter1d

from ..sensing.superres import estimate_delays

C = 299792458.0


def path_length_bins(n_sub: int, bandwidth_hz: float) -> np.ndarray:
    """Excess path length (m) of each delay bin. Spacing is exactly c/B."""
    return np.arange(n_sub) * (C / bandwidth_hz)


def delay_profile(csi: np.ndarray, window: bool = True) -> np.ndarray:
    """CSI across subcarriers -> power vs delay bin.

    The Hann window is load-bearing for the same reason it is in radar/processing.range_fft: a
    rectangular window's -13 dB sidelobes trip the detector at delays where nothing exists,
    manufacturing phantoms out of our own processing -- and the phantom rate is the number this
    paper reports.
    """
    h = np.asarray(csi, dtype=complex).ravel()
    if h.size == 0:
        return np.empty(0)
    w = np.hanning(h.size) if window else np.ones(h.size)
    return np.abs(np.fft.ifft(h * w)) ** 2


def cfar_excess_lengths(csi: np.ndarray, bandwidth_hz: float, pfa: float = 1e-3,
                        guard: int = 2, train: int = 8) -> np.ndarray:
    """CA-CFAR on the delay profile -> echo EXCESS path lengths (m). Bin 0 (the direct path) is
    excluded by construction -- it is the reference, not an echo.

    The threshold uses the SAME statistic as radar/processing.cfar_2d:

        alpha = N * (Pfa**(-1/N) - 1)

    so the detector is *identical* to the one the simulation used, just in one dimension. Using
    a different detector here would confound the hardware result with an algorithm change --
    the one thing this experiment must not do.
    """
    p = delay_profile(csi)
    if p.size == 0:
        return np.empty(0)

    w_full = 2 * (guard + train) + 1
    w_guard = 2 * guard + 1
    n_train = w_full - w_guard
    if n_train <= 0:
        raise ValueError("CFAR training region is empty; increase `train`")

    s_full = uniform_filter1d(p, size=w_full, mode="nearest") * w_full
    s_guard = uniform_filter1d(p, size=w_guard, mode="nearest") * w_guard
    noise = (s_full - s_guard) / n_train

    alpha = n_train * (pfa ** (-1.0 / n_train) - 1.0)
    mask = p > alpha * noise
    mask[0] = False                       # bin 0 IS the direct path: the reference, not an echo

    bins = path_length_bins(p.size, bandwidth_hz)
    return bins[mask]


def music_excess_lengths(csi: np.ndarray, bandwidth_hz: float, n_paths: int = 3,
                         max_path_m: float = 60.0) -> np.ndarray:
    """MUSIC on the delay axis -> echo EXCESS path lengths (m).

    Reuses `sensing.superres.estimate_delays` -- the IDENTICAL estimator papers 1-2 ran on
    simulated CSI -- so any difference we measure is the CHANNEL, not the algorithm.

    NOTE THE PATHOLOGY, WHICH IS THE POINT OF THE WHOLE PAPER. MUSIC has a FIXED MODEL ORDER:
    asked for `n_paths` it returns `n_paths` peaks WHETHER OR NOT that many paths exist. On a
    direct-path-only channel it therefore INVENTS peaks, while a calibrated detector correctly
    reports none. That is paper 2's ~89 % phantom rate, in one sentence.

    Delays come back relative to the estimator's grid origin; we subtract the first (direct)
    arrival to obtain the EXCESS, which is the quantity everything downstream compares.
    """
    h = np.asarray(csi, dtype=complex).ravel()
    if h.size == 0:
        return np.empty(0)
    taus = np.sort(estimate_delays(h[None, :], bandwidth_hz, n_paths,
                                   max_range_m=max_path_m))
    if taus.size == 0:
        return np.empty(0)
    excess = (taus - taus[0]) * C          # relative to the direct arrival
    return excess[1:]                      # drop the reference itself
```

- [ ] **Step 4: Run the tests and watch them pass**

```bash
.venv/bin/python -m pytest tests/test_hw_delay.py -q
```

Expected: 9 passed.

**If `test_MUSIC_EMITS_A_FIXED_NUMBER_OF_PEAKS_EVEN_WITH_NO_ECHOES` fails, do not "fix" it by
lowering `n_paths`.** That test *is* the paper's mechanism: MUSIC's fixed model order is exactly
what manufactures phantoms, and the test exists to prove the mechanism reproduces before we ever
touch hardware.

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/hw/delay.py tests/test_hw_delay.py
git commit -m "paper4(hw): CSI -> delay profile -> echo excess path lengths, CFAR and MUSIC

Both front-ends return the SAME quantity (excess path length in metres), so eval/phantom.py
scores them identically and MUSIC-vs-CFAR is a clean single-variable comparison.

The CFAR uses the SAME alpha = N(Pfa^(-1/N) - 1) statistic as radar/processing.cfar_2d, and the
MUSIC is sensing.superres.estimate_delays verbatim -- the identical estimator papers 1-2 ran on
simulated CSI. Any difference we measure is therefore the CHANNEL, not the algorithm.

Two tests carry the paper's whole thesis:
  - MUSIC asked for 3 paths returns 3 peaks on a channel with ONE path (it invents phantoms),
    while CFAR correctly returns none. That is paper 2's ~89% in a single assertion.
  - A per-packet phase offset (STO/CFO) does not move the EXCESS delays -- which is the fact
    that makes the whole method runnable on a \$5 chip."
```

---

### Task 2b: 🔴 Divide out the INSTRUMENT — or measure its phantoms instead of the world's

**Files:**
- Create: `src/wifi_radar_slam/hw/calib.py`
- Test: `tests/test_hw_calib.py`

**Interfaces:**
- Consumes: `hw.delay.delay_profile` (Task 2).
- Produces:
  - `reference_from_los(csi_los) -> np.ndarray` — `(n_sub,)` complex instrument+direct-path reference.
  - `apply_reference(csi, ref) -> np.ndarray` — the calibrated channel.

**WHY THIS EXISTS, AND WHY IT IS NOT OPTIONAL.** Every receiver has a **frequency-selective analog
response** — the `Hdist` term. PicoScenes measured **>15 dB of magnitude swing across subcarriers**
on commodity NICs and states, in their own words, that it ***"causes a phantom object that
interferes with the H_air measurement."*** It is **not** removed by SpotFi-style linear-fit
sanitisation.

> **An uncalibrated receiver manufactures phantom taps out of its own filter.** Since **the phantom
> rate is the number this entire paper reports**, shipping that uncorrected would mean measuring
> *our instrument* and calling it *the world*. This is the same class of self-inflicted artifact
> that had to be hunted out of paper 3 three separate times (the untapered aperture, CFAR
> self-masking, the periodic azimuth axis).

**The correction is free**, because the LOS-only capture of Task 7 already measures it. In an open
space with the TX ~40 cm away and no reflector within 10 m, the channel is essentially one path, so

```
H_los(f)  ≈  H_dist(f) · a · exp(-j 2 pi f tau_0)
```

Dividing any later capture by `H_los` removes **both** the instrument response **and** the direct
path in one stroke, leaving `1 + (echoes relative to the direct arrival)`. The IFFT of that is a
spike at bin 0 plus the **echo taps we actually want** — and the excess delays are, by construction,
exactly the quantity `hw/truth.py` predicts.

- [ ] **Step 1: Write the failing test**

Create `tests/test_hw_calib.py`:

```python
import numpy as np
import pytest
from wifi_radar_slam.hw.calib import reference_from_los, apply_reference
from wifi_radar_slam.hw.delay import delay_profile, cfar_excess_lengths
from wifi_radar_slam.hw.csi import ESP32_HT40_BANDWIDTH_HZ as B

C = 299792458.0
N = 128


def _instrument(n_sub=N, swing_db=15.0, rng=None):
    """A frequency-selective receiver response -- the `Hdist` term. PicoScenes measured >15 dB
    of swing across subcarriers on commodity NICs, and reported it manufactures a PHANTOM."""
    f = np.linspace(0, 1, n_sub)
    mag = 10 ** ((swing_db / 2) * np.sin(2 * np.pi * 2.5 * f) / 20.0)
    phase = 1.3 * np.sin(2 * np.pi * 1.7 * f + 0.4)      # not a linear ramp: a real filter
    return mag * np.exp(1j * phase)


def _channel(excess_m, n_sub=N, bandwidth_hz=B):
    f = np.arange(n_sub) * (bandwidth_hz / n_sub)
    h = np.ones(n_sub, dtype=complex)                     # the direct path
    for d in np.atleast_1d(excess_m):
        h = h + 0.6 * np.exp(-1j * 2 * np.pi * f * (d / C))
    return h


def test_an_UNCALIBRATED_instrument_MANUFACTURES_phantom_taps():
    # THE HAZARD. A LOS-only channel has exactly ONE path. But pushed through a
    # frequency-selective receiver, its delay profile sprouts extra taps -- and CFAR dutifully
    # detects them. Those are the INSTRUMENT's phantoms, not the world's, and they would inflate
    # the very number this paper reports.
    los = _channel([]) * _instrument()
    bogus = cfar_excess_lengths(los, B)
    assert len(bogus) > 0, "expected the instrument response to fabricate taps"


def test_dividing_by_the_LOS_REFERENCE_removes_them():
    ref = reference_from_los(_channel([]) * _instrument())
    clean = apply_reference(_channel([]) * _instrument(), ref)
    assert len(cfar_excess_lengths(clean, B)) == 0        # one path in, no echoes out


def test_calibration_PRESERVES_a_real_echo():
    inst = _instrument()
    ref = reference_from_los(_channel([]) * inst)
    got = cfar_excess_lengths(apply_reference(_channel([30.0]) * inst, ref), B)
    assert len(got) >= 1
    assert np.min(np.abs(got - 30.0)) < C / B


def test_the_reference_is_averaged_over_frames_to_beat_down_noise():
    rng = np.random.default_rng(0)
    frames = np.stack([_channel([]) * _instrument()
                       + 0.01 * (rng.normal(size=N) + 1j * rng.normal(size=N))
                       for _ in range(64)])
    ref = reference_from_los(frames)
    assert ref.shape == (N,)
    # the averaged reference must be closer to the truth than any single noisy frame
    truth = _channel([]) * _instrument()
    assert np.linalg.norm(ref - truth) < np.linalg.norm(frames[0] - truth)


def test_a_zero_in_the_reference_does_not_blow_up():
    # A deep notch in the receiver response would divide by ~0 and inject a spike.
    ref = np.ones(N, dtype=complex)
    ref[10] = 0.0
    out = apply_reference(_channel([]), ref)
    assert np.all(np.isfinite(out))


def test_shape_mismatch_is_rejected():
    with pytest.raises(ValueError):
        apply_reference(np.ones(N, dtype=complex), np.ones(N + 1, dtype=complex))
```

- [ ] **Step 2: Run the test and watch it fail**

```bash
.venv/bin/python -m pytest tests/test_hw_calib.py -q
```

Expected: `ModuleNotFoundError: No module named 'wifi_radar_slam.hw.calib'`.

- [ ] **Step 3: Implement**

Create `src/wifi_radar_slam/hw/calib.py`:

```python
"""Divide out the INSTRUMENT, so we measure the world and not our own receiver.

THE HAZARD. Every receiver has a frequency-selective analog response -- the `Hdist` term.
PicoScenes measured **>15 dB of magnitude swing across subcarriers** on commodity NICs and stated
plainly that it *"causes a phantom object that interferes with the H_air measurement"*. It is NOT
removed by SpotFi-style linear-fit sanitisation, which only takes out the common linear slope.

**An uncalibrated receiver manufactures phantom taps out of its own filter**, and the phantom rate
is the number this whole paper reports. Left uncorrected we would be measuring our instrument and
calling it the world.

THE CORRECTION IS FREE. The LOS-only capture that validates the byte layout (Task 7) already
measures it. With the TX ~40 cm away and no reflector within 10 m, the channel is essentially one
path:

    H_los(f)  ~=  H_dist(f) * a * exp(-j 2 pi f tau_0)

Dividing a later capture by `H_los` removes the instrument response AND the direct path together,
leaving `1 + (echoes relative to the direct arrival)`. Its IFFT is a spike at bin 0 plus the echo
taps we actually want -- and those excess delays are, by construction, exactly what hw/truth.py
predicts.
"""
from __future__ import annotations
import numpy as np

# Below this fraction of the median magnitude a reference bin is a notch, not a measurement.
# Dividing by it would inject a spike -- a phantom of our own making.
_NOTCH_FLOOR = 1e-3


def reference_from_los(csi_los: np.ndarray) -> np.ndarray:
    """LOS-only capture -> the instrument+direct-path reference, averaged over frames.

    `csi_los` is (n_frames, n_sub) or a single (n_sub,) vector. Averaging beats the noise down;
    the reference is measured once per power cycle, so it is worth measuring well.
    """
    h = np.asarray(csi_los, dtype=complex)
    if h.ndim == 1:
        return h.copy()
    if h.size == 0:
        raise ValueError("empty LOS capture -- cannot build a reference")
    return h.mean(axis=0)


def apply_reference(csi: np.ndarray, ref: np.ndarray) -> np.ndarray:
    """Divide out the instrument (and the direct path). Returns the calibrated channel.

    Notch bins -- where the receiver's own response is near zero -- are set to 1 rather than
    divided, because dividing by ~0 injects a spike, which is precisely the kind of self-inflicted
    phantom this module exists to prevent.
    """
    h = np.asarray(csi, dtype=complex)
    r = np.asarray(ref, dtype=complex).ravel()
    if h.shape[-1] != r.size:
        raise ValueError(f"csi has {h.shape[-1]} subcarriers but the reference has {r.size}")
    mag = np.abs(r)
    floor = _NOTCH_FLOOR * max(float(np.median(mag)), 1e-30)
    safe = np.where(mag > floor, r, 1.0)
    out = h / safe
    return np.where(mag > floor, out, 1.0)      # notches contribute nothing, not infinity
```

- [ ] **Step 4: Run the tests and watch them pass**

```bash
.venv/bin/python -m pytest tests/test_hw_calib.py -q
```

Expected: 6 passed.

**If `test_an_UNCALIBRATED_instrument_MANUFACTURES_phantom_taps` ever stops failing to detect
anything, do NOT relax it.** That test is the reason this module exists: it proves the hazard is
real *before* we go near hardware.

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/hw/calib.py tests/test_hw_calib.py
git commit -m "paper4(hw): divide out the INSTRUMENT, or measure its phantoms instead of the world's

Every receiver has a frequency-selective analog response (Hdist). PicoScenes measured >15 dB of
swing across subcarriers on commodity NICs and stated it 'causes a phantom object that interferes
with the H_air measurement' -- and that it is NOT removed by SpotFi-style linear-fit sanitisation.

An UNCALIBRATED receiver manufactures phantom taps out of its own filter. Since the phantom rate is
the number this entire paper reports, shipping that uncorrected would mean measuring our instrument
and calling it the world. This is the same class of self-inflicted artifact that had to be hunted
out of paper 3 three times over.

The correction is FREE: the LOS-only capture that validates the byte layout already measures it.
Dividing by it removes the instrument response AND the direct path at once, leaving exactly the
echoes relative to the direct arrival -- which is precisely the quantity hw/truth.py predicts.

A test proves the hazard is real before we touch hardware: a ONE-PATH channel pushed through a
frequency-selective receiver sprouts taps that CFAR dutifully detects."
```

---

### Task 3: The ground truth — predicted excess path lengths

**Files:**
- Create: `src/wifi_radar_slam/hw/truth.py`
- Test: `tests/test_hw_truth.py`

**Interfaces:**
- Consumes: nothing (pure NumPy).
- Produces:
  - `monostatic_excess(pose_xy, tx_xy, reflectors) -> np.ndarray` — `(n_reflectors,)` excess path lengths (m).
  - `bistatic_excess(pose_xy, ap_xy, reflectors) -> np.ndarray` — same, for the ambient-AP control.

Both return the **excess over the direct TX→RX path** — the *identical* quantity `hw/delay.py`
measures, so `eval/phantom.py` compares like with like.

- [ ] **Step 1: Write the failing test**

Create `tests/test_hw_truth.py`:

```python
import numpy as np
import pytest
from wifi_radar_slam.hw.truth import monostatic_excess, bistatic_excess


def test_monostatic_excess_is_the_round_trip_minus_the_baseline():
    # The vehicle carries BOTH radios: TX at (0,0), RX 0.4 m away at (0.4,0). A reflector 20 m
    # ahead gives |TX->R| + |R->RX| - |TX->RX|.
    rx = np.array([0.4, 0.0])
    tx = np.array([0.0, 0.0])
    refl = np.array([[20.0, 0.0]])
    got = monostatic_excess(rx, tx, refl)
    expected = (20.0 + 19.6) - 0.4          # |TX->R| + |R->RX| - baseline
    assert got[0] == pytest.approx(expected)


def test_monostatic_excess_approaches_TWICE_the_range_as_the_baseline_shrinks():
    # THE reason the transmitter belongs on the vehicle. With the foci collapsed, the excess is
    # 2*range -- so a round trip traverses the distance TWICE, and the effective range
    # resolution is c/2B = 3.75 m rather than the bistatic c/B = 7.5 m. Twice as forgiving.
    rx = np.array([0.0, 0.0])
    tx = np.array([1e-6, 0.0])              # effectively co-located
    got = monostatic_excess(rx, tx, np.array([[25.0, 0.0]]))
    assert got[0] == pytest.approx(50.0, abs=1e-3)


def test_bistatic_excess_uses_the_AP_as_the_far_focus():
    # The infrastructure-bound control: an access point ALREADY in the building at (0,30).
    # Vehicle at the origin; reflector 40 m ahead.
    rx = np.array([0.0, 0.0])
    ap = np.array([0.0, 30.0])
    refl = np.array([[40.0, 0.0]])
    got = bistatic_excess(rx, ap, refl)
    expected = (np.hypot(40.0, 30.0) + 40.0) - 30.0
    assert got[0] == pytest.approx(expected)


def test_a_reflector_ON_the_direct_line_has_almost_no_excess():
    # A reflector lying between the AP and the vehicle barely detours, so its excess is ~0 and
    # it is UNRESOLVABLE from the direct path. This is the geometric reason the site must be
    # big and the reflectors well OFF-AXIS.
    rx = np.array([0.0, 0.0])
    ap = np.array([0.0, 30.0])
    got = bistatic_excess(rx, ap, np.array([[0.0, 15.0]]))
    assert got[0] == pytest.approx(0.0, abs=1e-9)


def test_many_reflectors_at_once():
    rx = np.array([0.0, 0.0])
    tx = np.array([0.4, 0.0])
    refl = np.array([[10.0, 0.0], [0.0, 20.0], [-15.0, 5.0]])
    got = monostatic_excess(rx, tx, refl)
    assert got.shape == (3,)
    assert np.all(got > 0)


def test_no_reflectors_gives_an_empty_array():
    got = monostatic_excess(np.zeros(2), np.array([0.4, 0.0]), np.empty((0, 2)))
    assert got.shape == (0,)
```

- [ ] **Step 2: Run the test and watch it fail**

```bash
.venv/bin/python -m pytest tests/test_hw_truth.py -q
```

Expected: `ModuleNotFoundError: No module named 'wifi_radar_slam.hw.truth'`.

- [ ] **Step 3: Implement**

Create `src/wifi_radar_slam/hw/truth.py`:

```python
"""Predicted EXCESS path lengths, from the surveyed scene + the measured pose.

This is the ground truth the phantom rate is measured against, and it is deliberately trivial:
a tape measure and Pythagoras. In a SURVEYED scene the true reflector positions are numbers we
wrote down, which is BETTER ground truth than anything a LiDAR would infer -- and it is why the
$99 LiDAR was dropped from the bill of materials.

Both functions return the excess over the DIRECT transmitter->receiver path, which is the
identical quantity hw/delay.py measures. eval/phantom.py therefore compares like with like, and
the resulting phantom rate is directly comparable to paper 2's ~89 % and paper 3's numbers.
"""
from __future__ import annotations
import numpy as np


def _excess(rx_xy, tx_xy, reflectors) -> np.ndarray:
    rx = np.asarray(rx_xy, dtype=float).reshape(2)
    tx = np.asarray(tx_xy, dtype=float).reshape(2)
    R = np.asarray(reflectors, dtype=float).reshape(-1, 2)
    if R.size == 0:
        return np.empty(0)
    direct = float(np.linalg.norm(tx - rx))
    return (np.linalg.norm(R - tx, axis=1) + np.linalg.norm(R - rx, axis=1)) - direct


def monostatic_excess(pose_xy, tx_xy, reflectors) -> np.ndarray:
    """The vehicle carries BOTH radios -- the self-contained sensor.

    Excess = |TX->R| + |R->RX| - |TX->RX|. As the baseline |TX->RX| shrinks the two foci
    collapse and the excess tends to **2 x range** -- a round trip traverses the distance twice.
    That is why the effective range resolution is c/2B = 3.75 m rather than the bistatic
    c/B = 7.5 m: the monostatic geometry is **twice as forgiving**, and it needs no
    infrastructure at all.
    """
    return _excess(pose_xy, tx_xy, reflectors)


def bistatic_excess(pose_xy, ap_xy, reflectors) -> np.ndarray:
    """The infrastructure-bound CONTROL: an access point that already exists in the building.

    Excess = |AP->R| + |R->RX| - |AP->RX|. This is papers 1-2's ambient premise, and it requires
    the AP's position to be KNOWN -- which is exactly why ambient WiFi "SLAM" is not really SLAM.
    We measure it only as the control against which the vehicle-mounted transmitter is compared.

    Note the geometric trap this makes explicit: a reflector lying ON the AP-vehicle line barely
    detours, so its excess is ~0 and it is UNRESOLVABLE from the direct path. Reflectors must be
    well OFF-AXIS, and the site must be big.
    """
    return _excess(pose_xy, ap_xy, reflectors)
```

- [ ] **Step 4: Run the tests and watch them pass**

```bash
.venv/bin/python -m pytest tests/test_hw_truth.py -q
```

Expected: 6 passed.

- [ ] **Step 5: Full suite, then commit**

```bash
.venv/bin/python -m pytest -q
```

Expected: 200 prior + 23 new = **223 passed**.

```bash
git add src/wifi_radar_slam/hw/truth.py tests/test_hw_truth.py
git commit -m "paper4(hw): predicted excess path lengths from the surveyed scene

Deliberately trivial -- a tape measure and Pythagoras. In a SURVEYED scene the true reflector
positions are numbers we wrote down, which is BETTER ground truth than anything a LiDAR could
infer, and it is why the \$99 LiDAR left the bill of materials.

Both geometries return the excess over the DIRECT path -- the identical quantity hw/delay.py
measures -- so eval/phantom.py compares like with like and the phantom rate stays directly
comparable to paper 2's ~89%."
```

---

### Task 4: The ESP32 firmware (two tiny programs)

**Files:**
- Create: `firmware/esp32_csi_rx/main/main.c`, `firmware/esp32_csi_rx/main/CMakeLists.txt`, `firmware/esp32_csi_rx/CMakeLists.txt`
- Create: `firmware/esp32_tx_beacon/main/main.c`, `firmware/esp32_tx_beacon/main/CMakeLists.txt`, `firmware/esp32_tx_beacon/CMakeLists.txt`

**No tests** — this is C on a microcontroller, and it is validated by Task 7 against a real
capture, which is the only test that means anything.

- [ ] **Step 1: Write the RX firmware**

Create `firmware/esp32_csi_rx/main/main.c`:

```c
// ESP32 CSI receiver -- the vehicle's radar receiver.
//
// It does ONE thing: enable HT40 CSI and print every packet's raw CSI over serial. All
// processing happens offline in Python (hw/csi.py, hw/delay.py). Phase 2 will port the winning
// chain on-chip; this is deliberately not that.
//
// The output line format is parsed by hw/csi.py::parse_csi_csv:
//     CSI_DATA,<timestamp_us>,<rssi>,[b0 b1 b2 ...]
// where the bytes are SIGNED int8 pairs. The pair order (imag,real vs real,imag) is a
// HYPOTHESIS settled by experiments/validate_esp32_csi.py -- see hw/csi.py.
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

static void csi_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf) return;
    printf("CSI_DATA,%lld,%d,[", (long long) esp_timer_get_time(), info->rx_ctrl.rssi);
    for (int i = 0; i < info->len; i++) {
        printf("%d ", info->buf[i]);            // SIGNED int8
    }
    printf("]\n");
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // HT40 is what gives 40 MHz of bandwidth -> 7.5 m path-length resolution (c/B), i.e. a
    // 3.75 m monostatic range resolution (c/2B). HT20 would HALVE it and the experiment would
    // resolve nothing. This line is load-bearing.
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40));
    ESP_ERROR_CHECK(esp_wifi_set_channel(6, WIFI_SECOND_CHAN_ABOVE));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    wifi_csi_config_t csi = {
        .lltf_en = true,
        .htltf_en = true,
        .stbc_htltf2_en = true,
        .ltf_merge_en = true,
        .channel_filter_en = false,   // keep every subcarrier -- we want the full delay span
        .manu_scale = false,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));

    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
```

Create `firmware/esp32_csi_rx/main/CMakeLists.txt`:

```cmake
idf_component_register(SRCS "main.c" REQUIRES esp_wifi nvs_flash esp_timer esp_netif esp_event)
```

Create `firmware/esp32_csi_rx/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_csi_rx)
```

- [ ] **Step 2: Write the TX illuminator firmware**

Create `firmware/esp32_tx_beacon/main/main.c`:

```c
// ESP32 illuminator -- the vehicle's radar transmitter.
//
// It rides on the SAME vehicle as the receiver, 30-50 cm away. That is the entire trick: two
// separate radios, so there is no full-duplex self-interference problem, yet the two bistatic
// foci nearly collapse and the geometry becomes MONOSTATIC-IN-EFFECT.
//
// It does ONE thing: transmit packets at a steady rate so the receiver has something to measure
// the channel with. Nothing is installed in the building. This is what makes the sensor
// self-contained -- a SLAM sensor, not a positioning service.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#define TX_PERIOD_MS 10          // 100 Hz -- plenty for a vehicle at walking pace

// A minimal 802.11 action frame. Content is irrelevant: we measure the CHANNEL it travels
// through, not the bits it carries.
static uint8_t frame[] = {
    0xd0, 0x00, 0x00, 0x00,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,          // dest: broadcast
    0x02, 0x00, 0x00, 0x00, 0x00, 0x01,          // src  (locally administered)
    0x02, 0x00, 0x00, 0x00, 0x00, 0x01,          // bssid
    0x00, 0x00,
};

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // MUST match the receiver exactly -- same channel, same bandwidth.
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40));
    ESP_ERROR_CHECK(esp_wifi_set_channel(6, WIFI_SECOND_CHAN_ABOVE));

    while (1) {
        esp_wifi_80211_tx(WIFI_IF_STA, frame, sizeof(frame), false);
        vTaskDelay(pdMS_TO_TICKS(TX_PERIOD_MS));
    }
}
```

Create `firmware/esp32_tx_beacon/main/CMakeLists.txt`:

```cmake
idf_component_register(SRCS "main.c" REQUIRES esp_wifi nvs_flash esp_netif esp_event)
```

Create `firmware/esp32_tx_beacon/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_tx_beacon)
```

- [ ] **Step 3: Commit**

```bash
git add firmware/
git commit -m "paper4(fw): the two ESP32 firmwares -- illuminator and CSI receiver

Both ride on the SAME vehicle, 30-50 cm apart. Two SEPARATE radios, so there is no full-duplex
self-interference problem -- yet the bistatic foci nearly collapse and the geometry becomes
monostatic-in-effect. Nothing is installed in the building: this is a SLAM sensor, not a
positioning service.

The RX firmware does ONE thing: enable HT40 CSI and stream it. HT40 is load-bearing -- HT20 would
halve the bandwidth to 20 MHz and the experiment would resolve nothing."
```

---

### Task 5: The measurement (the GATE)

**Files:**
- Create: `experiments/run_hw_phantom.py`

**Interfaces:**
- Consumes: `hw.csi.parse_csi_csv`, `hw.delay.{cfar_excess_lengths, music_excess_lengths}`,
  `hw.truth.{monostatic_excess, bistatic_excess}`, `eval.phantom.phantom_stats_frames`
  (signature: `phantom_stats_frames(det_range_m, det_azimuth_rad, true_range_m, true_azimuth_rad, range_scale_m=3.0, ...) -> dict`).

**On azimuth.** A single ESP32 has one RF chain, so there is **no bearing**. We pass **zeros for
every azimuth**, on both the detection and the truth side. The cost term then depends on range
alone, which is exactly what we want: `phantom_stats_frames` degrades cleanly to a **delay-only**
phantom rate, using the *identical* definition and tolerance as papers 2–3. **Do not invent a
separate 1-D scorer** — that would fork the definition and destroy comparability.

- [ ] **Step 1: Write the measurement script**

Create `experiments/run_hw_phantom.py`:

```python
"""THE MEASUREMENT -- and the GATE for the whole hardware programme.

Measures the PHANTOM RATE on a real channel: the thing paper 2 named as the single most valuable
missing measurement in this entire programme.

Two comparisons, both single-variable:

    FRONT-END   MUSIC  vs  CFAR        on the IDENTICAL CSI   -> paper 3's M->A axis
    GEOMETRY    bistatic vs monostatic on the IDENTICAL drive -> paper 3's A->B axis, THE headline

The scene is SURVEYED and the poses are MEASURED. There is no estimator anywhere -- exactly as
paper 3 scores its ablation under ground-truth poses.

    .venv/bin/python experiments/run_hw_phantom.py data/hw/<run>/
"""
from __future__ import annotations
import json
import os
import sys

import numpy as np

from wifi_radar_slam.hw.csi import parse_csi_csv, ESP32_HT40_BANDWIDTH_HZ
from wifi_radar_slam.hw.calib import reference_from_los, apply_reference
from wifi_radar_slam.hw.delay import cfar_excess_lengths, music_excess_lengths
from wifi_radar_slam.hw.truth import monostatic_excess, bistatic_excess
from wifi_radar_slam.eval.phantom import phantom_stats_frames

C = 299792458.0


def load_scene(path: str) -> dict:
    """scene.json -- the SURVEYED truth. Written by hand with a tape measure.

    {
      "reflectors":  [[x, y], ...],     # surveyed reflector positions (m)
      "tx_offset":   [0.4, 0.0],        # the on-vehicle illuminator, RELATIVE to the receiver
      "ap_xy":       [0.0, 30.0],       # an access point ALREADY in the building (the control)
      "poses":       [[x, y], ...]      # MEASURED receiver position, one per CSI frame
    }
    """
    with open(path) as f:
        s = json.load(f)
    for k in ("reflectors", "tx_offset", "poses"):
        if k not in s:
            raise SystemExit(f"scene.json is missing {k!r}")
    return s


def main() -> None:
    root = sys.argv[1] if len(sys.argv) > 1 else "data/hw/run1"
    scene = load_scene(os.path.join(root, "scene.json"))
    with open(os.path.join(root, "csi.csv")) as f:
        _, csi = parse_csi_csv(f.read())

    # DIVIDE OUT THE INSTRUMENT. Without this we measure the receiver's own frequency-selective
    # response -- which manufactures phantom taps -- and report them as the world's.
    los_path = os.path.join(root, "los_only.csv")
    if not os.path.exists(los_path):
        raise SystemExit(f"missing {los_path}: capture a LOS-only run first (see Task 7). "
                         f"Without it the phantom rate measures OUR RECEIVER, not the channel.")
    with open(los_path) as f:
        _, csi_los = parse_csi_csv(f.read())
    ref = reference_from_los(csi_los)
    csi = apply_reference(csi, ref)
    print(f"instrument response divided out (reference from {len(csi_los)} LOS-only frames)")

    poses = np.asarray(scene["poses"], dtype=float)
    refl = np.asarray(scene["reflectors"], dtype=float)
    tx_off = np.asarray(scene["tx_offset"], dtype=float)
    ap_xy = np.asarray(scene["ap_xy"], dtype=float) if "ap_xy" in scene else None

    n = min(len(csi), len(poses))
    if n == 0:
        raise SystemExit("no CSI frames matched to poses")
    print(f"{n} frames · {len(refl)} surveyed reflectors · "
          f"path-length resolution c/B = {C/ESP32_HT40_BANDWIDTH_HZ:.2f} m")

    B = ESP32_HT40_BANDWIDTH_HZ
    rows = []
    for geom in ("monostatic", "bistatic"):
        if geom == "bistatic" and ap_xy is None:
            continue
        for fe in ("cfar", "music"):
            det, tru = [], []
            for i in range(n):
                h = csi[i]
                d = (cfar_excess_lengths(h, B) if fe == "cfar"
                     else music_excess_lengths(h, B, n_paths=3))
                t = (monostatic_excess(poses[i], poses[i] + tx_off, refl) if geom == "monostatic"
                     else bistatic_excess(poses[i], ap_xy, refl))
                det.append(np.asarray(d).ravel())
                tru.append(np.asarray(t).ravel())

            # Azimuth is ZERO everywhere: one RF chain means no bearing. The cost then depends
            # on range alone, so phantom_stats_frames degrades cleanly to a DELAY-ONLY phantom
            # rate -- with the IDENTICAL definition and tolerance as papers 2-3.
            zeros = [np.zeros_like(x) for x in det]
            zeros_t = [np.zeros_like(x) for x in tru]
            s = phantom_stats_frames(det, zeros, tru, zeros_t, range_scale_m=3.0)
            s.update(geometry=geom, front_end=fe)
            rows.append(s)
            print(f"  {geom:11s} {fe:5s}: phantom {100*s['phantom_rate']:5.1f}%  "
                  f"n_det={s['n_detections']:5d}  bias={s['range_bias_m']:+.2f} m")

    os.makedirs("results", exist_ok=True)
    with open("results/hw_phantom.json", "w") as f:
        json.dump(rows, f, indent=2)
    print("\nsaved -> results/hw_phantom.json")

    def get(g, fe):
        return next((r for r in rows if r["geometry"] == g and r["front_end"] == fe), None)

    print("\n" + "=" * 66)
    m, c = get("monostatic", "music"), get("monostatic", "cfar")
    if m and c:
        print(f"FRONT-END axis (paper 3 predicts CFAR << MUSIC):")
        print(f"  MUSIC {100*m['phantom_rate']:.1f}%   ->   CFAR {100*c['phantom_rate']:.1f}%")
    b = get("bistatic", "cfar")
    if b and c:
        print(f"GEOMETRY axis -- THE HEADLINE (paper 3 predicts a collapse):")
        print(f"  bistatic {100*b['phantom_rate']:.1f}%   ->   monostatic "
              f"{100*c['phantom_rate']:.1f}%")
    print("=" * 66)
    print("NOTE: a HIGH phantom rate is NOT a null result -- the research predicts real hardware")
    print("      will REPRODUCE the ceiling. The discriminating measurements are the DIFFERENCES.")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Commit**

```bash
git add experiments/run_hw_phantom.py
git commit -m "paper4: the real-CSI phantom measurement -- the GATE

Two single-variable comparisons on the SAME data:
  FRONT-END  MUSIC vs CFAR        on identical CSI   -> paper 3's M->A axis
  GEOMETRY   bistatic vs monostatic on identical drive -> paper 3's A->B axis, THE headline

Scored with eval.phantom.phantom_stats_frames VERBATIM. With one RF chain there is no bearing, so
azimuth is zero on both sides and the scorer degrades cleanly to a delay-only phantom rate --
using the IDENTICAL definition and tolerance as papers 2-3. A separate 1-D scorer would fork the
definition and destroy the comparability that is the entire point."
```

---

### Task 6: The build document

**Files:**
- Create: `docs/hardware-build.md`

- [ ] **Step 1: Write it**

It must contain, with no placeholders:

1. **Bill of materials** — 2× ESP32 (~$8 ea), 2WD chassis + driver + battery (~$20). **Total ~$40.**
   State explicitly that **no LiDAR, no Raspberry Pi and no Intel NIC are required**, and why each
   was dropped (see the spec).
2. **Wiring / mounting** — the two ESP32s on the vehicle, **30–50 cm apart**, antennas separated
   and ideally cross-polarised (the direct path is the timing reference *and* the thing swamping
   the echoes).
3. **Flashing** — `idf.py -p <port> flash monitor` for each firmware; both must be on **channel 6,
   HT40, second channel ABOVE**. Mismatched channel or bandwidth ⇒ no CSI.
4. **The survey procedure** — tape-measure the reflector positions and the pose track into
   `scene.json` (schema in `experiments/run_hw_phantom.py::load_scene`). This is the ground truth;
   it must be written down, not estimated.
5. **🔴 THE SITE-SIZE REQUIREMENT** — its own section, not a footnote:
   > At 40 MHz the path-length resolution is **c/B = 7.5 m**, i.e. a **3.75 m monostatic range**
   > resolution. **Reflectors must sit beyond ~3.75 m, and well off-axis.** A small lab collapses
   > every echo into the direct-path bin and measures **nothing**. Use a corridor, sports hall, or
   > car park.
6. **What each measurement can and cannot prove** — copied from the spec's levels of truth. In
   particular: **no AoA, ever, from one RF chain**, and **a high phantom rate is not a null
   result.**

- [ ] **Step 2: Commit**

```bash
git add docs/hardware-build.md
git commit -m "paper4(docs): the build -- BOM, wiring, survey procedure, and the site-size rule"
```

---

### Task 7: Validate the CSI layout against real hardware — THE GATE

**Files:**
- Create: `experiments/validate_esp32_csi.py`

Two things cannot be settled without a device, and **both would silently corrupt every number**:

1. **The int8 pair order.** If it is (real, imag) rather than the assumed (imag, real), the channel
   is **conjugated** and the delay profile is **mirrored** — which looks like a physics result, not
   a bug.
2. **Whether HT40 actually yields 128 subcarriers** in the buffer, and how the LLTF / HT-LTF /
   STBC-HT-LTF fields are laid out.

**The decisive test costs nothing:** put the TX ~40 cm from the RX **in an open space with no
reflector within 10 m**. The channel is then essentially **one path**. Therefore:

- the delay profile must show **exactly one dominant peak, at bin 0**;
- **CFAR must report ZERO echoes**;
- **MUSIC (n_paths=3) must report peaks anyway** — and that is the paper's mechanism, observed on
  real hardware in the very first experiment.

If instead the profile is **symmetric or mirrored**, the pair order is wrong. Flip `imag_first`
and re-run. **Fix it in `hw/csi.py` — never compensate downstream.**

- [ ] **Step 1: Write the validation script**

Create `experiments/validate_esp32_csi.py`:

```python
"""Validate the ESP32 CSI byte layout against a REAL capture. The gate for sub-project 1.

Two things cannot be checked without a device, and each would silently corrupt every number:

  1. THE int8 PAIR ORDER. If it is (real, imag) rather than the assumed (imag, real), the channel
     is CONJUGATED and the delay profile is MIRRORED -- which looks like a physics result rather
     than a parsing bug.
  2. Whether HT40 really yields 128 subcarriers, and how the LLTF/HT-LTF fields are laid out.

THE DECISIVE TEST IS FREE. Put the TX ~40 cm from the RX in an open space with NO reflector within
10 m. The channel is then essentially ONE path, so:

    * the delay profile must show ONE dominant peak, at bin 0
    * CFAR must report ZERO echoes
    * MUSIC (n_paths=3) must report peaks ANYWAY -- the paper's mechanism, on real hardware,
      in the very first experiment

    .venv/bin/python experiments/validate_esp32_csi.py data/hw/los_only/csi.csv
"""
from __future__ import annotations
import sys

import numpy as np

from wifi_radar_slam.hw.csi import parse_csi_csv, ESP32_HT40_BANDWIDTH_HZ
from wifi_radar_slam.hw.delay import (delay_profile, path_length_bins,
                                      cfar_excess_lengths, music_excess_lengths)

B = ESP32_HT40_BANDWIDTH_HZ


def main() -> None:
    path = sys.argv[1] if len(sys.argv) > 1 else "data/hw/los_only/csi.csv"
    with open(path) as f:
        text = f.read()

    for imag_first in (True, False):
        ts, csi = parse_csi_csv(text, imag_first=imag_first)
        if csi.size == 0:
            raise SystemExit(f"no CSI rows parsed from {path}")
        n_sub = csi.shape[1]
        prof = np.mean([delay_profile(h) for h in csi], axis=0)
        bins = path_length_bins(n_sub, B)
        peak = int(np.argmax(prof))
        half = n_sub // 2
        # a MIRRORED profile puts as much energy in the upper half as the lower
        mirror_ratio = float(prof[half:].sum() / max(prof[1:half].sum(), 1e-12))

        print(f"\n=== imag_first={imag_first} ===")
        print(f"  subcarriers per row : {n_sub}")
        print(f"  peak bin            : {peak}  (must be 0 for a LOS-only channel)")
        print(f"  upper/lower energy  : {mirror_ratio:.2f}  (>>1 suggests a MIRRORED profile)")
        print(f"  path-length res c/B : {bins[1]:.2f} m")

        cf = cfar_excess_lengths(np.mean(csi, axis=0), B)
        mu = music_excess_lengths(np.mean(csi, axis=0), B, n_paths=3)
        print(f"  CFAR echoes         : {len(cf)}  (must be 0 -- there is nothing to see)")
        print(f"  MUSIC peaks         : {len(mu)}  (will be 3 -- it INVENTS them)")
        if len(cf) == 0 and len(mu) > 0:
            print("  ^^ THE PAPER'S MECHANISM, ON REAL HARDWARE: the calibrated detector")
            print("     correctly reports nothing while MUSIC conjures peaks from one path.")

    print("\n" + "=" * 70)
    print("VERDICT: choose the imag_first setting whose peak bin is 0 and whose upper/lower")
    print("energy ratio is LOW. If both look mirrored, the layout is not int8 pairs at all --")
    print("STOP and re-derive it. Fix it in hw/csi.py; NEVER compensate downstream.")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Capture a LOS-only run and execute the gate**

Flash both ESP32s, place them ~40 cm apart in the open (**no reflector within 10 m**), and capture:

```bash
mkdir -p data/hw/los_only
# adjust the port; 921600 baud
python3 -m serial.tools.miniterm /dev/ttyUSB0 921600 > data/hw/los_only/csi.csv
# ...let it run ~30 s, then Ctrl-]
.venv/bin/python experiments/validate_esp32_csi.py data/hw/los_only/csi.csv
```

**Act on what it prints:**

| observation | meaning | action |
|---|---|---|
| peak at bin 0, low mirror ratio | layout correct | record `imag_first`; proceed |
| peak at bin 0 only for `imag_first=False` | pairs are (real, imag) | **change the default in `hw/csi.py`** and add a test pinning it |
| mirrored under **both** settings | not int8 pairs at all | **STOP.** Re-derive the layout from a hex dump before trusting any number. |
| CFAR reports echoes on a LOS-only channel | the detector is manufacturing phantoms | **STOP.** The phantom rate would be uninterpretable. |
| MUSIC reports 3 peaks | **expected — this is the mechanism** | record it; it is the paper's first real-hardware evidence |

- [ ] **Step 3: Record the outcome and commit**

Create `docs/results-paper4-csi-validation.md` with the REAL printed values: the subcarrier count,
the settled `imag_first`, the peak bin, the mirror ratio, and the CFAR-vs-MUSIC counts on a
LOS-only channel. **None of these are re-derivable without the hardware, and every later result
depends on them.**

```bash
.venv/bin/python -m pytest -q
git add experiments/validate_esp32_csi.py docs/results-paper4-csi-validation.md src/wifi_radar_slam/hw/csi.py
git commit -m "paper4(hw): validate the ESP32 CSI layout on real hardware -- the gate

Settles empirically what the docs cannot: the int8 pair order (a wrong order CONJUGATES the
channel and MIRRORS the delay profile, which looks like a physics result rather than a parsing
bug) and the true subcarrier count under HT40.

The decisive test is free: TX 40 cm from RX with no reflector within 10 m is a one-path channel.
CFAR must report ZERO echoes; MUSIC (n_paths=3) will report peaks anyway. That is the paper's
mechanism, observed on real hardware in the very first experiment."
git push
```

---

## Definition of done

- [ ] `hw/csi.py` — the ESP32 stream parsed, byte layout **validated on real hardware**, not assumed.
- [ ] `hw/delay.py` — CSI → delay profile → excess path lengths, with **both** front-ends returning
      the **same quantity**, and the MUSIC-invents-peaks mechanism pinned by a test.
- [ ] `hw/calib.py` — the **instrument's own frequency response divided out**, so the phantom rate
      measures the world and not our receiver.
- [ ] `hw/truth.py` — predicted excess from the **surveyed** scene and **measured** pose.
- [ ] Two ESP32 firmwares: illuminator + CSI receiver, both HT40 on channel 6.
- [ ] `experiments/run_hw_phantom.py` — the phantom rate on real CSI, scored with
      `eval/phantom.py` **verbatim**.
- [ ] `docs/hardware-build.md` — BOM, wiring, survey, and the **site-size rule**.
- [ ] Full suite green (229 expected).

**Sub-project 2 is the GATE.** If we cannot measure a phantom rate on real CSI at all, the geometry
experiment has nothing to compare against and the programme stops there.

---

## Self-review of this plan

**Spec coverage.** The spec's sub-projects 1 and 2 ask for: the vehicle-mounted TX+RX rig, a
surveyed scene, measured poses, streamed CSI in both geometries, and the real-CSI phantom
measurement reusing `eval/phantom.py`. Task 4 → the firmwares; Tasks 1–3 → the offline pipeline;
Task 5 → the measurement; Task 6 → the build doc; Task 7 → the hardware gate. The spec's
**no-infrastructure** rule is Global Constraint 1 and shapes `hw/truth.py` (the AP is only ever a
*control*). The **site-size** rule appears in the physics section, in `hw/truth.py`'s docstring
(the off-axis trap), and as its own section of the build doc.

**Phase 2 (on-chip autonomy) is deliberately NOT in this plan** — it is spec sub-project 5, and it
cannot start until Tasks 1–7 have named the winning front-end. Porting an algorithm we have not yet
chosen would be the definition of premature.

**The one thing I have front-loaded** is the CSI byte layout. Sub-project 1 of paper 3 was burned
three times by assumed conventions (the Sionna angle convention, the `paths.a` layout, the
transmitter count), and a conjugated channel here would mirror the delay profile in a way that
*looks like a result*. So `imag_first` is a parameter, the default is labelled a hypothesis, and
Task 7 settles it against a real capture before any number is trusted.

**Type consistency.** `parse_csi_csv(text, imag_first) -> (ts, csi)` is defined in Task 1 and called
that way in Tasks 5 and 7. `cfar_excess_lengths(csi, bandwidth_hz, ...)` and
`music_excess_lengths(csi, bandwidth_hz, n_paths, max_path_m)` are defined in Task 2 and called with
those signatures in Tasks 5 and 7. `monostatic_excess(pose_xy, tx_xy, reflectors)` /
`bistatic_excess(pose_xy, ap_xy, reflectors)` are defined in Task 3 and called that way in Task 5.
`phantom_stats_frames(det_range_m, det_azimuth_rad, true_range_m, true_azimuth_rad, range_scale_m=…)`
is the existing signature on `paper3-sub3-ablation`, verified, and is called with it in Task 5.
`estimate_delays(block, bandwidth_hz, n_paths, max_range_m=None)` is the existing signature in
`sensing/superres.py`, verified, and is called with it in Task 2.
