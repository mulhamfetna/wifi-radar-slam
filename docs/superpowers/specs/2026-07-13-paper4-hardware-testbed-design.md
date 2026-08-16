> # ⛔ SUPERSEDED — DO NOT FOLLOW THIS DOCUMENT
>
> **Withdrawn 2026-07-14.** It describes a **moving vehicle**, a **servo**, and a SLAM-adjacent framing.
> All three are wrong. Its differential-CSI method is **broken as written** (it subtracts raw complex CSI
> across recordings — see Part 5 of the replacement).
>
> **Replaced by: [`docs/paper4-restart-static-bench.md`](../../paper4-restart-static-bench.md)**
> (Arabic mirror: `docs/paper4-restart-static-bench.ar.md`)
>
> Kept only as a record of what was tried. The **research** in `docs/research-paper4-hardware.md`
> remains valid.

# Paper 4 — design: a physical WiFi-CSI testbed that replicates the ablation

**Date:** 2026-07-13
**Status:** approved (brainstorming), pending spec review
**Branch:** `paper4-hardware-testbed` (off `paper3-wifi-vs-radar`)
**Working title:** *A $40 Self-Contained WiFi Radar: Testing the Phantom Ceiling on a Real Channel*

---

## Why this exists

Everything in papers 1–3 is ray-traced. Paper 2 named the gap itself:

> *"The single most valuable missing measurement is whether the 89 % phantom rate survives contact
> with a real channel — it is currently a property of MUSIC operating on **simulated** CSI."*

Paper 3 then sharpened *what* to measure. Its ablation found the phantom ceiling is **not** a
property of WiFi and **not** a property of RF sensing. It is two things:

| mechanism | simulated effect |
|---|---|
| the **superresolution front-end** (MUSIC, fixed model order) | phantoms ~89 % → **18 %** when swapped for a calibrated detector |
| the **bistatic geometry** (ambient AP illuminator) | phantoms 18 % → **~0 %** when the transmitter moves onto the vehicle |
| the **carrier** (5.2 GHz → 77 GHz) | **no effect at all** |
| **bandwidth** (160 MHz → 4 GHz) | phantoms got **worse**, not better |

Each of those is a *falsifiable physical prediction*. This testbed exists to test them on a real
channel, cheaply.

---

## The finding that makes this affordable

**Paper 3 proved the carrier does nothing.** That result is what *licenses us to buy 2.4 GHz
hardware* even though the simulation ran at 5.2 GHz. We are not cutting a corner — we are spending
a result we earned. If a reviewer objects that the testbed is not at the simulated carrier, the
answer is cell B→C of our own ablation.

---

## 🔴 THE governing constraint: this is a SLAM sensor, not a positioning service

**An earlier draft of this spec proposed three surveyed transmitter beacons in the room. That was
wrong, and it is worth saying why, because the mistake is seductive.**

Three fixed, surveyed transmitters + a moving receiver is an **anchor network** — infrastructure-based
localisation. It is not SLAM. A SLAM sensor must be **self-contained**: bolt it to any vehicle, drop
it into any building, and it works, with nothing pre-installed and nothing surveyed. A LiDAR passes
that test. A radar passes it. An anchor mesh does not.

**Our own best result already satisfies this.** Paper 3's **cell B — monostatic, the transmitter ON
THE VEHICLE — needs no infrastructure whatsoever.** The vehicle illuminates the world and listens
to its own echoes, exactly as a radar does. And cell B is *also* the best cell we measured (0 %
phantoms; highest map IoU).

That hands the paper a far sharper argument than the one we started with:

> **Ambient WiFi SLAM is not really SLAM** — it needs the access points' positions, so it is
> infrastructure-bound. **Move the transmitter onto the vehicle and you get a genuinely
> self-contained sensor that is *also* dramatically better.** Infrastructure-free and
> higher-performing at the same time.

**Everything below is therefore vehicle-mounted. Nothing is installed in the environment.**

### Where bearing comes from, with no phase-coherent array

A single receive antenna gives **range only** — a circle of possible reflector positions, not a
point. Two cheap ways to get bearing, neither needing phase coherence:

1. **From motion (free).** Each pose yields a range circle. Circles from successive poses
   **intersect at real reflectors and fail to intersect for phantoms.** The vehicle's own movement
   is the baseline. This is exactly what our simulated SLAM does when it accumulates detections
   across frames.
2. **From a servo (~$5) + a directional antenna (~$8).** Mechanically sweep the beam and you get
   **range–azimuth scans — the exact data format of paper 3's radar chain** (range → azimuth →
   CFAR), and of the Navtech spinning radar we anchored against in sub-project 2. A poor man's
   spinning WiFi radar. **Mechanical scanning buys azimuth without any coherence at all.**

---

## What real hardware can and cannot give (verified, 2026-07-13)

| claim | verdict | source |
|---|---|---|
| ESP32 gives **40 MHz** CSI (HT40, 128 subcarriers) | **TRUE** — matches our `nominal.yaml` | ESP-IDF WiFi vendor features |
| ESP32 is **2.4 GHz only, one RF chain** | **TRUE** — "antenna diversity" is an RF *switch* (RTC6603SP), one antenna at a time | ESP-IDF PHY guide |
| ⇒ **a single ESP32 can never do AoA** | **TRUE** | follows from one RF chain |
| ESP32-C5 adds 5 GHz | true, **but its 5 GHz CSI returns static IQ that never changes** | esp-idf issue #18493 |
| **Multiple ESP32s cannot be phase-coherent** | **FALSE — do not claim this.** ESPARGOS built an **8-ESP32 coherent array doing MUSIC AoA**, using a daisy-chained 40 MHz reference **plus** a distributed phase-reference signal (a shared clock alone is *not* enough — the PLL re-locks with random phase). Capped at 2.4 GHz / 20 MHz. | arXiv 2502.09405 |
| **Absolute time-of-flight** from commodity CSI | **IMPOSSIBLE.** Packet-detection delay is *"an order of magnitude larger than time of flight."* | Tsinghua CSI tutorial, arXiv 2206.09532 |
| **Relative / excess delay** (w.r.t. the LOS arrival) | **MEASURABLE.** STO/SFO enter as a phase ramp **common to all paths within a packet**, so delays *relative to LOS* survive. | ibid. |
| ⇒ **our bistatic ellipse method works on a $5 chip** | **TRUE** — the ellipse needs exactly the *excess* path length, not the absolute one | follows |
| Cheapest **phase-coherent multi-antenna** CSI | **Intel 5300**, 3 antennas, 5 GHz, 40 MHz, ~$60–125 used — but needs an **ancient kernel (3.2–4.2)** | Linux 802.11n CSI Tool |
| Widest-bandwidth commodity CSI | **Intel AX210 + PicoScenes**, **160 MHz**, card ~$30 — but **only 2 RX chains** | Vetco / PicoScenes |
| 4 antennas **AND** ≥80 MHz on commodity hardware | ✅ **IT EXISTS — my "does not exist" claim was FALSE, now RETRACTED. [V]** **WiROS + ASUS RT-AC86U**: *"for the Asus hardware, there are **4 receive antennas**"*, **20/40/80 MHz** (80 MHz → 256 subcarriers), median bearing error **5.3°** post-calibration vs **115°** without. ~$70–180 (discontinued; the "~$110" is **[P]** — in no paper). **Coherent out of the box is FALSE; coherent after their published calibration is TRUE** — and it must be re-run on every **channel change and device reset**. | WiROS arXiv:2305.13418 (UCSD/Bharadia); nexmon_csi |

### ❌ RETRACTED — verified FALSE, never publish

> ~~"With cheap WiFi you must choose **either** aperture **or** bandwidth — not both."~~ **FALSE** (see
> the row above). **The TRUE and stronger claim:** *cheap WiFi cannot buy **range resolution** at **any**
> price, because the ceiling is the **802.11 standard**, not the budget.* 160 MHz is the widest channel
> permitted below 6 GHz; a 77 GHz FMCW radar sweeps **1–4 GHz**. The real trade is **WiFi vs radar** —
> a **25–100× bandwidth gap that money cannot close**. Full argument: `docs/research-paper4-hardware.md` §0.

### ⚠ Explicitly NOT verified — do not fill these in by guessing

- **SDR price table** (USRP B210/X310, bladeRF, LimeSDR).
  *(KrakenSDR is **REFUTED [V]**: tunes 24–1766 MHz — it **cannot reach 2.4 GHz** — at 2.56 MHz/channel.)*
- **ESPARGOS price / purchasability** — hardware is **not open source**; no public price. **[U]**
  *(But ESPARGOS itself is **[V] REAL**: 8×ESP32, measured MUSIC spectra. Caveat: a shared crystal gives
  frequency lock, **NOT phase lock** — the PLL re-randomises its phase on every reset and channel change.
  Their fix is a custom RF PCB with a wired phase-reference network. **Multi-ESP32 AoA is a board
  bring-up project, not a weekend.**)*
- A fetched claim that bcm43455c0 works on **Raspberry Pi 5** — **treat as false until checked.**
- Commercial 4-element 5 GHz ULA products and prices.
- **Whether an ESPARGOS-class coherent ESP32 array can run HT40 (40 MHz)** rather than the 20 MHz
  reported. If it can, the "array costs you bandwidth" trade-off above weakens.
- **Intel AX200/AX210 per-chain phase offset is UNCHARACTERISED in the literature** — no paper
  validates AoA on it. So Stage 3 (160 MHz) buys bandwidth into *unknown* AoA territory.

*(CLOSED since the first draft: the inter-antenna phase-calibration procedure — see the antenna
section. It is a per-boot constant, and ArrayTrack's cable-swap trick removes it along with the
cable imbalance.)*

---

## 🔴 A correction to paper 2, found by this research — fix BEFORE submission

Paper 2 states, in three places, that its **6.45 m median range bias** is *"far beyond the
**0.94 m** resolution limit"* at 160 MHz (`main.tex:72`, `main.tex:521`, `DOSSIER.md:292`).

**That is the wrong resolution figure for the quantity being measured.**

- MUSIC in our code reports `delay * C` — a **bistatic PATH LENGTH** (`sensing/frontend.py:33,41`),
  which is then fed to `_triangulate_bistatic(..., path_len, ...)`.
- The Rayleigh resolution of a *path-length* estimate at bandwidth *B* is **c/B**.
- At 160 MHz: **c/B = 1.87 m.** The quoted 0.94 m is **c/2B** — the *monostatic range* resolution,
  which is the resolution of a **different quantity** (range = path length / 2).

| | value | ratio to the 6.45 m bias |
|---|---|---|
| quoted limit (c/2B) — **wrong quantity** | 0.94 m | 6.9× |
| correct limit (c/B) | **1.87 m** | **3.4×** |

**The conclusion survives**: 6.45 m is still 3.4× the true resolution limit, so it remains a
**bias**, not a resolution bound. Only the number is wrong. **Paper 2 is held and unsubmitted, so
this is a clean pre-submission fix, not an erratum** — which is precisely what the hold was for.

**Action:** correct the three occurrences to 1.87 m, re-cut the submit-from tag. *(Paper 1 is
unaffected: its 60 GHz/aperture null result does not rest on this figure. To be re-checked when
paper 1's revision is prepared.)*

---

## The staged rig — ESP32-ONLY for both headline experiments

**Revised 2026-07-13.** Both experiments that matter run on **ESP32 boards alone, for ~$60.** The
LiDAR, the Raspberry Pi and the Intel NIC were each carrying an assumption that does not survive
scrutiny:

| dropped | why it was there | why it goes |
|---|---|---|
| RPLIDAR ($69–99) | ground-truth *map* of an unknown room | papers 1–2's own `controlled_wall` scene is **one reflector at a known position**. We **survey the scene with a tape measure**. A written-down number is *better* ground truth than a LiDAR-derived one, not worse. |
| Raspberry Pi 4 ($60) | logging | the ESP32 streams CSI over WiFi/serial to a laptop you already own |
| Intel 5300 ($100) | AoA | **three TX beacons give three ellipses; their intersection locates the reflector with no AoA at all** |

### Stage 0 — the self-contained sensor (~$40, ESP32 only, NOTHING in the environment)

| item | qty | ~USD |
|---|---|---|
| **ESP32 — illuminator** (transmits), on the vehicle | 1 | 8 |
| **ESP32 — CSI receiver**, on the vehicle, 30–50 cm away | 1 | 8 |
| 2WD chassis + motor driver + battery | 1 | 20 |
| *(offline)* a laptop you already own — runs MUSIC/CFAR | — | 0 |

**Nothing is installed in the building. Nothing is surveyed except the reflectors we are scoring
against** (the ground truth, exactly as papers 1–2's `controlled_wall` scene is a single reflector
at a known position).

**Physics:** 2.4 GHz, HT40 → **40 MHz**. Monostatic-in-effect (the TX/RX foci nearly collapse), so
the observable is a **round-trip range** with resolution **c/2B = 3.75 m** — *twice as good as the
bistatic 7.5 m*, and one more reason the geometry flip is the right move.

**No on-board compute is needed.** The ESP32 only **streams CSI**; MUSIC and CFAR run offline on a
laptop. So "do we need a more powerful processor?" — **no**, not on the vehicle.

**Why delay-only still tests the mechanism.** Paper 2's ~89 % arises from MUSIC's **fixed model
order**: asked for 3 paths it emits 3 peaks *whether or not 3 resolvable paths exist*. That
pathology lives on the **delay axis alone**. No AoA is required to expose it.

### Stage 0b — mechanical azimuth (+~$13): a poor man's spinning WiFi radar

Add a **servo (~$5)** and a **directional antenna (~$8, patch or Yagi)** to the receiver. Sweep it.

You now produce **range–azimuth scans** — the *identical data format* to paper 3's radar chain and
to the Navtech spinning radar of the credibility anchor. The **same code** (`radar/processing.py`:
range → azimuth → CFAR) processes it.

Beamwidth is coarse (a small 2.4 GHz patch is ~60–80°; a Yagi ~30–40°), so azimuth resolution is
poor — but it is **real bearing, obtained with zero phase coherence**.

**How a phantom is measured without AoA.** The LiDAR gives ground-truth pose *and* a 2-D map. From
that map plus the known beacon positions we **predict the true path lengths** at every pose. The CSI
gives the measured excess-delay taps. **A tap that matches no predicted path is a phantom** — the
same definition as `eval/phantom.py`, evaluated on the delay axis alone.

**Claims it can support:**
- ✅ the **delay-only phantom rate on a real channel** — the measurement paper 2 called for
- ✅ **MUSIC vs CFAR on the identical CSI** → paper 3's **front-end** axis (M→A)
- ✅ reflector localisation by **3-ellipse intersection** (no AoA)
- ❌ **no AoA** → cannot reproduce papers 1–2's *(delay, AoA)* detection exactly
- ❌ 7.5 m resolution is coarse relative to indoor features

### Stage 1 — the GEOMETRY axis. The headline. ($0 extra — it is a LOGGING mode, not hardware)

The monostatic rig above is already the *good* configuration. The **comparison** — bistatic, the
infrastructure-bound one — costs **nothing extra**: on the same drive, also log CSI from **an access
point that already exists in the building** (its position measured once). That is precisely papers
1–2's ambient premise.

So one drive, one vehicle, **two geometries**:

| | illuminator | infrastructure needed | paper-3 cell |
|---|---|---|---|
| **bistatic** | an existing building AP | **YES — the AP's position must be known** | A |
| **monostatic** | the ESP32 on the vehicle | **NONE** | B |

**Claim it tests:** paper 3 predicts phantoms collapse **18 % → ~0 %** across that flip — while the
monostatic side simultaneously **removes the infrastructure dependence that stops ambient WiFi SLAM
from being SLAM at all.** Two arguments, one experiment, zero extra hardware.

Two *separate* radios (TX and RX), so there is **no full-duplex self-interference problem** — we
sidestep the ~110 dB monster that makes real monostatic WiFi radar expensive.

**Costs, stated up front, not discovered later:**
- **~3.75 m blind range**: the direct TX→RX path occupies the first resolution cell.
- **A real dynamic-range fight**: the direct path we *need* as the timing reference is the same
  signal swamping the echoes. Mitigations: antenna isolation, cross-polarisation, RX attenuation.

### Stage 2 — the AoA axis (OPTIONAL, +~$100) — and why an ESP32 array is the WRONG way to get it

AoA is the one thing ESP32s cannot give cheaply, and it is **not needed for either headline
experiment**. If we want it:

**Intel 5300** (3 phase-coherent antennas, 5 GHz, 40 MHz) in a used mini-PCIe laptop, ~$60–125.
Antennas: three 5 GHz dipoles at **λ/2 = 2.9 cm** on a bar (~$10) plus inter-antenna phase
calibration. It gives full **(delay, AoA) MUSIC — papers 1–2's exact front-end** — so it measures
the **89 % claim directly**. Risk: it needs a kernel from 2012–2015; the driver work, not the $60
card, is the real cost.

**The ESP32-only route to AoA is a TRAP.** An ESPARGOS-style coherent array is ESP32-only, but it is
capped at **20 MHz** — i.e. **15 m** path-length resolution, against **7.5 m** for a single HT40
ESP32.

> **A coherent ESP32 array buys angle by paying with half the bandwidth — and bandwidth is what
> resolves the delays the phantom rate is *defined on*. For this experiment, ONE ESP32 at 40 MHz
> beats FOUR at 20 MHz.**

*(Whether an ESPARGOS-class array could run HT40 is NOT verified — added to the gaps list.)*

### Stage 3 — the BANDWIDTH axis (+~$30)

**Intel AX210 + PicoScenes**: **160 MHz** (→ 1.87 m path-length resolution) but **only 2 RX chains**.

**Claim it tests:** paper 3 predicts more bandwidth does **not** fix phantoms — it made them *worse*
(0 % → 9 %). A sharp, falsifiable prediction.

### Stage 4 — research grade (deferred, unpriced)

ESPARGOS-class coherent ESP32 array, or a coherent multi-channel SDR. **Prices unverified**; not
planned until Stages 0–3 are done.

---

## Antennas — the honest answer

- **Stages 0–1 need NO array.** One RF chain means an array is *physically meaningless*. A $2 dipole
  (or the ESP32's own PCB antenna) is correct, not a compromise.
- **The only array we ever build** is Stage 2's 3-element ULA of cheap dipoles at λ/2. At 5 GHz,
  λ/2 = **2.9 cm**, so the whole aperture is under 6 cm.
- **Directional antennas (patch, Yagi) buy gain and a narrower field of view — we need neither.**
  Skip them.
- **Inter-antenna phase calibration IS required** for AoA (Stage 2+) — **now verified, gap closed:**

  **The folklore is wrong, in a way that helps.** The per-antenna phase offset is **not random per
  packet**. It is a **per-boot** (Intel/Atheros) or **per-retune** (Broadcom) constant drawn from a
  small discrete set. What *is* random per packet — CFO, packet-detection delay, SFO — is **common
  to all antennas and cancels in the inter-antenna difference** (Zubow et al., arXiv:2005.03755,
  verbatim). **One calibration per power cycle suffices.**

  **The procedure** (ArrayTrack, NSDI'13, Eqs. 9–12): splitter + reference tone into all channels,
  with the **cable-swap-and-average trick** — measure the offsets, physically swap the cables at the
  splitter, measure again, average. This kills the **LO offset *and* the cable imbalance at once**,
  so **matched cables stop mattering**. Every serious DF platform does some version of this
  (KrakenSDR: internal noise source + RF switches; Ettus: *"re-align the LOs after each tune
  command"*; ADI Phaser: boresight tone → `phase_cal_val.pkl`). **None skips it.**

  **Why it is not optional:** at 5.2 GHz, **1 mm of cable mismatch ≈ 9° of phase ≈ 3° of bearing
  error**, and 10° of residual per-element phase yields a **3.2° bias** — which dominates the noise
  floor (CRLB ~1.8° for a 4-element ULA at 10 dB SNR) by **10–100×** on an uncalibrated array.

  **⚠ TWO TRAPS, both flagged:**
  1. **"SpotFi needs no calibration" is a trap.** That claim is about the *environment*, not the
     array. SpotFi's Algorithm 1 deliberately removes only the **common-mode** slope — fitting one
     slope jointly across all antennas — so it **preserves** (and therefore does **not** remove) the
     per-chain PLL offset.
  2. **PicoScenes' `Hdist`** — a per-chain, **frequency-selective** distortion (>15 dB swing across
     subcarriers) which, in their own words, ***"causes a phantom object."*** **Not** removed by
     linear-fit sanitisation. **This one bites us even at Stage 0 with a single antenna**, and is
     why the plan divides out the instrument response (`hw/calib.py`) before reporting any number.

---

## Phase 2 — the vehicle becomes TOTALLY self-contained (no laptop)

The laptop in Phase 1 is a **development tool, not a dependency**. You cannot tune an algorithm you
cannot see, and Phase 1 must compare MUSIC against CFAR and score both against ground truth. Once
the winner is known, it is ported to the ESP32 and the laptop is cut loose.

**And our own result hands us the cheap algorithm.** Paper 3 found CFAR beats MUSIC (89 % → 18 %
phantoms). Computationally:

- **MUSIC** requires an eigendecomposition — **O(N³)**.
- **CFAR** is a sliding average and a threshold — **O(N)**.

> **The scientifically better front-end is also the computationally cheaper one.** That is not luck:
> MUSIC's cost *is* its pathology — it forces a fixed model order onto data that does not support it,
> and pays O(N³) for the privilege of inventing peaks.

### The on-board compute budget (128 subcarriers, 64 range bins, 36 azimuths/sweep, 240 MHz)

| stage | flops per sweep |
|---|---|
| 128-pt FFT × 36 azimuths | 161,280 |
| CA-CFAR × 36 | 18,432 |
| polar → Cartesian projection | 13,824 |
| **total** | **193,536** |

- **Sustained sweep rate: ~1,240 Hz** at one flop/cycle (pessimistic — ESP32-S3 has SIMD and
  `esp-dsp` ships an optimised FFT). **We need 1–5 Hz.** ~250× headroom.
- **MUSIC, for contrast:** 11.8 Mflop/sweep → ~20 Hz. Still feasible, but **61× more expensive —
  and worse.**
- Scan-to-map ICP (≈50 scan points vs ≈500 map points, 30 iterations) ≈ **15 ms** → a few Hz.
  Comfortable.

### Memory

| | |
|---|---|
| CSI frame (128 complex float32) | 1.0 KB |
| range–azimuth map (36 × 64) | 9.0 KB |
| accumulated point map (2,000 pts) | 15.6 KB |
| **total** | **≈ 26 KB** |

**ESP32 SRAM is 520 KB; the ESP32-S3 adds up to 8 MB PSRAM.** The whole sensor fits in ~5 % of the
base chip's RAM.

**Conclusion: compute was never the obstacle.** A fully autonomous $40 sensor — CSI in, map out, no
host — is a port, not a research problem. Use an **ESP32-S3** for the on-board build (SIMD + PSRAM);
the plain ESP32 suffices for Phase 1 streaming.

---

## Scope: we are validating the READING, not building a SLAM system

**We do not estimate poses and we do not build a map.** That is deliberate, and it is exactly what
paper 3 already does (it scores every cell **under ground-truth poses**, with no estimator in the
loop — see `docs/results-paper3-anchor.md` for why that is the *stronger* experiment).

| | how |
|---|---|
| **poses** | **measured** — tape/rail/wheel encoders. Not estimated. |
| **map** | **surveyed** — the reflectors' true positions. Not built. |
| **outputs** | phantom rate · range bias · MUSIC vs CFAR · bistatic vs monostatic |

If the reading is wrong, no SLAM back-end can save it. If the reading is right, the SLAM back-end is
already written (it is `lidar/slam_icp.py`, shared across all three papers). **Get the reading
right; the rest exists.**

---

## 🔴 THE make-or-break parameter: the scene must be BIG

At 40 MHz the **monostatic round-trip range resolution is c/2B = 3.75 m**, and the **bistatic
path-length resolution is c/B = 7.5 m**. An echo is separable from the direct TX→RX path only if it
falls outside the first resolution cell.

- **Monostatic (the good case):** a reflector must be **beyond ~3.75 m**. Easy — that is most of a
  corridor or car park.
- **Bistatic (the control):** the **excess** path must exceed 7.5 m. For a reflector offset *r*
  from the AP–receiver line at separation *d*, excess ≈ **2r²/d**; with the AP 10 m away that needs
  **r ≈ 6 m**.

> **A small lab collapses every echo into the direct-path bin and measures nothing.** The site must
> be a corridor, sports hall, or car park, with large surveyed reflectors (metal sheets) placed well
> off-axis. This is geometry, not budget, and it decides whether the experiment works at all.

Note the asymmetry: **the monostatic configuration is twice as forgiving** (3.75 m vs 7.5 m),
because a round trip traverses the distance twice. That is a *second*, independent reason the
transmitter belongs on the vehicle.

The research also predicts real hardware will **reproduce** the phantom ceiling rather than refute
it, and that this 7.5 m coarseness may be its *physical origin*.

**That is a confirmation, not a failure** — but it has two consequences we accept in advance:
1. **Run in a corridor or car park, not a small lab.** Path lengths must span many resolution cells.
2. **A high phantom rate at Stage 0 is not a null result.** The *discriminating* measurements are
   the **differences**: MUSIC vs CFAR (Stage 0), and bistatic vs monostatic (Stage 1). A ceiling that
   fails to move when the geometry changes would falsify paper 3; one that collapses confirms it.

---

## Honesty guards

- **Do not claim multiple ESP32s cannot be phase-coherent.** ESPARGOS did it. The true statement is
  narrower: *coherence requires a shared clock **and** a distributed phase reference, and no such
  system reaches our simulated bandwidth.*
- **Do not claim absolute ToF from commodity CSI.** It is not measurable. Only the **excess** delay
  is — and that is all our method needs. Say so explicitly rather than letting a reviewer find it.
- **Report the 7.5 m resolution limit prominently**, not in a footnote. It bounds everything Stage 0
  can conclude.
- **The unverified list is part of the spec.** Anything on it must be verified before it enters a
  paper, not filled in from memory.

---

## Non-goals

- No 5 GHz at Stages 0–1 (the ESP32 cannot, and paper 3 proved the carrier does not matter).
- No true full-duplex monostatic radar (self-interference cancellation is out of budget and
  unnecessary — the co-located-TX trick achieves the geometry we care about).
- No outdoor vehicular driving. A small robot in a corridor/car park is sufficient and safe.
- **No anchor/tower/mesh network.** Any design that needs transmitters installed and surveyed in the
  environment is disqualified: it is positioning, not SLAM. The vehicle carries its own illuminator.
- **No pose estimation and no map building.** We validate the READING, under measured poses, exactly
  as paper 3 scores its ablation.
- No change to papers 1–3 **except** the paper-2 `c/B` correction above, which is mandatory.

---

## Decomposition into sub-projects

| # | Sub-project | Deliverable |
|---|---|---|
| **1** | **The self-contained sensor + CSI capture** | Vehicle carrying **TX + RX ESP32s** (nothing installed in the building), a **surveyed** reflector scene, measured poses, streamed CSI. Deliverable: a dataset of (CSI, measured pose, surveyed reflector map) in BOTH geometries — monostatic (own TX) and bistatic (an existing building AP). |
| **2** | **The real-CSI phantom measurement** ⟵ **GATE** | Port `eval/phantom.py` to real data: predict the true echo ranges from the surveyed scene + measured pose, extract taps, measure the phantom rate and range bias. **MUSIC vs CFAR** on identical CSI. |
| **3** | **The geometry experiment** | Compare monostatic vs bistatic on the SAME drive. **This is the headline** — and it is a *logging mode*, not new hardware. |
| **4** | **Mechanical azimuth (Stage 0b)** | Servo + directional antenna → range–azimuth scans, processed by the EXISTING `radar/processing.py`. Only if 1–3 succeed. |
| **5** | **On-board autonomy (Phase 2)** | Port the winning chain (FFT → CFAR → project → map) to an **ESP32-S3**. Budget: 194 kflop/sweep, 26 KB — ~250x headroom. **Cut the laptop loose.** Only after 1–3 have named the winner. |

**Sub-project 2 is the gate.** If we cannot measure a phantom rate on real CSI at all, the geometry
experiment has nothing to compare against and the programme stops there.

---

## Acceptance

- A committed dataset: real CSI + **measured** pose + **surveyed** reflector map, on a moving
  vehicle carrying **its own illuminator** — nothing installed in the environment.
- A **phantom rate measured on real CSI**, with the same definition as `eval/phantom.py`.
- **MUSIC vs CFAR** on identical real CSI — the front-end axis, physically.
- **Bistatic vs monostatic** on identical real CSI — the geometry axis, physically. *The headline.*
- Every claim traceable to a committed artifact; the unverified list emptied or the claims dropped.
