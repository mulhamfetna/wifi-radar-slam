# Paper 4 — design: a physical WiFi-CSI testbed that replicates the ablation

**Date:** 2026-07-13
**Status:** approved (brainstorming), pending spec review
**Branch:** `paper4-hardware-testbed` (off `paper3-wifi-vs-radar`)
**Working title:** *Does the Phantom Ceiling Survive Contact with a Real Channel? A $60 ESP32 Test
of Simulated WiFi Sensing*

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

## The design insight: coherence is a receiver problem, not a transmitter problem

> **Multiple *receivers* must be phase-coherent (hard, expensive). Multiple *transmitters* need no
> coherence whatsoever (free).**

Each transmitter yields an independent bistatic **ellipse** (from its excess delay, with the AP and
the vehicle at the foci). **Three transmitters → three ellipses → intersect → the reflector is
located with no angle-of-arrival estimate at all.**

This is the single most important architectural decision here. It means the cheapest tier can do
real geometry — not merely log signal strength — with $8 chips and no synchronisation.

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
| 4 antennas **AND** ≥80 MHz on commodity hardware | **DOES NOT EXIST.** You choose aperture *or* bandwidth. | survey of the above |

### ⚠ Explicitly NOT verified — do not fill these in by guessing

- **SDR price table** (USRP B210/X310, bladeRF, LimeSDR, KrakenSDR frequency coverage).
- **PicoScenes licence cost and terms** (site returned 404/403).
- **ESPARGOS price / purchasability** (site returned 403).
- **Whether `nexmon_csi` captures all four bcm4366c0 RX cores coherently per packet.**
- A fetched claim that bcm43455c0 works on **Raspberry Pi 5** — **treat as false until checked.**
- Commercial 4-element 5 GHz ULA products and prices.
- **Whether an ESPARGOS-class coherent ESP32 array can run HT40 (40 MHz)** rather than the 20 MHz
  reported. If it can, the "array costs you bandwidth" trade-off above weakens.

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
| RPLIDAR A1M8 ($99) | ground-truth *map* of an unknown room | papers 1–2's own `controlled_wall` scene is **one reflector at a known position**. We **survey the scene with a tape measure**. A written-down number is *better* ground truth than a LiDAR-derived one, not worse. |
| Raspberry Pi 4 ($60) | logging | the ESP32 streams CSI over WiFi/serial to a laptop you already own |
| Intel 5300 ($100) | AoA | **three TX beacons give three ellipses; their intersection locates the reflector with no AoA at all** |

### Stage 0 — the FRONT-END axis (~$60, ESP32 only)

| item | qty | ~USD |
|---|---|---|
| **ESP32-DevKitC — CSI receiver** (on the car) | 1 | 8 |
| **ESP32 — TX beacons**, at surveyed positions | 3 | 24 |
| 2WD robot chassis + driver + battery | 1 | 20 |
| *(optional)* ESP32-CAM + ArUco tags — pose ground truth | 1 | 8 |

**Physics:** 2.4 GHz, HT40 → **40 MHz** → bistatic path-length resolution **c/B = 7.5 m**.

**Why delay-only still tests the mechanism.** Paper 2's ~89 % arises from MUSIC's **fixed model
order**: asked for 3 paths it emits 3 peaks *whether or not 3 resolvable paths exist*. That
pathology lives on the **delay axis alone**. No AoA is required to expose it.

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

### Stage 1 — the GEOMETRY axis. The headline. (+$8, one more ESP32)

**The entire change: unplug one beacon from the wall and bolt it to the car**, 30–50 cm from the
receiver. The two ellipse foci nearly collapse and the geometry becomes **monostatic-in-effect**.

Two *separate* radios, so there is **no full-duplex self-interference problem** — we sidestep the
~110 dB monster that makes real monostatic WiFi radar expensive.

**Claim it tests:** paper 3 predicts phantoms collapse **18 % → ~0 %**. This is our novel finding,
and it costs **ten dollars**.

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
- **Inter-antenna phase calibration IS required** for AoA (Stage 2+). *(Exact procedure and
  references: NOT yet verified — see the gaps list. Do not invent one.)*

---

## 🔴 THE make-or-break parameter: the scene must be BIG

At 40 MHz the path-length resolution is **7.5 m**. A reflection is separable from the direct path
only if its **excess** path length exceeds that.

For a reflector offset *r* from the AP–receiver line at separation *d*, the excess is
approximately **2r²/d**. With an AP 10 m away, clearing 7.5 m of excess needs **r ≈ 6 m**.

> **A small lab collapses every echo into the LOS bin and measures nothing.** The site must be a
> corridor, sports hall, or car park, with large surveyed reflectors (metal sheets) placed WELL
> OFF-AXIS. This is geometry, not budget, and it decides whether the experiment works at all.

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
- No change to papers 1–3 **except** the paper-2 `c/B` correction above, which is mandatory.

---

## Decomposition into sub-projects

| # | Sub-project | Deliverable |
|---|---|---|
| **1** | **Stage 0 rig + CSI capture** | Robot, ESP32 RX + 3 ESP32 TX beacons, **a SURVEYED scene** (no LiDAR), synchronised logging. Deliverable: a dataset of (CSI, ground-truth pose, surveyed map). |
| **2** | **The real-CSI phantom measurement** | Port `eval/phantom.py` to real data: predict true path lengths from the LiDAR map, extract taps, measure the phantom rate. **MUSIC vs CFAR.** |
| **3** | **The geometry experiment (Stage 1)** | Move the TX onto the car. Re-measure. **This is the headline.** |
| **4** | **AoA (Stage 2) + bandwidth (Stage 3)** | Intel 5300, then AX210. Only if 1–3 succeed. |

**Sub-project 2 is the gate.** If we cannot measure a phantom rate on real CSI at all, Stage 1's
geometry experiment has nothing to compare against and the programme stops there.

---

## Acceptance

- A committed dataset: real CSI + LiDAR ground-truth pose and map, on a moving robot.
- A **phantom rate measured on real CSI**, with the same definition as `eval/phantom.py`.
- **MUSIC vs CFAR** on identical real CSI — the front-end axis, physically.
- **Bistatic vs monostatic** on identical real CSI — the geometry axis, physically. *The headline.*
- Every claim traceable to a committed artifact; the unverified list emptied or the claims dropped.
