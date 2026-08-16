# Paper 4 — RESTART. The Static Bench.

**Date:** 2026-07-14
**Branch:** `paper4-hardware-testbed`
**Status:** **This document SUPERSEDES** `docs/superpowers/specs/2026-07-13-paper4-hardware-testbed-design.md`
and `docs/superpowers/plans/2026-07-13-paper4-sub1-esp32-sensor.md`. Those two described a *moving
vehicle* with a *servo* and a *SLAM-adjacent* framing. **Both are withdrawn.** The research in
`docs/research-paper4-hardware.md` remains valid and is the reference for everything below.

**Verification legend used throughout:**
**[V]** = verified, a source was fetched and it says this · **[P]** = partial · **[U]** = unverified,
**do not put in a paper** · **[D]** = derived (arithmetic from verified inputs, shown so it can be checked).

---

## PART 0 — Why we restarted, stated honestly

This document exists because the previous plan accumulated **layers of error**, and every layer had the
same shape: **an elaborate structure built on top of something that had never been tested.**

The actual record, without softening:

| # | The mistake | How it was caught | The lesson |
|---|---|---|---|
| 1 | Designed a **3-beacon anchor network** for a SLAM paper. | **The user caught it.** *"We are not installing towers."* | An anchor network is **positioning, not SLAM**. I designed the wrong thing entirely and did not notice. |
| 2 | Claimed **"with cheap WiFi you must choose aperture OR bandwidth"**. | A verification agent **refuted it** with a citation. | **It was simply false.** WiROS + ASUS RT-AC86U gives 4 coherent antennas **and** 80 MHz. Had it reached a paper, one reviewer would have killed it. |
| 3 | Paper 2 reported **0.94 m** WiFi range resolution. | Caught during paper 3 work. | It is a **bistatic path length** → resolution is **c/B = 1.87 m**, not **c/2B**. A **factor of two**, in five places. |
| 4 | **Pooled** every frame's true paths when computing the phantom rate. | Caught by a hang, not by a test. | It let a detection be "explained" by a reflector from **elsewhere on the trajectory** — **massively undercounting phantoms**, the paper's headline number. |
| 5 | Concluded **"k=40 is 11× better"** from a perfect-init ICP test. | Self-caught. | **Wrong.** A flatter cost makes ICP *move less*, so a method returning its input unchanged scores perfectly. **I was measuring stillness and calling it accuracy.** |
| 6 | The differential plan in this very document's first draft. | A verification agent, **before any hardware was bought**. | Subtracting raw complex CSI across two recordings is **meaningless** — see Part 5. It would have produced noise, and I would have concluded *"no echo exists."* |

**The pattern is not carelessness. It is sequencing.** In every case I built layer *N+1* before verifying
layer *N*. Mistake 6 is the proof that the fix works: it was caught **by testing the foundation first**,
and it cost nothing.

**The discipline this document enforces:**

> **One new thing per experiment. Every experiment has a written kill criterion, decided BEFORE the
> measurement. If a rung fails, we stop and report the failure — we do not climb past it.**

---

## PART 1 — Restudy: what the three papers ACTUALLY say

I re-read the three dossiers rather than trusting memory. Here is the distilled record.

### Paper 1 — *Ambient WiFi as a Radar Replacement for Automotive SLAM* (IEEE IoT-J, SUBMITTED)

- **Localization:** centimetre-level. Joint 2-D (delay–angle) MUSIC gives ATE **0.098 ± 0.028 m**
  (5 seeds), against an oracle of **0.049 ± 0.027 m**. It **approaches** but does **not match** the oracle.
- **⚠️ ERRATUM (self-reported to the editor, 2026-07-12):** the submitted manuscript claimed **0.027 m**.
  Re-running paper 1's **own frozen code** with its **own committed config** gives 0.098 ± 0.028. The
  reported 0.027 is **below the minimum of every single seed** — this is not run-to-run variance. The
  submitted paper also labels the row **40 MHz** while the committed config is **160 MHz**.
- **Mapping:** ~**25–30 cm** with oracle sensing; ~**4–5 m** with realistic sensing.
- **Paper 1's explanation of the mapping floor:** it is **not** a bandwidth/aperture limit (proved by a
  60 GHz + 16-antenna test that changed nothing), but a **path-discrimination** limit — and it claimed
  discrimination is **learnable** (random forest, F1 ≈ 0.9).

### Paper 2 — *Can Ambient WiFi Replace LiDAR?* (COMPLETE, FROZEN, HELD)

**Paper 2 proved paper 1's explanation WRONG.** This is the single most important fact in the programme.

The mapping ceiling decomposes into **three** mechanisms, and paper 1 named the smallest one:

| mechanism | magnitude | |
|---|---|---|
| **1. Phantom detections** | **89.2 % (controlled) / 89.5 % (street)** | **DOMINANT.** These MUSIC detections match **no real propagation path at all.** They are estimator artefacts. |
| **2. Estimation bias** | **6.45 m** median range bias (controlled) | Ruins even *correctly identified* paths. True params 100 % within 1 m → MUSIC params **2.4 %**. This **dwarfs** the resolution limit — it is a **bias**, not a resolution bound. |
| **3. Path discrimination** | **2–8 %** | **Paper 1's mechanism. Real, but the SMALLEST of the three.** |

> **"You cannot discriminate among real paths when most detections are not real paths."**

And paper 1's inference — *"discrimination is learnable (F1 ≈ 0.9) ⇒ mapping is fixable"* — **does not
hold**, because (a) discrimination is the smallest mechanism, and (b) the F1 ≈ 0.9 used an **`elevation`**
feature that a **single-ULA 2-D front-end cannot measure**. On observable features only, **F1 is 0.00–0.45**.

**Crucially, paper 2 also showed the geometry IS recoverable** if phantoms and bias are fixed. **The
ceiling is set by the FRONT-END, not by the physics.**

### Paper 3 — *Is the Phantom Ceiling Universal?* (IN PROGRESS)

Two results, both load-bearing.

**(a) The credibility gate — FAILED, and the failure is architectural.**

Our point-to-point scan-to-map **ICP back-end cannot estimate rotation from radar point clouds at all.**
The yaw cost is **FLAT**:

| frame | true Δyaw | cost at truth | cost at its minimum | where the minimum actually is |
|---|---|---|---|---|
| 151 | +11.90° | 0.3933 | 0.3850 | **−6.0°** from truth |
| 150 | +11.69° | 0.3966 | 0.3886 | **+6.0°** |
| 149 | +11.20° | 0.3900 | 0.3839 | **+8.0°** |
| 147 | +10.93° | 0.3890 | 0.3789 | **−8.0°** |

The cost varies **~2 % over ±8°** and its minimum sits at **random** offsets. Correlation between the yaw
**error** and the **true** yaw change: **−0.992** — the estimator recovers **nothing**; it returns its
initial guess. **This was measured on REAL 77 GHz Boreas radar with a full 360° scan (400 azimuths ×
3371 range bins).** Six hypotheses were tested and killed; every point density from k=2 to k=40 is flat.

**(b) The ablation (scored under GROUND-TRUTH poses — because of (a)).**
Scene `controlled_wall`, 3 seeds:

| cell | front-end | geometry | carrier / BW | **phantom rate** | map IoU |
|---|---|---|---|---|---|
| **M** | MUSIC (papers 1–2) | bistatic | 5.2 GHz / 160 MHz | — | **0.003 ± 0.004** |
| **A** | CFAR | **bistatic** | 5.2 GHz / 160 MHz | **18.2 ± 0.6 %** | 0.392 |
| **B** | CFAR | **monostatic** | 5.2 GHz / 160 MHz | **0.1 ± 0.2 %** | **0.615** |
| **C** | CFAR | monostatic | **77 GHz** / 160 MHz | **0.0 %** | 0.572 |
| **D** | CFAR | monostatic | 77 GHz / **4 GHz** | **9.0 ± 1.5 %** | 0.599 |

**Read that table carefully. It is the most important table in the programme:**

- **B → C: the CARRIER DOES NOTHING.** 5.2 GHz and 77 GHz give the same answer. *(This is what licenses
  us to build at 2.4 GHz.)*
- **A → B: GEOMETRY IS EVERYTHING.** Moving the transmitter from *off-vehicle* (bistatic) to
  *on-vehicle* (monostatic) takes the phantom rate from **18.2 % to 0.1 %** — a **180× reduction.**
- **C → D: MORE BANDWIDTH MAKES PHANTOMS WORSE.** 4 GHz gives **9.0 %** where 160 MHz gave **0.0 %**.
  *(More bandwidth = more resolvable cells = more opportunities for a false alarm.)*
- **M: MUSIC reproduces paper 2's TOTAL mapping failure (IoU 0.003) even under perfect ground-truth
  poses.** So the ≈89 % ceiling is **the front-end plus the geometry** — it is **not** "WiFi", and it is
  **not** "RF sensing".

### 🔑 THE THROUGH-LINE OF ALL THREE PAPERS

Every result above reduces to **one scalar**:

> ## **THE PHANTOM RATE**
> **the fraction of detections that correspond to no real reflector**

MUSIC + bistatic → **≈89 %** → mapping is destroyed.
CFAR + monostatic → **≈0.1 %** → mapping works.

Localization, mapping, drift, IoU, cost, the LiDAR comparison, the radar comparison — **all of it is
downstream of that one number.** Nothing else in the programme is upstream of anything.

**Therefore: the hardware experiment has exactly one job — MEASURE THE PHANTOM RATE ON REAL SILICON.**
Not SLAM. Not a map. Not a trajectory. **One number.**

---

## PART 2 — Can we prove a self-contained SLAM system? **NO.** (Settled, with citations)

Verified against the literature by two independent research passes. Full record in
`docs/research-paper4-hardware.md` §0. Summarised here because it **bounds what this paper may claim**.

**The blocker was never infrastructure.** The no-anchors constraint **is satisfiable**. There are four
independent reasons a *SLAM* claim is unavailable, and **each one is fatal on its own**:

**Reason 1 — our own estimator cannot recover rotation.** *(§Paper 3(a) above.)* The yaw cost is flat on
**real 77 GHz radar with 400 azimuths.** An ESP32 has **one RF chain → ZERO azimuths.** If yaw is
unrecoverable from a full angular scan, it is not recoverable from a bearing-less delay profile. **[ours]**

**Reason 2 — range-only SLAM has never been shown with anonymous returns.** Every foundational result
assumes **labelled** landmarks **and** an odometry prior. **Blanco et al., ICRA'08, verbatim:**
> *"The only assumptions are the availability of **odometry** and a range sensor **able of identifying
> the different beacons**."*

Newman & Leonard (ICRA'03) used *interrogated* LBL transponders — **they reply, so they are addressable.**
Djugash et al. (ICRA'06) used radio beacons that range to *each other*. **A WALL HAS NO ID.** **[V]**

**Reason 3 — a wall is not a point landmark.** A monostatic echo from a flat specular surface has delay =
2 × the **perpendicular** distance, so **the apparent scattering centre SLIDES along the wall as the
vehicle moves.** The point-landmark model that every range-only SLAM paper is built on is violated **even
with perfect data association**. The only literature that ever solved this class of problem is 1990s
**sonar** (Leonard & Durrant-Whyte, *Directed Sonar Sensing*, 1992 — Regions of Constant Depth +
arc-intersection across many poses) — **and it needed odometry and many returns per surface.** **[V/P]**

**Reason 4 — the synthetic-aperture escape hatch is closed by two top-tier NEGATIVE results.**
Angle-from-motion needs trajectory knowledge to **λ/16 = 7.8 mm** at 2.4 GHz (two-way).
- **Ubicarse** (MobiCom'14), verbatim: SAR *"requires **sub-centimeter accuracy**… commercial motion
  sensors are **virtually unusable** to measure such fine-grained translation."*
- **Zhu et al.** (MobiCom'15): injected **5 mm** of trajectory error → imaging error grew **4× to 40 cm**;
  they then **abandoned coherent SAR** for a non-coherent RSS method.

Best-case *calibrated* wheel odometry (Borenstein & Feng, UMBmark) = 0.3–0.5 % of distance → **3–5 mm per
metre — exactly on the boundary, zero margin.** A slipping RC car: **2–5 cm — 3–6× over budget.** The
aperture **decoheres**. **[V]**

### What it would actually take — and it is a NUMBER, not a cleverness

Anchorless radar SLAM **does exist and does work**, on-platform, no beacons:

| work | sensor | **bandwidth** | result |
|---|---|---|---|
| arXiv:2311.14970 | 2 × IR-UWB (Novelda X4M300) on-board + wheel odometry | ~1.5 GHz | real 2-D **occupancy grid**, EKF-SLAM, loop closure |
| arXiv:2510.02874 | UWB SAR (ARIA LT102), odometry-driven back-projection | **2 GHz** | **9 %** cell-wise difference vs a LiDAR grid |

**The only thing they have that we do not is BANDWIDTH: 500 MHz – 2 GHz. We have 40 MHz.**

| platform | max B | path-length resolution (c/B) |
|---|---|---|
| **ESP32 (2.4 GHz, HT40)** | **40 MHz** | **7.5 m** |
| RT-AC86U / nexmon (5 GHz) | 80 MHz | 3.75 m |
| AX210 (5–6 GHz) | 160 MHz | 1.9 m |
| **77 GHz automotive FMCW** | **1–4 GHz** | **15 → 3.75 cm** |

**A 25–100× gap that MONEY CANNOT CLOSE**, because **160 MHz is the widest channel the standard permits
below 6 GHz.** The limit is the **regulation**, not the **wallet**. The honest trade is **WiFi vs radar** —
not cheap vs expensive.

### The closest prior art to our exact rig — and it stops exactly where we predict

**Daniels, Yeh & Heath**, *Forward Collision Vehicular Radar with IEEE 802.11* (arXiv:1702.03351):
transmitter **and** receiver **on the vehicle**, real over-the-air measurements, 802.11 OFDM, 20 MHz.
**Output: range to the closest target, meter-level. NO ANGLE. NO MAP. NO SLAM.** **[V]**
**Nobody has gone past that at sub-7 GHz.** That ceiling is precisely what we propose to **measure**.

### ⛔ The claim we must NEVER make
> Claiming **full self-contained WiFi SLAM on cheap hardware** puts us in **direct contradiction with two
> published negative results** (Ubicarse; Zhu et al.). **One citation kills the paper.**

### ✅ The claim the literature WILL defend
> **A self-contained, on-vehicle RF sensor** (TX + RX on the platform, **zero infrastructure**),
> **validated against measured ground truth**, with **detections evaluated at KNOWN poses** —
> **full SLAM NOT claimed.**

---

## PART 3 — Why STATIC is not a compromise. It is the correct experiment.

The user's instinct — *"start simple: static car"* — is not a concession. **It is a complete and valid
test of the headline claim of the entire programme.** Here is the argument, explicitly:

**The phantom rate is a property of a SINGLE POSE.** Papers 2 and 3 both compute it **frame by frame**, at
one position, comparing detections against the true propagation paths **at that pose**. *(In fact, one of
our bugs — mistake #4 in Part 0 — was caused by* **pooling** *frames, which is semantically wrong precisely
because the phantom rate is per-pose.)*

**Therefore the phantom rate needs NO motion, NO trajectory, NO map, and NO SLAM.** It needs exactly one
thing: **a known pose and a known set of reflectors.** A tape measure supplies both.

### What motion would ADD — i.e. what static DELETES

| motion brings | static removes it |
|---|---|
| **Pose estimation** | ✅ **and we PROVED it fails** (flat yaw cost, corr −0.992) |
| **Motion smearing** within a sweep | ✅ |
| **Aperture coherence** (needs 7.8 mm trajectory truth) | ✅ |
| **Odometry error** (2–5 cm on an RC car) | ✅ **ground truth becomes a TAPE MEASURE** |

**Static deletes all four sources of error, and loses nothing that the headline number depends on.**

### And the one thing static UNLOCKS

Angle — the single capability one ESP32 physically cannot provide (**one RF chain**) — can later be
obtained by **mechanically rotating a directional antenna on a servo**. This needs **ZERO phase
coherence** and produces range–azimuth scans in the **identical format** to our existing
`radar/processing.py`.

**A slow mechanical scan is ONLY legitimate because the platform is static.** Motion is what forbids
slow scanning. **The user's constraint is precisely what makes the cheapest possible angle solution
valid.** This is not a coincidence; it is the whole reason to start here.

### ⚠️ The one thing to REJECT from the original request

> *"measure distance around it **in all directions**"*

**"Directions" = ANGLE. Angle is the one thing a single ESP32 cannot do.** Starting there would be **layer
#1 of a brand-new stack of mistakes.** **We start with ONE direction and ONE reflector.** Angle is
**Rung 7**, and only if Rungs 1–6 pass.

---

## PART 4 — The physics. Every number, derived.

### 4.1 Bandwidth, resolution, and the delay grid

| quantity | value | source |
|---|---|---|
| Channel bandwidth (HT40) | **B = 40 MHz** | **[V]** ESP32 supports 20/40 MHz, **2.4 GHz only** |
| Subcarrier spacing | **Δf = 312.5 kHz** | **[V]** 802.11 OFDM |
| HT-LTF subcarriers reported | **128** (indices `0~63, -64~-1`) | **[V]** ESP-IDF `wifi.rst` CSI table |
| Non-null subcarriers, 40 MHz | **114** of 128 (−57…+57); usable 108; pilots at ±11, ±25, ±53 | **[V]** ESP-IDF PHY table |
| **Delay resolution** | **Δτ = 1/B = 25 ns** | **[D]** |
| **→ one resolution cell, in PATH length** | **c·Δτ = 7.5 m** | **[D]** *(independently confirmed: "for a 802.11ac 40 MHz channel, the path length resolution is 7.5 m")* |
| **→ one resolution cell, in monostatic RANGE** | **c·Δτ/2 = 3.75 m** | **[D]** |
| Delay-profile span (128-pt IFFT) | 128 × 25 ns = **3.2 µs = 960 m of path** | **[D]** — vastly more than we need |

### 4.2 The geometry of the atomic test

Two ESP32s, **baseline b = 0.5 m**, static. A flat metal plate at perpendicular distance **d**.

**Echo path length** = |TX→plate| + |plate→RX| ≈ **2d** (for d ≫ b).
**Excess path** over the direct line-of-sight = **2d − b**.
**Excess delay** τ = (2d − b)/c.

| plate at d | excess path (2d − 0.5) | excess delay | **in resolution cells** | verdict |
|---|---|---|---|---|
| 4 m | 7.5 m | 25 ns | **1.0** | ❌ **buried in the LOS main lobe** |
| 6 m | 11.5 m | 38 ns | **1.53** | ⚠️ sits in the LOS skirt |
| 8 m | 15.5 m | 52 ns | **2.07** | ⚠️ marginal |
| **10 m** | **19.5 m** | **65 ns** | **2.60** | ✅ |
| **12 m** | **23.5 m** | **78 ns** | **3.13** | ✅ **← START HERE** |
| **14 m** | **27.5 m** | **92 ns** | **3.67** | ✅ |

**→ The site requirement is a CORRIDOR OF 15–20 m.** Not a car park. A university hallway does it.

### 4.3 "Monostatic" is a MEASUREMENT here, not an assertion

Pretending TX and RX are co-located introduces a geometric error of **at most b = 0.5 m** in path length.
One resolution cell is **7.5 m** of path.

> **error / cell = 0.5 / 7.5 = 6.7 %.**
>
> **The 0.5 m baseline is INVISIBLE at 40 MHz.** The rig is monostatic **to within 6.7 % of one
> resolution cell.** This is a derived bound **[D]**, not hand-waving — and it is why paper 3's cell-B
> result (0.1 % phantoms, monostatic) is the one this rig physically implements.

### 4.4 ⚠️ RESOLUTION vs PRECISION — the distinction that caused mistake #3 and #5

These are **different physical quantities** and they require **different experiments**. Conflating them is
exactly the error class that produced three months of mistakes.

| | **RESOLUTION** | **PRECISION (accuracy)** |
|---|---|---|
| **Question it answers** | Can I **separate TWO** targets? | Can I **locate ONE isolated** target? |
| **Limited by** | **BANDWIDTH.** Hard cap. | **SNR.** Not bandwidth. |
| **Our value** | **3.75 m** (monostatic). **No escape.** | Interpolable to **a small fraction of a bin** |
| **Scaling** | c/2B | CRLB ∝ 1/(B·√SNR) |
| **Which rung tests it** | **Rung 3** | **Rung 2** |

**Consequence:** Rung 2 (does the tap track the plate's true distance?) **can be precise even at 40 MHz**,
because there is only **ONE** plate and a single peak can be interpolated. Rung 3 (can two plates be
separated?) is where the **3.75 m** limit bites, hard. **They must never be merged into one experiment.**

### 4.5 The dynamic-range problem — and why coherent averaging is MANDATORY

| quantity | value | source |
|---|---|---|
| ESP32 CSI sample format | **8-bit signed I and Q** | **[V]** 2 bytes/subcarrier, **imaginary first, then real** |
| **→ raw dynamic range** | **≈ 48 dB** | **[D]** (6 dB/bit × 8) |
| Echo strength below LOS, 0.5 m plate at 6 m | **≈ −30 dB** | **[P]** estimate |
| Echo strength below LOS, 0.5 m plate at 14 m | **≈ −46 dB** | **[P]** estimate |
| **→ echo amplitude at 14 m** | **≈ 0.6 LSB** | **[D]** ⚠️ **BELOW THE QUANTIZATION FLOOR** |
| Coherent averaging gain, N packets | **10·log₁₀(N)** dB | **[D]** |
| **→ gain from N = 1000 packets** | **+30 dB** | **[D]** |

> ### 🚨 **THE ECHO IS BELOW THE QUANTIZATION FLOOR OF A SINGLE PACKET.**
> **Coherent averaging over ~10³ packets is not an optimisation. It is the ONLY reason this experiment
> can work at all.** And coherent averaging is impossible without the referencing procedure in Part 5.
> **This is why Part 5 is the most important section of this document.**

---

## PART 5 — 🚨 THE HOLE IN MY OWN PLAN, and the fix

### What I originally wrote (WRONG)

> *"Record CSI with the plate present, record again with it removed, then **subtract the two complex CSI
> vectors**."*

### Why it fails **[V]**

Every packet carries:
- an **independent random common phase** (residual CFO + PLL state), and
- an **independent time offset** (packet-detection jitter / STO).

Across two *separate recordings* you additionally get a **PLL re-lock**. So `H_A` and `H_B` are each
defined only **up to an arbitrary complex scalar and an arbitrary time shift.**

> **Subtracting them coherently is MEANINGLESS. The result is noise.**
>
> **And I would have concluded: "there is no echo."** A null result, caused entirely by my own method,
> misdiagnosed as physics. **This is mistake #4 and #5 all over again — measuring the instrument and
> calling it the world.**

### ✅ The fix — the LoS-referencing procedure. **THE ORDER IS NOT OPTIONAL.**

**Per packet:**
1. **Remove the linear phase ramp** across subcarriers. This kills the time offset (STO) and aligns the
   time origin. *(Working implementation exists: `espargos.util.remove_mean_sto()`, which estimates the
   slope via `angle(Σ H[k+1]·conj(H[k]))` and de-ramps.)* **[V]**
2. **Divide the entire CSI vector by the complex value of the LOS tap** (self-reference to the first
   arriving path). **This kills the random common phase AND the amplitude scaling in one step.** **[V]**

**Then, per recording:**
3. **Coherently average over N ≈ 1000 packets** → **+30 dB** of processing gain. *(Only now is this
   legitimate — steps 1 and 2 are what make the packets coherent with each other.)*
4. **NOW subtract** the two averaged, referenced vectors.

**Do it in this order or the experiment does not work.**

### Why the physics permits this — the STO argument, verified three ways **[V]**

The claim: *STO is a **linear phase ramp** across subcarriers ⇒ a **pure time shift** of the whole CIR ⇒
the **excess** delay (tap position **relative to** the LOS tap) is **preserved**.*

**This is CORRECT.** Verified:

1. **"Hands-on Wireless Sensing with Wi-Fi: A Tutorial"** (arXiv:2206.09532) gives the error model
   `φ̃ₙ = φₙ + 2π(f_c + Δf_j + f_D)·ε_t`. The **`Δf_j·ε_t` term is LINEAR in subcarrier index** — i.e. a
   time shift `ε_t` of the CIR. Verbatim: *"the ε_t is eliminated, **at the cost of losing absolute ToF
   measurement**."* **Absolute delay dies; RELATIVE delay survives.** ← *This is the single fact the whole
   programme rests on.*
2. **Splicer** (MobiCom'15, §3): SFO *"can cause s[n] after ADC a **time shift** τ_o"*; packet-boundary
   detection error *"introduces another **time shift** τ_b"*. **Both are time shifts.**
3. **It is implemented and shipping** — `espargos.util.remove_mean_sto()`, called immediately before the
   IFFT in the working `azimuth-delay` demo, on real ESP32 hardware.

**CFO is constant across subcarriers within a packet — NOT a ramp. [V]** It **rotates the whole CIR in the
complex plane but MOVES NO TAP.** Our **|CIR|** is immune to it. *(Bonus: the ESP32 actually reports CFO —
pyespargos reverse-engineered `get_cfo_from_rx_ctrl()` and exposes it in Hz.)*

### 🚨 The NON-LINEAR error that DOES exist — and it IS the phantom generator **[V]**

- The Tutorial's Eq. 28 describes a **fixed hardware distortion**: an **"M-shaped amplitude-frequency
  characteristic"** and an **"S-shaped" phase curve** across subcarriers. **NOT linear in subcarrier
  index.** An **S-shaped phase = a non-constant group delay = it DOES smear and displace taps.**
- *"Same Signal, Different Story: Demystifying Receiver Effects in Wi-Fi CSI"* (arXiv:2605.26836):
  *"different Wi-Fi receivers introduce **distinct and systematic distortions** to the CSI measurements"* —
  subcarrier-dependent **non-linear** distortion from the baseband filters, **persisting after correcting
  global gain and linear phase.**
- **PicoScenes** reports the same effect and names it exactly: a >15 dB swing across subcarriers that
  ***"causes a PHANTOM OBJECT that interferes with the H_air measurement"*** — and it is **NOT** removed by
  SpotFi-style linear-fit sanitisation.

> ### **THIS IS OUR PHANTOM. The receiver MANUFACTURES phantom taps out of its own filter.**
> **Uncorrected, we would measure OUR INSTRUMENT and report it as THE WORLD** — a catastrophic,
> paper-invalidating error, since the phantom rate **is the number this paper reports.**
>
> **The differential subtraction is what removes it.** And it works **because** that distortion is
> **static per device** — which is exactly the central claim of the "Same Signal" paper: these effects are
> **systematic and TIME-INVARIANT**. **That time-invariance is the load-bearing assumption of our entire
> method.** If it fails, the method fails.

### The framing gift 🎁 **[V]**

This experiment is, formally, **a bistatic RCS measurement of a plate with coherent background
subtraction.** That is not a WiFi-sensing trick — it is **the procedure IEEE Std 1502 recommends**:

> *"Coherent background subtraction is typically used and also recommended by IEEE Std 1502… yielding a
> **purified target signal, eliminating antenna crosstalk and parasitic reflections**, as long as
> **coherence between foreground and background measurement is ensured**."*

**Cite it that way.** A standards citation is enormously stronger than "a WiFi trick", and it names our
exact failure mode (*"as long as coherence is ensured"* — which is Part 5's whole point).

*(Note the role inversion worth stating in the paper: standard WiFi sensing keeps the **dynamic**
component and discards the **static** one. **Our target IS static and our background is the room.** The
mathematics is identical; the framing is **RCS**, not WiFi sensing.)*

### ⚠️ Caveats on the subtraction **[P]**

- **Thermal drift** of the RF front-end between the two recordings will **de-cohere** them.
- Take the two recordings **BACK-TO-BACK**, and **NEVER reset or re-tune the chips between them** — the
  **PLL phase changes on reset** (ESPARGOS confirms this in print).
- **Interleave** the recordings (plate-in / plate-out / plate-in / plate-out) so drift is **detectable**
  rather than silent.
- Prior art on exactly this failure mode: *"Background Subtraction with Drift Correction for Bistatic
  Radar Reflectivity Measurements"* (arXiv:2601.14080).

---

## PART 6 — 🚨 THE THREE DEFAULTS THAT WOULD HAVE SILENTLY KILLED THE EXPERIMENT

**Every one of these is ON by default. Two of them are left ON in Espressif's own official example.**
**With the stock config, we would never have seen the echo — and there would have been NO error message.**

| # | setting | default | what it does to us |
|---|---|---|---|
| **1** | **`channel_filter_en`** | **`true`** | Espressif's own doc: *"enable to turn on channel filter to **smooth adjacent sub-carrier**."* **A smoothing filter across subcarriers IS A LOW-PASS WINDOW IN THE DELAY DOMAIN.** It **directly attenuates and smears exactly the late taps we are hunting.** **Espressif's own `esp-csi` example ships with `.channel_filter_en = true`.** **[V]** |
| **2** | **`ltf_merge_en`** | **`true`** | Espressif's own doc: *"enable to generate htltf data by **averaging lltf and ht_ltf** data… **Default enabled**."* **In HT40 the LLTF covers only HALF the band** (64 bins on the primary channel). **Averaging it into the HT-LTF POLLUTES 20 of our 40 MHz.** **[V]** |
| **3** | **the raw buffer layout** | — | The 384-byte HT40 record is **`[LLTF(64) ‖ HT-LTF(128)]` CONCATENATED**. **IFFT that and you get meaningless garbage.** **Use HT-LTF ONLY.** *(ESPARGOS does exactly this — their paper states CSI is estimated "based on the high throughput long training field (HT-LTF)".)* **[V]** |

### The config we actually need (ESP32 / ESP32-S3) **[V]**

```c
wifi_csi_config_t csi_config = {
    .lltf_en           = false,  // HT40 LLTF covers only HALF the band -- useless AND dangerous
    .htltf_en          = true,   // THIS is the contiguous 40 MHz block
    .stbc_htltf2_en    = false,
    .ltf_merge_en      = false,  // ** CRITICAL ** default true -> averages LLTF in -> corrupts HT40
    .channel_filter_en = false,  // ** CRITICAL ** default true -> windows our CIR -> kills late taps
    .manu_scale        = true,   // fix the scaling so AGC cannot rescale between the two recordings
    .shift             = <tune>,
};
```

*Note: with `lltf_en = false`, the documented **`first_word_invalid`** hardware limitation (the first 4
bytes = first 2 subcarriers are invalid) now lands on HT-LTF bins 0 and 1 — **which are DC nulls anyway.**
Convenient. **Check the flag on every packet regardless.*** **[V]**

---

## PART 7 — The rest of the verified hardware facts

### 7.1 The HT40 subcarrier layout — the gap I feared **DOES NOT EXIST** **[V]**

I was worried HT40 CSI might be **two separate 20 MHz halves with a gap**, whose IFFT would ring and
produce sidelobes **indistinguishable from phantom taps**. **It is not.**

- **HT40 is ONE CONTIGUOUS 128-bin block on a uniform 312.5 kHz grid.** The HT-LTF field is
  `0~63, -64~-1` = 128 subcarriers spanning the **full 40 MHz**. Frequency of bin *k* =
  `f_center + k × 312.5 kHz`. **[V]** *(ESP-IDF `wifi.rst` CSI table; confirmed by
  `pyespargos util.get_frequencies_ht40()`.)*
- **BUT there is a 3-subcarrier NULL NOTCH at band centre.** ESPARGOS names it explicitly:
  `HT40_GAP_SUBCARRIERS = 3` — *"Gap between primary and secondary channel in HT40 mode, in
  subcarriers."* Their HT40 vector is **2 × 57 + 3 = 117** subcarriers, indices **−58…+58**; the three at
  **−1, 0, +1 are dead**. **[V]**
- **Size of the notch:** 3 × 312.5 kHz = **937.5 kHz = 2.6 % of the band.** **[D]**
- **Its IFFT signature is a WIDE, LOW-AMPLITUDE PEDESTAL** (a `−sinc(937.5 kHz)` term) — **NOT a discrete
  phantom tap.** ✅ **The worry was unfounded.** **[D]**
- **The real artifact generator is the 11 EDGE GUARD BINS** (|k| ≥ 59) — they set the sidelobe skirt.
  **Window (Hann/Blackman) before the IFFT.**
- **How published code handles it:** `espargos.csi.interpolate_ht40ltf_gap()` fills the 3 dead bins by
  **linear interpolation** between bins −2 and +2; the `azimuth-delay` demo then **zero-pads ×10** and
  calls `np.fft.ifft`. *(It applies **no window** — we should, given our dynamic-range problem.)* **[V]**
- **Null bins: what the ESP32 actually returns there is UNVERIFIED [P].** The docs never say whether
  guard/DC bins are zero or garbage. pyespargos **drops** the guards and **overwrites** the DC gap —
  implying they **do not trust them**. **Treat all 14 non-active bins as untrusted.**

### 7.2 Byte order, and a DISPUTED official parser **[V]**

- Each subcarrier = **2 signed bytes, IMAGINARY FIRST, THEN REAL.**
- To get ascending frequency order for HT-LTF: take buffer items **64…127** (= subcarriers 0…+63) and
  items **128…191** (= subcarriers −64…−1), and concatenate as **`[items 128..191] ‖ [items 64..127]`** →
  bins −64…+63 (an `fftshift`).
- ⚠️ **Espressif's OWN parser is disputed and the bug report is still OPEN.** `esp-csi` **issue #224**
  argues that `csi_data_read_parse.py` **mis-orders the HT-LTF halves**. **No maintainer has resolved
  it.** **DO NOT inherit their index list.** Derive it from the docs table and **validate empirically** —
  see **Rung 0.5**.

### 7.3 AGC will break the subtraction unless we stop it **[V]**

- The ESP32 reports **`agc_gain`** (1 dB units) and **`fft_gain`** (0.25 dB units) **per packet**.
  *(Confirmed in Espressif's own CSV header: `type,seq,mac,rssi,rate,noise_floor,fft_gain,agc_gain,channel,...`)*
- **Espressif ships an AGC control API:** `esp_csi_gain_ctrl_get_rx_gain()`,
  `esp_csi_gain_ctrl_record_rx_gain()`, `esp_csi_gain_ctrl_get_rx_gain_baseline()`,
  **`esp_csi_gain_ctrl_set_rx_force_gain()`** (force a fixed gain), and
  **`esp_csi_gain_ctrl_get_gain_compensation()`**.
- **pyespargos independently implements the compensation:** `scale_csi_by_reported_gain()` —
  `gain_db = 1.0·rx_gain + 0.25·fft_gain; scale = 10^(−gain_db/20)`.
  ⚠️ **Their gain bytes are SIGNED** (`>= 128 → −256`).
- **Belt AND braces:** set **`manu_scale = true`** with a **fixed `shift`** so the CSI scaling **never
  changes at all**. *(Default is `manu_scale = false` — automatic. **Change it.**)*

### 7.4 The HT40 trap is on the TRANSMIT side **[V]**

`esp-csi` **issue #52**: a user set 40 MHz and got **`cwb = 0` (20 MHz) in every single CSI record.**

> **`esp_wifi_set_bandwidth()` sets your CAPABILITY. The CSI bandwidth is determined by the PACKET YOU
> ACTUALLY RECEIVE.** Management frames, broadcast frames and legacy-rate frames are **all 20 MHz.**

**Our TX must transmit HT (802.11n) DATA frames at an MCS rate with 40 MHz.** And on every packet we must
**verify `rx_ctrl.cwb == 1`, `sig_mode == 1`, `len == 384`** — and **DISCARD anything else.**

### 7.5 Chip choice — **ESP32-S3.** Not plain ESP32. **Emphatically not C5/C6.** **[V]**

| chip | verdict |
|---|---|
| **ESP32-S3** | ✅ **USE THIS.** Correct 256/384-byte buffers. An **Espressif engineer states** (`esp-csi` #146): *"Among all Espressif chips supporting CSI functionality, **the performance of esp32 is comparatively lower**. Therefore, it is recommended to utilize the CSI functionality on **ESP32-C3 or ESP32-S3**."* |
| plain ESP32 | ⚠️ Works, but is the **weakest CSI** per Espressif's own engineer. |
| **ESP32-C3** | ❌ **HT20 only** for our purposes — half the bandwidth, double the resolution cell. |
| **ESP32-C6** | ❌❌ **BROKEN.** `esp-idf` **issue #14271**: returns only **128 bytes** (no L-LTF) and has **WRONG HT-LTF SUBCARRIER ORDERING**. **Still open.** |
| **ESP32-C5** | ❌❌ **TRAP.** Wants a **48 MHz** crystal (not 40), and has a **documented, UNRESOLVED shared-clock boot failure.** |

⚠️ **C5/C6 also use a COMPLETELY DIFFERENT `wifi_csi_config_t`** (`acquire_csi_ht40` etc., with **NO**
`channel_filter_en` / `ltf_merge_en` / `manu_scale`). **None of Part 6's advice applies to them.**

### 7.6 The UART is the bottleneck, not the radio **[P]**

`esp-csi` **issue #249**: 128 subcarriers @ 100 Hz over 2 Mbaud is stable; **256 subcarriers @ 100 Hz
DROPS PACKETS** — because the **stock example prints CSI as ASCII** (~2000 characters per packet).

**Our HT40 record is 192 subcarriers.** → **We must write a BINARY UART dump** (~75 % bandwidth saving).
**Espressif has not shipped an official binary output.** ⚠️ **Silent packet loss would corrupt the
coherent average — the very thing our +30 dB depends on.**

Also: **the CSI callback runs in the Wi-Fi task. Do NOTHING in it but `xQueueSend()`.**
And raise the TX rate with **`usleep()`**, not `vTaskDelay()` *(Espressif's own advice, issue #114)*.

### 7.7 No association needed **[V]**

We do **not** need to associate. The docs **recommend against it**: *"it is suggested to enable **sniffer
mode** to receive more CSI data by calling `esp_wifi_set_promiscuous()`."* Espressif's own examples call
`esp_wifi_set_promiscuous(true)`. **Use a fixed sender MAC and filter on it.**

### 7.8 ESP-IDF version **[U]**

**No version is required.** The classic `wifi_csi_config_t` (with `channel_filter_en` / `ltf_merge_en` /
`manu_scale`) is stable across **v4.4 → v5.x** for ESP32 / S2 / S3. **Use a recent v5.x.**

---

## PART 8 — 🔍 HAS ANYONE EVER DONE THIS? **NO.** (A verified negative result)

I searched thoroughly. **Nobody has published a delay tap from an ESP32 resolved against a known physical
reflector at a measured distance.** The complete landscape:

| work | hardware | delay domain? | **reflector tap validated vs ground truth?** |
|---|---|---|---|
| **ESPARGOS `azimuth-delay` demo** | ESP32-S2 × 8, HT40 | ✅ **YES** — real, working IFFT-to-delay code | ❌ **No.** A qualitative **live heatmap**. No quantitative validation, no known-distance reflector, **not in any paper.** |
| **ESPARGOS papers** (2408.16377, 2502.09405) | ESP32-S2 | ❌ **NO** | ❌ Purely frequency-domain / spatial (AoA, channel charting). **2502.09405 contains ZERO time-domain analysis.** |
| **Espressif `esp-crab`** | ESP32-C5 × 2 | ⚠️ README **says** *"extracting CIR data"* | ❌ **No — and the README is misleading.** Reading their code: they run a 64-pt IFFT on the **raw unordered buffer** and then use **only `x_iq[0]`** — **the DC bin, which is just the MEAN of the CSI vector.** It is a phase-of-total-channel sensor, **not delay-resolved.** **The word "CIR" is marketing.** |
| **Diadiuk & Pavlenko 2026** (ESP32-S3 ranging) | ESP32-S3, HT20 | ❌ **NO — they explicitly did NOT IFFT** | ❌ RSSI/amplitude only; MAE **1.45–3.90 m**. They **state** that sub-metre needs *"ToF processing of the CIR with parabolic peak interpolation"* as **FUTURE WORK**, and cite the **15 m range-bin limit of 20 MHz.** |
| **Splicer** (MobiCom'15) | Intel 5300 | ✅ | ✅ — but on the 5300, and it needs **multi-band splicing** to beat the 15 m bin. |
| **WiFi Radar via OTA Referencing** (arXiv:2602.05344) | **Intel AX210 + PicoScenes, 160 MHz** | ✅ | ✅ **YES** — bistatic range of a walking human vs **HTC VIVE** ground truth, **using the LoS path as the delay/phase reference.** ⭐ **This is EXACTLY our excess-delay method — validated. But at 4× our bandwidth, on a laptop NIC.** |
| **SoundiFi** (arXiv:2602.21573) | 802.11ax, 160 MHz, 2025 subcarriers | ✅ noise floor −115 dB | ✅ — but it needs a **coax-cabled reference channel.** |

### What this means — **both ways, stated honestly**

> **✅ THE NOVELTY:** The delay-domain machinery for ESP32 **exists as working open-source code**
> (pyespargos). The reflector-tap-versus-ground-truth methodology **is established at 160 MHz on Intel
> silicon** (arXiv:2602.05344). **NOBODY HAS CLOSED THE LOOP ON A $5 CHIP AT 40 MHz.** That is genuinely
> unclaimed ground.
>
> **⚠️ THE RISK:** It is unclaimed **for a reason** — the physics at 40 MHz is **tight**. See Part 4.5:
> **the echo is BELOW the quantization floor of a single packet.** It is *tight but not impossible*,
> **provided** we do the LoS-referencing before subtracting **and** put the plate at **10–14 m** rather
> than 6 m.
>
> **This is exactly the kind of question that should be MEASURED, not argued about.** That is the entire
> point of the ladder below.

---

## PART 9 — THE LADDER. One new thing per rung. Kill criteria written IN ADVANCE.

> **THE RULE: If a rung's kill criterion fires, we STOP and REPORT THE FAILURE. We do not climb past it.
> We do not tune until it passes. A negative result at any rung is a publishable finding and an honest
> one — and it is infinitely cheaper than discovering it three rungs later.**

---

### RUNG 0 — the pipeline, on synthetic CSI. **No hardware.**

**Tests exactly one thing:** does our code recover a delay tap that *we ourselves injected*?

**Setup:** Pure Python. Synthesise a CSI vector: `H[k] = Σᵢ aᵢ · exp(−j2π · k·Δf · τᵢ)` for a known set of
taps `(aᵢ, τᵢ)`. Add the 3-bin DC notch. Add the 11 edge guard bins. Add 8-bit quantisation. Add an
S-shaped phase distortion (the receiver artefact from Part 5). Add a random per-packet common phase and a
random per-packet STO.

**Procedure:** Run the *exact* pipeline the hardware will use — interpolate the notch → de-ramp (STO) →
LOS-reference → Hann window → zero-pad ×10 → IFFT → CFAR.

**Prediction:** Taps recovered at their injected delays. The random per-packet phase and STO **vanish**.
The S-shaped distortion **survives** in a single recording and **cancels** in the differential.

**KILL:** We cannot recover a tap we injected ourselves. *(Then the bug is ours, and no hardware would
have helped.)*

**Cost:** zero. **This runs today, with no hardware, and it must pass before anything is purchased.**

---

### RUNG 0.5 — 🚨 THE ORDERING CHECK. **30 minutes. Do this before ANYTHING else.**

**Tests exactly one thing:** is our subcarrier ordering correct?

**Setup:** Two ESP32-S3s. An empty corridor. **No plate.** One HT40 capture.

**Procedure:** fftshift → interpolate the 3-bin gap → Hann window → zero-pad ×10 → IFFT → plot `|CIR|`.

**Prediction:** **EXACTLY ONE dominant tap, and nothing else.**

**KILL / DIAGNOSE:**
- **TWO peaks**, or **a peak at a NEGATIVE delay** → **the subcarrier ordering is WRONG.** Try the other
  half-swap. *(This is `esp-csi` issue #224 territory — **their official parser is disputed and the bug is
  still OPEN.**)*

> ### **⚠️ WHY THIS RUNG EXISTS**
> **A subcarrier-ordering bug produces a spurious second peak in the delay profile.**
> **That looks EXACTLY like a phantom reflection.**
> **This one hour is the ONLY thing standing between us and WEEKS of chasing a reordering bug while
> believing we had discovered a physical echo.** It is the cheapest insurance in the entire programme.

---

### RUNG 1 — ⭐ THE ATOMIC TEST. **Does a wall echo exist AT ALL?**

**Tests exactly one thing:** is a reflector's excess-delay tap **visible** in ESP32 CSI?

**Setup:**
- 2 × ESP32-S3, **static, on a table.** *(**No car. No servo. No motion. No angle.**)*
- Baseline **b = 0.5 m** (measured with the tape).
- A **corridor of ≥ 15 m**.
- **One flat metal plate** (a baking tray is fine) at **d = 12 m** *(→ 3.13 resolution cells out — Part 4.2)*.
- TX ESP32 emits **HT 802.11n data frames at an MCS rate, 40 MHz**, at ~100 Hz.
- RX ESP32 in **promiscuous mode**, filtering on the TX's MAC, **binary UART dump**.
- Config **exactly as Part 6**. Verify **`cwb == 1`, `sig_mode == 1`, `len == 384`** on every packet.

**Procedure — INTERLEAVED, back-to-back, NEVER resetting the chips:**
> **plate-IN (1000 pkts) → plate-OUT (1000) → plate-IN (1000) → plate-OUT (1000)**
> *(Interleaving makes thermal drift **detectable** instead of silent.)*

Then, per packet: **de-ramp (STO) → divide by the LOS tap** → then **coherently average within each
recording** → then **subtract**. **THE ORDER IS NOT OPTIONAL (Part 5).**

**Prediction:** the differential `|CIR|` shows a **clear tap at 78 ns** (= (2 × 12 − 0.5)/c).

**KILL:** **No tap appears above the noise floor of the differential, with the plate at 10, 12 AND 14 m.**

> ### 🛑 **IF RUNG 1 FAILS, THE HARDWARE PROGRAMME STOPS.**
> **We report the negative result — "a 40 MHz commodity CSI receiver cannot resolve a specular reflector
> at 12 m" — which is itself a genuine, citable, useful contribution**, given that **nobody has ever tried
> it** (Part 8) and that the whole field assumes it works.
>
> **We do NOT tune until it passes. Tuning until a null result disappears is how mistake #5 happened.**

**Cost:** 2 × ESP32-S3 (**~$16**), a baking tray, a tape measure. **That is the entire bill for the single
most important experiment in the programme.**

---

### RUNG 2 — does the echo RANGE correctly? *(measures paper 2's RANGE BIAS)*

**Tests exactly one thing:** does the tap **move to the right place** when the plate moves?
**(PRECISION — one isolated target. NOT resolution. See Part 4.4.)**

**Setup:** identical to Rung 1. Move the plate: **d = 8, 10, 12, 14 m** (tape-measured).

**Procedure:** Rung 1's pipeline at each `d`. **Interpolate the peak to sub-bin precision** *(legitimate —
there is only ONE target; precision is SNR-limited, not bandwidth-limited)*.

**Prediction:** measured excess delay **τ = (2d − 0.5)/c**, i.e. the tap advances **linearly** with `d` at
**6.67 ns per metre**. **[D]**

**Measures:** the **RANGE BIAS** — paper 2's **6.45 m** number, **measured on real silicon for the first
time.**

**KILL:** the tap does not track `2d/c`; or the bias is so large the tap is unusable.

---

### RUNG 3 — can TWO targets be SEPARATED? *(measures RESOLUTION)*

**Tests exactly one thing:** the **c/2B = 3.75 m** resolution limit. **(RESOLUTION — two targets. This is
a DIFFERENT experiment from Rung 2. See Part 4.4.)**

**Setup:** **Two** plates. Separations of **8 m, 6 m, 4 m, 3 m, 2 m.**

**Prediction — a FALSIFIABLE prediction from theory:**
- **≥ 3.75 m apart → TWO resolvable taps.**
- **< 3.75 m apart → they MERGE into one.**

**This is a genuine test of theory, and it can fail.**

**KILL:** the two plates are **never** separable, **even at 8 m apart** → the front-end is broken, and
Rung 2's apparent success was an artefact.

---

### RUNG 4 — ⭐⭐ **THE PHANTOM RATE.** The headline number of all three papers.

**Tests exactly one thing:** **what fraction of detections correspond to NO real reflector?**

**Setup:**
- A **large space** *(the site-size constraint from Part 4.2 — at 3.75 m range resolution, reflectors
  must sit **well beyond** 3.75 m or they collapse into the direct-path bin and we measure **nothing**)*.
- **EVERY reflector surveyed with a tape measure.** Walls, pillars, radiators, lockers, doors.
- The ESP32 pair **static**, at a **tape-measured pose.**

**Procedure:**
1. Predict the **true** echo delays from the surveyed geometry — **reusing `radar/truth.py` exactly.**
2. Run detections through **the existing CFAR chain — `radar/processing.py`, unchanged.**
3. Match detections to true paths using **`eval/phantom.py`'s `phantom_stats_frames()` — the EXACT
   definition papers 2 and 3 use** *(frame-by-frame; **never pooled** — that was mistake #4)*.

> **⚠️ REUSING THE EXISTING CHAIN IS NOT AN OPTIMISATION. IT IS A REQUIREMENT.** A different chain would
> **confound the hardware comparison with an algorithm change**, and the resulting number would be
> comparable to **nothing**. The whole value of this measurement is that it lands on the **same axis** as
> paper 2's **89 %** and paper 3's **18.2 % / 0.1 %**.

**This is the number. Everything else in the programme was upstream of it.**

---

### RUNG 5 — **MUSIC vs CFAR.** **FREE.** *(Same recording. Two algorithms.)*

**Tests exactly one thing:** is the **89 % phantom rate** a property of **MUSIC**, or of **WiFi**?

**Setup:** **ZERO new hardware. ZERO new recordings.** Re-process **Rung 4's data** twice.

**Prediction — from papers 2 and 3:** **MUSIC ≈ 89 % phantoms. CFAR ≈ 0.1 %.**

**Why this matters more than almost anything else:** paper 1 blamed **WiFi**. Paper 2 blamed **phantoms**.
Paper 3's simulation says **the front-end** is the culprit. **This rung tests that on real silicon, for
free.**

---

### RUNG 6 — **MONOSTATIC vs BISTATIC.** **FREE.** *(Same session. Also log the building's own AP.)*

**Tests exactly one thing:** paper 3's headline finding — **geometry is everything** (18.2 % → 0.1 %).

**Setup:** **ZERO new hardware.** During the **same** Rung 4 session, the RX ESP32 **also** logs CSI from
**an access point that ALREADY EXISTS in the building** (position measured **once**, with the tape).

- **Monostatic arm** = our own on-table TX *(2d − b — Part 4.3, monostatic to within **6.7 %** of a cell)*.
- **Bistatic arm** = the building's AP *(|AP→R| + |R→RX| − |AP→RX|)*.

> **⚠️ THIS DOES NOT VIOLATE THE NO-ANCHORS RULE.** The AP is a **CONTROL**, not part of the sensor. It
> exists to **reproduce paper 3's cell-A geometry physically**. Our **sensor** — the thing we claim — is
> **only the two on-platform ESP32s.** **Say this explicitly in the paper**, or a reviewer will (rightly)
> think we smuggled infrastructure back in through the side door.

**Prediction:** the **bistatic** arm has a **dramatically higher phantom rate** than the **monostatic** arm,
**on the same data, in the same room, at the same instant.**

> ### 🎁 **THE PRIZE**
> **Rungs 4, 5 and 6 all come out of ONE STATIC RECORDING SESSION.**
> **The entire discriminating science of the programme — the phantom rate, MUSIC-vs-CFAR, and
> monostatic-vs-bistatic — is obtainable from two ESP32s sitting still on a table.**
> **No car. No motion. No SLAM. No pose estimation. No odometry. No servo.**

---

### RUNG 7 — angle *(ONLY if 1–6 pass)*

A **directional antenna on a servo.** **ZERO phase coherence required.** Rotate, capture a delay profile
per angle, assemble a **range–azimuth scan** — in the **identical format** to `radar/processing.py`.
**Legitimate ONLY because the platform is static.**

### RUNG 8 — motion *(ONLY if 7 passes)*

**And even then, we do NOT claim SLAM** (Part 2). We would claim **mapping under measured poses.**

---

## PART 10 — Bill of materials

### Rungs 0 – 3 (the entire critical path)

| item | qty | cost |
|---|---|---|
| **ESP32-S3 dev board** *(NOT plain ESP32; NOT C3/C5/C6 — Part 7.5)* | 2 | **≈ $16** |
| Flat metal plate (a baking tray) | 1–2 | ≈ $5 |
| Tape measure (**the ground-truth instrument**) | 1 | ≈ $5 |
| USB cables | 2 | ≈ $5 |
| **TOTAL** | | **≈ $31** |

**No car. No LiDAR. No Raspberry Pi. No motion capture. No SDR. No servo.**
A **laptop** does all processing offline. **A corridor of 15–20 m** is the only site requirement.

### Rung 4+ — additionally: a large surveyed space (a sports hall, a car park, a long atrium).

---

## PART 11 — What we claim, and what we DO NOT

### ✅ WILL claim (defensible; the literature supports it)
1. **The FIRST delay-resolved reflector measurement from ESP32 CSI**, validated against a tape-measured
   ground truth. *(Nobody has done this — Part 8.)*
2. **The phantom rate of a commodity 40 MHz CSI receiver**, measured on real silicon, using **the same
   definition** as papers 2 and 3 — **so the numbers are directly comparable.**
3. **MUSIC vs CFAR**, measured. *(Does the 89 % ceiling survive contact with reality?)*
4. **Monostatic vs bistatic**, measured. *(Does paper 3's 180× geometry effect appear in hardware?)*
5. **The instrument's own phantom contribution**, quantified and **divided out** — via coherent background
   subtraction (**IEEE Std 1502**).

### ⛔ WILL NOT claim
1. ❌ **A SLAM system.** *(Part 2. Four independent, citable reasons.)*
2. ❌ **A map.** *(No angle from one RF chain.)*
3. ❌ **Ego-motion / odometry.** *(Our estimator cannot recover yaw even from real 77 GHz radar.)*
4. ❌ **That WiFi replaces LiDAR or radar.** *(The bandwidth gap is 25–100× and money cannot close it.)*
5. ❌ **Anything about "all directions".** *(One RF chain = ZERO azimuths. Rung 7, later, and only
   mechanically.)*

### 🚨 The honesty guard — **a HIGH phantom rate is NOT a null result**

**Our own research PREDICTS that real hardware will REPRODUCE the ceiling.** So a high phantom rate at
Rung 4 **confirms the theory** — it does not refute it, and it **must not be tuned away.**

**The DISCRIMINATING measurements are the DIFFERENCES:**
- **MUSIC vs CFAR** *(Rung 5)*
- **bistatic vs monostatic** *(Rung 6)*

**Those are where the science is. Rung 4's absolute number is the anchor, not the finding.**

---

## PART 12 — Open questions. **DO NOT fill these by guessing.**

1. **[P]** The **echo strength estimates** (−30 dB at 6 m, −46 dB at 14 m for a 0.5 m plate) are
   **estimates**, not measurements. **The whole dynamic-range argument (Part 4.5) rests on them.**
   → **Rung 1 measures them for real.** Until then they are **[P]**.
2. **[P]** **What the ESP32 actually returns in the 14 null/guard bins** — the docs never say. pyespargos
   distrusts them. **Verify empirically at Rung 0.5.**
3. **[U]** Whether the receiver's **S-shaped phase distortion is stable enough** between two back-to-back
   recordings for the subtraction to cancel it. **This is the load-bearing assumption of the entire
   method.** The "Same Signal" paper says these effects are **systematic and time-invariant** — **but
   nobody has verified that for the ESP32 specifically.** **→ The interleaved recording schedule (Rung 1)
   is designed to MEASURE this, not assume it.**
4. **[U]** Whether **coherent averaging over 1000 packets actually delivers the full +30 dB** on this
   hardware, or whether residual per-packet phase noise erodes it. **→ Measurable at Rung 1: plot the
   noise floor versus N. It should fall as 10·log₁₀(N). If it plateaus, we have found our real limit.**
5. **[U]** Whether **`esp-csi` issue #224** (the disputed HT-LTF ordering) affects the ESP32-S3 as well as
   the chips discussed there. **→ Rung 0.5 settles it empirically.**

---

## PART 13 — Immediate next actions

| # | action | needs hardware? |
|---|---|---|
| 1 | **Withdraw** the old paper-4 spec and plan. Mark them **SUPERSEDED**. | no |
| 2 | Write `src/wifi_radar_slam/hw/csi.py` — the ESP32 CSI parser *(with our OWN index derivation, not Espressif's disputed one — Part 7.2)*. | no |
| 3 | Write `src/wifi_radar_slam/hw/delay.py` — de-ramp → LOS-reference → interpolate notch → window → zero-pad → IFFT. | no |
| 4 | Write `src/wifi_radar_slam/hw/synth.py` — the **synthetic CSI generator** (Rung 0), including the notch, the guard bins, 8-bit quantisation, the S-shaped distortion, and per-packet random phase + STO. | no |
| 5 | **RUN RUNG 0.** It must pass **before a single component is bought.** | **no** |
| 6 | Order **2 × ESP32-S3**. | — |
| 7 | Write the firmware *(config exactly as Part 6; **binary** UART dump)*. | no |
| 8 | **RUN RUNG 0.5** — the ordering check. **30 minutes.** | yes |
| 9 | **RUN RUNG 1** — the atomic test. | yes |

**Steps 1–5 and 7 need NO hardware at all.** The entire Python side can be built, unit-tested and proven
on synthetic CSI **before a single ESP32 arrives.** *(Test runner: `.venv/bin/python -m pytest`.)*

---

## PART 14 — The one-paragraph summary

**All three papers reduce to a single scalar: the PHANTOM RATE — the fraction of detections that
correspond to no real reflector.** MUSIC in a bistatic geometry gives **≈89 %** and mapping is destroyed;
CFAR in a monostatic geometry gives **≈0.1 %** and mapping works. **That number is a property of a single
pose, so it requires NO motion, NO map and NO SLAM to measure** — a tape measure supplies the ground truth.
**Therefore two ESP32-S3s sitting still on a table in a corridor, with a metal plate at 12 m, can measure
the headline claim of the entire programme for about $31.** A full self-contained SLAM system **cannot** be
proved on this hardware — for four independent and separately-fatal reasons — and **the honest limit is a
bandwidth gap of 25–100× that no amount of money can close, because it is set by the 802.11 standard
rather than by the budget.** We therefore claim **a sensor, validated against measured ground truth at
known poses — and we do not claim SLAM.**

---

*Prepared by Claude (Opus 4.8) for Mulham Fetna (ORCID 0009-0006-4432-798X).
Every **[V]** claim in this document has a source that was fetched and read. Every **[U]** claim is marked
as such and **must not enter a paper** without verification.
Arabic mirror: `docs/paper4-restart-static-bench.ar.md`.*
