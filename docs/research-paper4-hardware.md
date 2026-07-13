# Paper 4 — hardware research reference

**Compiled 2026-07-13** from two research passes (WiFi-CSI hardware; array phase calibration).
This is the **durable record**: everything the spec and plan rest on, plus the material that did
not make it into either but will matter later.

**Verification legend:** **[V]** verified — the source was fetched and says this · **[P]** partial ·
**[U]** unverified — **do not put in a paper**.

---

## 1. The four facts the whole programme rests on

| # | fact | why it is load-bearing | status |
|---|---|---|---|
| 1 | **Absolute time-of-flight is NOT measurable on commodity CSI.** Packet-detection delay is *"an order of magnitude larger than time of flight."* | Kills any method needing absolute range. | **[V]** arXiv:2206.09532 |
| 2 | **But the EXCESS delay — relative to the first/LOS arrival — IS preserved.** STO/SFO/CFO enter as a phase ramp **common to every path in a packet**. | **Our bistatic ellipse and our monostatic range both need exactly the excess.** This is why a **$5 chip** can run the method. | **[V]** ibid. |
| 3 | **The carrier does not matter** (paper 3, cell B→C). | **Licenses 2.4 GHz hardware** even though the sim ran at 5.2 GHz. We are spending a result we earned. | ours |
| 4 | **A receiver's own frequency-selective response manufactures phantom taps.** PicoScenes: >15 dB swing across subcarriers, *"causes a **phantom object** that interferes with the H_air measurement"*, and it is **not** removed by SpotFi-style linear-fit sanitisation. | The phantom rate is **the number this paper reports**. Uncorrected, we would measure **our instrument** and call it **the world**. → `hw/calib.py`. | **[V]** PicoScenes |

---

## 2. ESP32 — what it can and cannot do

| | | status |
|---|---|---|
| CSI bandwidth | **HT40 = 40 MHz** real (384-byte buffer; 128 subcarriers per LTF field) | **[V]** ESP-IDF vendor-features doc |
| Band | **2.4 GHz ONLY** | **[V]** |
| RF chains | **ONE.** "Antenna diversity" is an RF **switch** (RTC6603SP) — one antenna at a time | **[V]** ESP-IDF PHY guide |
| ⇒ AoA from one ESP32 | **IMPOSSIBLE** — follows from one RF chain | — |
| ESP32-C5 (5 GHz) | adds 5 GHz, **but its 5 GHz CSI returns static IQ that never changes** | **[V]** esp-idf issue #18493 |

**Resolution at 40 MHz:** path-length `c/B` = **7.5 m**; monostatic round-trip range `c/2B` =
**3.75 m**. *(Confusing these two is exactly the factor-of-2 error corrected in paper 2.)*

### Multiple ESP32s

- **They CAN be made phase-coherent** — **ESPARGOS** built an **8-ESP32 coherent array doing MUSIC
  AoA**, via a daisy-chained 40 MHz reference **plus** a distributed phase-reference signal (a shared
  clock alone is **not** enough: the PLL re-locks with random phase). **[V]** arXiv:2502.09405
  - **I nearly published the opposite claim.** Capped at 2.4 GHz / **20 MHz**.
  - ⚠️ **Whether an ESPARGOS-class array can run HT40 (40 MHz) is UNVERIFIED.** **[U]**
- **They need NO coherence as transmitters.** Multiple TX = multiple independent bistatic ellipses.
  *(Superseded for us: an anchor mesh is not SLAM. Kept because the physics is sound and may serve a
  different paper.)*

> **A coherent ESP32 array buys angle by paying with half the bandwidth (20 vs 40 MHz → 15 m vs
> 7.5 m path-length resolution). For a phantom-rate experiment, ONE ESP32 at 40 MHz beats FOUR at
> 20 MHz** — bandwidth is what resolves the delays the phantom rate is *defined on*.

---

## 3. Phase-coherent multi-antenna CSI (only needed if we ever want AoA)

| platform | antennas | bandwidth | price | status |
|---|---|---|---|---|
| **Intel 5300** (Linux 802.11n CSI Tool) | **3 coherent** | 40 MHz | **$60–125** used | **[V]** — needs kernel **3.2–4.2**; phase reliable **only at 5 GHz** on this NIC |
| **WiROS + ASUS RT-AC86U** | **4 coherent** | (bandwidth **not confirmed**) | **~$110**, **5.3° median bearing error** post-calibration | **[P]** — **this is the highest-value open question**; if it is 80 MHz it beats the 5300 on every axis |
| **Intel AX210 + PicoScenes** | **2** | **160 MHz** | card **$29.95** | **[V]** card; **[U]** licence cost — *and its per-chain phase offset is **UNCHARACTERISED in the literature*** |
| **Nexmon** (Broadcom) | up to 4 cores (**RPi variant = 1 antenna, no AoA**) | up to 80 MHz | varies | **[P]** — re-tune **re-randomises** phase; nexmon's own issue #222 asking about this is **unanswered** |

⚠️ **My claim that "4 antennas AND ≥80 MHz does not exist" is probably FALSE** (WiROS/RT-AC86U).
**Do not repeat it until settled.**

⚠️ **KrakenSDR cannot reach 5.2 GHz.** **[V]**

---

## 4. Array phase calibration — the folklore is wrong in a way that HELPS

> **The per-antenna phase offset is NOT random per packet.** It is a **per-boot** (Intel/Atheros/USRP)
> or **per-retune** (Broadcom, USRP LO) constant, drawn from a **small discrete set**. What *is* random
> per packet — CFO, packet-detection delay, SFO — is **common to all antennas and cancels exactly** in
> the inter-antenna difference. **One calibration per power cycle suffices.**
> — Zubow et al., arXiv:2005.03755, p.2, **[V] read verbatim**

| source of phase error | common-mode or per-chain? | cadence | fix |
|---|---|---|---|
| residual CFO | **common** | per packet | cancels in AoA |
| SFO/STO → linear-in-subcarrier slope | **common** | per packet | SpotFi Alg. 1 (**one** joint slope across **all** antennas) |
| packet-detection delay | **common** | per packet | same |
| **PLL/LO initial phase** | **PER-CHAIN** | **per boot / per retune** | **the one you must calibrate** |
| cable/splitter electrical length | per-chain | static | **cable-swap trick** (below) |
| analog filter chain (`Hdist`) | per-chain, **frequency-selective** | static | **wired per-link profile** — *this is what bites us at Stage 0* |
| mutual coupling | per-chain, **angle-dependent** | static | coupling matrix / manifold |

**Measured discrete structure [V]:** Intel 5300 / Atheros AR9380 → **2 values, ≈π apart**, and *the
same values across different chips of the same model* — **lookup tables are published** (Zhang et al.,
IEEE Systems Journal 2019). Intel 9260 → up to **4 values, frequency-dependent**. **AGC does NOT
corrupt inter-chain phase** (it is a per-packet *amplitude* scalar).

### The procedure (ArrayTrack, NSDI'13, Eqs. 9–12) **[V]**

Splitter + reference tone into all channels, with the **cable-swap-and-average trick**: measure the
offsets, **physically swap the cables at the splitter**, measure again, average. This removes the
**LO offset AND the cable imbalance simultaneously** — so **matched cables stop mattering**.

ArrayTrack states the deeper problem verbatim: *"small manufacturing imperfections exist for SMA
splitters and cables labelled the same length"* — **nominally equal cables are not electrically
equal.**

**Every serious DF platform does a version of this. None skips it. [V]**
KrakenSDR (internal noise source + RF switches, automatic) · Ettus (*"re-align the LOs after each
tune command"*) · ADI Phaser (boresight tone → `phase_cal_val.pkl`).

### The cable maths (5.2 GHz)

| velocity factor | ° per mm | 1 mm mismatch → bearing bias (4-elt ULA, λ/2, broadside) |
|---|---|---|
| 0.66 (solid PE) | **9.46°/mm** | **3.0°** |
| 0.70 (foam) | 8.92°/mm | 2.8° |

### Accuracy: why calibration is not a refinement

CRLB ∝ **1/(SNR · snapshots · M³)**. For a 4-element ULA at λ/2, broadside:

| | 1 snapshot | 10 | 100 |
|---|---|---|---|
| SNR 0 dB | 5.77° | 1.82° | 0.58° |
| SNR 10 dB | **1.82°** | 0.58° | 0.18° |
| SNR 20 dB | 0.58° | 0.18° | 0.06° |

Residual per-element phase → bearing **bias**: 1° → 0.32° · 5° → 1.59° · **10° → 3.18°** · 30° → 9.55°.

> **An uncalibrated array produces a bias 10–100× larger than its own noise floor.** Post-calibration,
> Zubow et al. measured a residual phase std of **0.05°** — negligible.

Beamwidth of the same array: **25.4°–38°**. MUSIC beats that by an order of magnitude **only if the
array is calibrated**.

### ⚠️ Two traps

1. **"SpotFi needs no calibration" is a TRAP.** That claim is about the **environment**, not the array.
   Algorithm 1 fits **one** slope jointly across all antennas and subtracts the same slope from each —
   so it **preserves** inter-antenna differences and therefore **never removes the per-chain PLL
   offset**. **[V]** read pp.7–8.
2. **Häfner et al. (open access, DOI 10.1155/2019/1523469):** *"resolution of two **coherent** sources
   necessitates array calibration."* **Multipath IS coherent sources.** This bears directly on
   phantom-target work. **[V]**

---

## 5. Ground truth — and why we chose a tape measure

### Odometry is a trap **[V]** (Borenstein & Feng, UMBmark, Table I read directly)

- **Uncalibrated: 310 mm over 16 m ≈ 1.9 % of distance travelled.**
- Meticulously calibrated: 26 mm systematic — but **non-systematic σ ≈ 32 mm remains**.
- Verbatim: *"Typical odometry errors will become so large that the robot's internal position estimate
  is **totally wrong after as little as 10 m of travel**."*
- **A single 10 mm bump under one wheel = 0.6° of PERMANENT yaw error.** A power cable on the floor
  costs you half a degree, forever.
- ⚠️ **Gyrodometry numbers are UNVERIFIED [U]** — do not cite a "gyro → <0.5° heading" figure.

**Why this does not hurt us:** with **one RF chain there is no AoA**, so we measure **range only** —
and excess path length depends on **position alone, not heading**. Yaw enters solely via the 0.4 m TX
offset, where 5° of error displaces the transmitter by **3.5 cm** against a **3.75 m** resolution cell.
**The thing odometry is worst at is the thing we do not need.** We use a **measured track**.

### If we ever DO buy a LiDAR

| | **RPLIDAR C1 — $69** | RPLIDAR A1M8 — $99 |
|---|---|---|
| principle | **DTOF** | triangulation |
| accuracy | **±30 mm, FLAT** | **≤1 % of distance** |
| …at 12 m | **±30 mm** | **±120 mm** |

> **The A1M8 is a trap: "±1 %" sounds fine until you convert it — ±120 mm at the far wall is the
> entire error budget. The cheaper part is the more accurate one.** **[V]**
> Equal-price alternative: **LDROBOT LD19/D500, $69**, ±20–30 mm, and lighter (45 g vs 110 g).

**Product status [V]:** RPLIDAR **S1 — DISCONTINUED** · **A2M8/A2M7/A2M6 — EOL** (A2M12 $229 is the
live one) · Hokuyo **URG-04LX — DISCONTINUED** (historically ~$1,040; **worse than a $69 C1 on every
axis** — cite only as a historical price anchor). Skip the S-series ($299–549): you are paying for
30–40 m of range you do not have indoors.

**Cartographer accuracy [V]** (Hess et al., ICRA 2016, **Table II read directly**): **2.3–4.5 cm** and
**0.35–0.54°** on normal indoor datasets (Intel 0.0229 m; Aces 0.0375; MIT CSAIL 0.0319; Freiburg 79
0.0452). ⚠️ **But feature-poor repetitive spaces blow it up to 5.2 m** (Freiburg hospital) — a warning
about *where* you drive.

**The best single data point for the budget argument [V]:** Hess et al. built a 5 cm map using a
**Neato vacuum-cleaner lidar "which costs under $30"** and got **−0.2 % to +0.8 %** relative geometry
error against laser-tape measurements.

### The admissibility rule — a citation that closes an objection before it opens

*Benchmarking Egocentric Visual-Inertial SLAM at City Scale* (arXiv:2509.26639) **[V]** states its
pseudo-ground-truth *"has an overall accuracy of ~20 cm"* and that this is *"sufficiently accurate to
measure keyframe errors larger than 50 cm."*

> **A reference is valid when it is several times better than the errors it must resolve. They accept a
> 2.5× margin. A tape-measured track at ~1 cm against a WiFi map of 3.75 m resolution is ~375×.**
> **Put a sentence like this in the paper.**

**Hygiene precedent [V]** (arXiv:2407.09242 — RPLIDAR S2 + slam_toolbox labelling WiFi RSSI): they
built a **separate manual grid ground truth** and pointedly did **NOT** call the SLAM output "ground
truth". **Imitate this.** Call ours a **"reference trajectory"**, never "ground truth".

**⚠️ The cautionary example [V]** (arXiv:2501.09490): reports *"Cartographer ATE = 0.024 m"* — but on
reading it, **their ground truth IS the Hector SLAM trajectory.** They **scored one scan-matcher with
another.** **Cite it *as the trap*, not as evidence.**

---

## 6. Antennas

- **λ/2 spacing:** 2.4 GHz → **6.25 cm**; 5.2 GHz → **2.9 cm** (research suggests **26 mm** in
  practice **[P]** — verify against the actual channel centre before cutting anything).
- **Stages 0–1 need NO array.** One RF chain makes an array *physically meaningless*. A $2 dipole is
  correct, not a compromise.
- **Directional antennas (patch/Yagi) buy gain and a narrower FOV — we need neither** for the phantom
  measurement. *But* a directional antenna **on a servo** buys **mechanically scanned azimuth** with
  **zero phase coherence** → range–azimuth scans in the identical format to `radar/processing.py`.
- **Phase-matched cables are a waste of money** — use ArrayTrack's cable-swap trick instead.

---

## 7. Open questions — **do NOT fill these by guessing**

1. **[HIGHEST VALUE]** Does **WiROS + ASUS RT-AC86U** really give **4 coherent antennas at 80 MHz** for
   ~$110? If yes, it beats the Intel 5300 on every axis and my "you must choose aperture OR bandwidth"
   claim is simply **false**.
2. **Intel AX200/AX210 per-chain phase offset is UNCHARACTERISED** in the literature — no paper
   validates AoA on it. Stage 3 buys bandwidth into **unknown** AoA territory.
3. **Can an ESPARGOS-class coherent ESP32 array run HT40 (40 MHz)?** If so, the "an array costs you
   half your bandwidth" trade-off weakens.
4. **Does `nexmon_csi` capture all four bcm4366c0 RX cores coherently per packet?** (Its own issue #222
   asks exactly this and is **unanswered**.)
5. **PicoScenes licence cost/terms** — site returned 404/403.
6. **SDR price table** (USRP B210/X310, bladeRF, LimeSDR) — never returned.
7. **ESPARGOS price / purchasability** — site returned 403.
8. A fetched claim that **bcm43455c0 works on Raspberry Pi 5** — **treat as FALSE until checked.**
9. **Gyrodometry** heading-error figures — **[U]**, PDF unobtainable.
10. Friedlander & Weiss 1991 mutual-coupling text — DOI verified, **content paywalled [P]**.

---

## 8. Practical build notes

- **slam_toolbox chokes** when an LD06/LD19 scan has a *varying* number of range readings per
  revolution (455 vs 460). The LDROBOT driver exposes a **`lidar.bins`** parameter to pin it. Budget an
  afternoon. **[P]**
- **Independence argument, stated plainly:** LiDAR SLAM and WiFi ranging share **no sensing physics and
  no estimator**. For a WiFi-vs-radar comparison, a LiDAR reference is independent of **both** arms.
  Say so explicitly, quantify it, and the circularity objection is closed.
