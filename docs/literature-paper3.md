# Paper 3 — literature & market synthesis (WiFi vs automotive radar)

Deep-research pass (2026-07-12): 5 threads, 105 agents, 0 errors, adversarially verified
(3-vote). Threads 4 and 5 returned **nothing** and were re-run separately.

## Novelty gap (HIGH confidence, 3-0)

**No published work compares WiFi/CSI sensing against automotive mmWave radar for SLAM,
odometry, or mapping.** Every existing WiFi-vs-radar head-to-head is confined to **human
sensing** (HAR / pose classification): RadarConf24 (Dahal et al.), Electronics 2025 14(8)
1518, MM-Fi (NeurIPS 2023 D&B), XRF55 (IMWUT 2024), OPERAnet (Sci. Data 2022). None reports
pose or map metrics (ATE, drift).

⚠️ **Framing correction from the verifiers:** do *not* rest the novelty claim on the
RadarConf24 paper's self-declared "first". The correct phrasing is: *existing WiFi-vs-radar
head-to-heads are confined to human sensing; none addresses SLAM, odometry or mapping.*

One paper is a **false positive**: Milani/Colone/Lombardo, RadarConf18, "WiFi emission-based
vs passive radar localization" — **both arms are WiFi-illuminated** (a 2.4 GHz passive
bistatic radar), not an automotive FMCW sensor.

## 🚨 Warning 1 — do NOT assume parity; radar wins every existing comparison (3-0)

| Study | Radar | WiFi-CSI |
|---|---|---|
| RadarConf24, matched 7-class HAR (identical CNN, split, subjects) | **97.78 %** | 65.09 % |
| Electronics 2025, 6 cross-scene transfers (hardware-synchronised) | wins **all six**, by 7–8 pts | — |

**Mandatory caveats** (the verifiers insisted, and a reviewer will too):
1. "Matched" means matched *activities/subjects/classifier* — **not sensor parity**. The
   RadarConf24 radar is an INRAS Radarbook2 (76–80 GHz, **4 GHz** bandwidth) against a
   Raspberry Pi + nexmon at **80 MHz**: a **~50× sensing-bandwidth asymmetry**.
2. The RadarConf24 WiFi arm is **anomalously weak versus its own literature** (raw STFT over
   hand-picked subcarriers, no denoising / PCA / phase sanitisation). CSI-HAR on the same
   RPi+nexmon+CNN class routinely reports **95–98 %**.

So the gap is real but **overstated by that experiment**. Paper 3 must confront the
expectation head-on rather than assume equivalence *or* accept the 32.7-point figure at face
value.

## 🚨 Warning 2 — the physical-layer deficit is KNOWN, and it indicts our own pipeline (2-1)

Li, Vishwakarma, Tang, Woodbridge, Piechocki, Chetty, *"On CSI and Passive Wi-Fi Radar for
Opportunistic Physical Activity Recognition"*, **IEEE Trans. Wireless Commun. 2021**
(DOI 10.1109/TWC.2021.3098526), states verbatim:

> "with the low sampling rates of the 20 and 40 MHz channel bandwidths (50 ns and 25 ns time
> resolution, respectively), the direct signal may arrive between sampled intervals, giving
> rise to **distance estimation errors in the order of several meters**"

and

> "**Due to the limited bandwidth in WiFi, the range resolution is not sufficient** for indoor
> applications"

— **they abandon range entirely and use Doppler only.**

**This matters enormously for papers 2 and 3.** Our whole pipeline is *range-based* (bistatic
delay + AoA → ellipse triangulation). That is precisely what this group says does not work.
Paper 2's measured **6.45 m range bias** is not an anomaly — it is **the known physics, which
we quantified**. Even the most favourable modern WiFi (320 MHz, 802.11be) gives ~0.47 m range
resolution against ~4 cm for a 4 GHz-sweep 77 GHz FMCW radar.

**Action taken:** paper 2 (still held) was amended to cite this and to reframe its
contribution accordingly — see below.

## Ghost / phantom detections — qualitative parallel, NO number (3-0)

Hong, Petillot, Wang, *"RadarSLAM: Radar based Large-Scale SLAM in All Weathers"*, IROS 2020 /
IJRR 2022 (doi:10.1177/02783649221080483), §III-B.1 verbatim:

> "the peaks can be **distributed randomly across the whole radar image, even for the areas
> with no real object**, due to the speckle noises"

— forcing an explicit probabilistic extraction step. Multipath ambiguity is also one of three
stated reasons Bag-of-Words loop closure fails on radar.

**So the phenomenon is known in radar — but the literature gives NO percentage.**
⚠️ Do **not** assert a numeric equivalence with our ~89 % WiFi figure. The honest position:
we contribute the **first quantification**, and paper 3 can measure whether radar's rate is
comparable — which would be genuinely new.

## The bar a radar baseline must clear

**Spinning radar (Oxford Radar RobotCar / MulRan / Boreas):**
- **CFEAR** (learning-free, classical scan matching): **1.76 %** translation error, Oxford
  (IROS 2021) → **1.09 %** (T-RO 2023). BFAR: 1.55 %.
- **CFEAR-CTF-S10**: **0.61 % / 0.2°per100 m** on the **held-out** Boreas test set, at 68 Hz,
  with cross-dataset generalisation without retuning.
- **DRO** (Doppler-aware, gyro-aided; RSS 2025) **leads the Boreas leaderboard at 0.26 %**
  (0.18 % with a Doppler-enabling modulation pattern). This — not CFEAR — is the top number.

Notably **CFEAR is learning-free and beats learned baselines** (Under the Radar, HERO) —
directly relevant to our pure-signal-processing vs deep-learning axis.

**Evaluation protocol:** the accepted radar-odometry protocol is **KITTI-style drift %**
(translational error over 100–800 m sub-trajectories; rotational error in °/100 m), scored on
hold-out sequences, run online/incrementally. **ATE is NOT the ranking metric.** We must
report drift %, not only ATE.

**4D imaging radar is a different, much weaker story.** SNAIL Radar (IJRR 2025; 44 sequences;
Oculii Eagle + Continental ARS548): current 4D-radar odometry is *"far from robust"* — ATE
spans **0.2 m → 216.7 m** (best/worst, same method, same radar, different sequence). A
credible 4D/automotive-radar baseline must report a **spread**, not a headline.

## Open threads (re-run separately, 2026-07-12)

- **Radar EM-ray-tracing simulation practice** — no claim survived verification. We have no
  sourced basis for how to simulate FMCW radar (beat-signal + range-Doppler vs direct path
  extraction), which is the *foundation* of the radar baseline.
- **Cost** — **zero** sourced pricing for 77 GHz / 4D radar or CSI receivers survived. The
  cost argument, the project's stated selling point, currently has no citable backing.

## Implications for paper 3's design

1. **Novelty is defensible** — but phrase it as "no WiFi-vs-radar SLAM/odometry/mapping
   comparison exists," not "first WiFi-vs-radar comparison."
2. **Expect radar to win.** Design the study to *explain* the gap (bandwidth vs geometry vs
   active TX — an ablation), not to manufacture parity.
3. **The cost argument is dead on arrival** at radar prices (~same order as the WiFi package).
   WiFi's remaining case is **zero marginal cost** (hardware already present, transmitter
   free), not performance. Be prepared to publish that.
4. **Report drift %**, and treat the radar baseline against CFEAR/DRO numbers.
5. **The ghost-rate comparison may be the real contribution:** the first quantification of
   phantom rate in *both* modalities, with a shared instrument.
