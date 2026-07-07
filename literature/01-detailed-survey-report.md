# WiFi Signals as a Radar Replacement for Automotive SLAM
## A Detailed, Verified Literature Survey and Novelty Positioning Report

**Project:** WiFi-Radar — a WiFi-based radar-replacement sensor that exploits ambient WiFi signals in
the environment, together with an on-vehicle WiFi antenna, to produce a 3D scan of the surroundings
that can substitute for radar in Simultaneous Localization and Mapping (SLAM) pipelines.

**Document type:** Detailed related-work survey and novelty-positioning report (Rounds 1–2).
**Intended use:** Foundation for a Q1 journal submission; source material for the paper's Related Work,
Introduction, and Motivation sections; and a contact sheet for author outreach.
**Compiled:** 2026-07-03 (verification run) / 2026-07-04 (report).
**Companion file:** `00-literature-foundation.md` (condensed, tabular version of the same evidence).

---

## 0. How to read this report

This is the *verbose* companion to the condensed foundation document. Where the foundation doc gives
you tables and one-liners, this report gives you:

- the **full narrative** for each of the four research threads;
- **per-paper deep dives** with the verified quotations that back each claim;
- the **novelty argument** developed at length rather than asserted;
- a **technical feasibility and risk register** that turns the open questions into a research agenda;
- an **annotated bibliography** with access status and author contacts.

Every substantive claim in Sections 3–6 carries a **confidence label** (High / Medium) that reflects
adversarial verification, not the author's enthusiasm. Two claims that *failed* verification are
documented in Section 11 precisely so they do not leak into the paper. Treat this document as a
scaffold to be edited into prose, not as finished paper text.

A note on epistemic honesty, which matters for a Q1 venue: several of the most exciting results (the
2025–2026 preprints) are **self-reported and un-peer-reviewed**. They are cited here because they
establish *feasibility and direction*, not because their headline numbers should be repeated as
settled fact. The report flags each such case explicitly.

---

## 1. Executive summary

The central question behind this project — *can existing WiFi signals, received on a moving vehicle,
be turned into a radar-grade 3D sensing modality for SLAM?* — sits at the intersection of four mature
but historically separate research communities. This survey mapped all four and subjected the
load-bearing claims to three-vote adversarial verification. The result is unambiguous in one crucial
respect: **each of the four component capabilities has been demonstrated, but never in the combination
this project proposes, and never on a moving vehicle outdoors.**

The four threads and their state of maturity:

1. **Passive WiFi radar** is a *mature* field. Researchers have used ambient WiFi access points as
   "illuminators of opportunity" for more than a decade, detecting moving people through walls,
   extracting both range and Doppler, and recently tracking targets to sub-metre accuracy — but every
   demonstration is indoor, with fixed geometry.

2. **WiFi CSI sensing and imaging** has advanced to genuinely dense reconstruction: 3D human skeletons,
   multi-person pose comparable to millimetre-wave radar, depth images, and even diffusion-model image
   synthesis from channel data. Yet the field is *repeatedly and self-admittedly* confined to
   small (2–4 m), fixed indoor zones and generalizes poorly to unseen environments.

3. **RF-based SLAM** already provides the mathematical machinery we would need — most importantly the
   "multipath component as virtual transmitter" formulation of Channel-SLAM, which requires no prior
   map and no knowledge of reflector positions. A 2026 survey confirms that the *joint* treatment of
   communications and SLAM is still nascent.

4. **ISAC / IEEE 802.11bf / mmWave WiFi** is the standardization and high-resolution frontier. The
   Round-2 pass (Section 6) **settled it decisively:** commodity sub-7 GHz WiFi is bandwidth-limited to
   ~1–4 m range resolution (ΔR = c/2B) and *cannot* reach radar grade; only 60 GHz mmWave (~8.5 cm) can.
   This forces a central design fork — reframe around multipath-geometry SLAM, or move to 60 GHz/802.11bf.

Against this backdrop, the **defensible contribution** of the project crystallizes: *an on-vehicle,
mobile, outdoor WiFi (802.11) passive-radar / CSI system that produces 3D scans usable inside a SLAM
pipeline.* The closest existing prior art is a passive radar mounted on a moving car — but it uses **5G
downlink**, not WiFi, as its illuminator. Substituting an ambient-WiFi illuminator, on a vehicle,
outdoors, with a SLAM-facing 3D output, is an unoccupied point in the design space. The novelty is not
a single clever trick; it is the *transfer* of indoor/static WiFi sensing into the mobile/outdoor
automotive regime, and the engineering required to make that transfer work is exactly the paper's
technical content.

Two cautions, both derived from claims that were adversarially **refuted**, must shape how the paper is
written. First, do **not** argue that WiFi's narrow bandwidth restricts passive radar to velocity
(Doppler) measurements only — cross-ambiguity processing recovers full range–Doppler surfaces;
bandwidth limits range *resolution*, not range *extraction*. Second, do **not** attribute the automotive
gap to any existing survey — no source states it. The gap is our own, well-founded inference from the
pervasive indoor/static pattern across dozens of papers, and it should be argued as such.

---

## 2. Methodology of this survey (so reviewers trust the map)

This survey was produced by a deep-research harness rather than ad-hoc searching, and the procedure is
worth recording because it affects how much weight each claim can bear.

- **Decomposition.** The research question was decomposed into five complementary search angles:
  (1) passive WiFi radar / bistatic / illuminators of opportunity; (2) WiFi CSI sensing, imaging and
  3D reconstruction; (3) WiFi/RF-based SLAM, multipath-assisted localization and RF mapping;
  (4) ISAC, IEEE 802.11bf and mmWave/60 GHz WiFi imaging; and (5) the automotive/vehicular novelty gap
  plus datasets and toolkits.

- **Search and fetch.** Each angle was searched independently; the union of results was de-duplicated
  and the **24 highest-value sources were fetched in full** and mined for falsifiable claims,
  yielding **107 candidate claims**.

- **Adversarial verification.** The **25 most load-bearing claims** were each submitted to a
  three-vote verification in which the verifiers were instructed to *refute*. A claim survived only if
  it was not killed by a majority; a claim was killed on a 2/3 (or 3/3) refutation. **23 claims were
  confirmed; 2 were killed; 0 remained unverified.** After semantic de-duplication, the confirmed set
  condensed to **11 findings**.

- **What this means for the paper.** The confirmed findings are safe to build arguments on. The two
  killed claims (Section 11) are recorded as anti-patterns. Thread 4, which produced no surviving claim
  in Round 1, was completed by a dedicated Round-2 verification pass (108 agents; Section 6) rather than
  left unsourced — an honesty the paper's related-work section should preserve.

**Run statistics (both rounds):** Round 1 — 5 angles · 24 sources · 107 claims · 25 verified · 23
confirmed / 2 killed · 106 agents. Round 2 (Thread 4) — 108 agents · ~25 further claims verified.
Combined: ~214 agent invocations, ~4M tokens.

---

## 3. Thread 1 — Passive WiFi Radar: a mature foundation

### 3.1 The concept and why it is the right analogue

Passive bistatic radar (PBR) abandons the transmitter. Instead of emitting its own waveform, it listens
to an existing "illuminator of opportunity" — here, an ordinary WiFi access point — on two channels: a
*reference* channel that captures the direct-path transmitted signal, and a *surveillance* channel that
captures the same signal after it has scattered off targets in the scene. Cross-correlating the two
(the Cross-Ambiguity Function, CAF) yields a range–Doppler surface from which moving targets are
detected. This is the closest conceptual analogue to classical automotive radar, which is why it
anchors Thread 1: if a car already carries a WiFi antenna, the *reception* hardware for a passive radar
is largely already present.

### 3.2 Foundational demonstration: through-wall personnel detection (High confidence)

The field's foundational, most-cited demonstration is **Chetty, Smith & Woodbridge (2012)**, "Through-
the-Wall Sensing of Personnel Using Passive Bistatic WiFi Radar at Standoff Distances" (*IEEE
Transactions on Geoscience & Remote Sensing*). The paper reports, in its own words, *"the first
through-the-wall (TTW) detections of moving personnel using passive WiFi radar,"* and — critically for
us — it **extracts both range and Doppler**, with the measured Doppler shifts *agreeing with bistatic
predictions*. This is the empirical rebuttal, incidentally, to the tempting-but-false intuition that
WiFi PBR can only measure velocity (see Section 11). Range and Doppler are both recoverable; the
constraint imposed by WiFi's ~20–40 MHz bandwidth is on range *resolution*, not on whether range can
be measured at all. **Two independent verifiers confirmed the range-and-Doppler claim 3–0.**

### 3.3 Ordinary beacons are usable illuminators (Medium confidence)

A recurring practical worry is whether *idle* WiFi — a network carrying little traffic — still emits
enough signal to illuminate a scene. **Pham Duc Su (2021)** addresses exactly this by analyzing *"the
ambiguity function of 802.11b Beacon signal from the actual received data"* and proposing appropriate
correlation processing intervals for a PBR built on beacons alone. Because 802.11 beacon frames are
transmitted periodically regardless of user traffic, this establishes a *guaranteed* illumination
source. The claim is rated **Medium** only because the venue (the *Journal of Military Science &
Technology*, a lightly indexed Vietnamese military journal) is weak for a Q1 citation; the underlying
point is independently corroborated by higher-tier IEEE work (Colone, Falcone et al. on WiFi-based
PBR and tracking), which should be cited alongside it.

### 3.4 The signal-processing problem WiFi creates — and its fix (High confidence)

WiFi is a *poor* radar waveform in two specific ways, and **Yildirim, Griffiths et al. (2021)**,
"Super-resolution passive radars based on 802.11ax Wi-Fi signals for human movement detection" (*IET
Radar, Sonar & Navigation*), names both. First, *"due to the limited integration time of Wi-Fi bursts
and relatively low bandwidths, Fourier-Transform-based methods do not provide the required accuracy."*
Concretely, a DFT-based approach needs on the order of ≥50 ms of coherent integration at 5.6 GHz, while
WiFi bursts last only a few milliseconds. Their answer is **super-resolution (ESPRIT-based) processing**,
which extracts fine delay/Doppler estimates from short, low-bandwidth bursts. For our project this is
directly load-bearing: any automotive WiFi sensor inherits the same short-burst, low-bandwidth handicap,
so a super-resolution front end is likely mandatory rather than optional. **Confirmed 3–0** across two
merged claims.

### 3.5 Operating against unmodified infrastructure (High confidence)

For a deployable system it matters whether the illuminating AP must be special. **Li, Piechocki,
Woodbridge, Tang & Chetty (2021)**, "Passive WiFi Radar for Human Sensing Using a Stand-Alone Access
Point" (*IEEE TGRS*), reports what it calls *"the first work study on stand-alone WiFi AP which has no
specific modification to either the hardware [or] software."* The system exploits either continuous
high-rate OFDM traffic or, when the network is idle, periodic beacon frames — handling the latter with
a *modified* CAF tuned to low-data-rate frames. The important nuance to preserve in the paper: **the
transmitter is unmodified, but the receiver is still a dedicated SDR passive-radar front end.** In
other words, prior art has removed the need to control the illuminator, but not yet the need for
purpose-built reception — which is one of the places an automotive integration must do new work.
**Confirmed 3–0.**

### 3.6 The 2025 state of the art: sub-metre passive tracking (High confidence, preprint)

The most recent point in this thread is **arXiv:2511.22144 (2025)**, "Bistatic Passive Sensing via CSI
Power." It sidesteps the notorious phase-instability of commodity CSI by operating on *phase-independent
CSI power* — a self-conjugate `|CSI|²` operation that *"removes all CSI phase offsets,"* i.e. cancels
the hardware and clock phase errors that plague commodity NICs. A cascaded 3D-FFT recovers
delay/AoA/Doppler, and an Extended Kalman Filter tracks the trajectory, achieving a *"median tracking
error of 0.4 m … [with] computation delay under 2 ms."* Two cautions: (a) this is an un-peer-reviewed
preprint validated in a single indoor lab with a 1-transmitter/3-receiver geometry, so the 0.4 m figure
is indicative, not settled; and (b) an upstream note called the framework "PowerSense" — **the paper
does not use that name**, and repeating it would be an error. **Confirmed 3–0.**

### 3.7 The automotive analogue — proven, but with 5G, not WiFi (High confidence)

The single most important paper for *positioning* this project is **Maksymiuk et al. (2024)**, "5G-based
passive radar on a moving platform — Detection and imaging" (*IET Radar, Sonar & Navigation*, Warsaw
University of Technology). It demonstrates a passive radar receiver **mounted on a moving vehicle** that
uses an *operative 5G base station* as its illuminator of opportunity to perform *moving-target detection
and radar/SAR imaging of the vehicle's surroundings*, and — crucially for credibility — it is *"tested
using both simulated and real-life data"* with a dual-channel reference-plus-surveillance configuration.
This is the load-bearing prior art for our novelty claim, and it cuts both ways: it *proves* the moving-
platform passive-radar concept is real, which de-risks the project; and it *scopes* our novelty precisely,
because it uses 5G rather than WiFi. Our contribution is, in one sentence, *the WiFi-illuminator version
of this, extended toward a SLAM-facing 3D output.* **Confirmed 3–0.**

### 3.8 Thread 1 takeaway

The physics and the signal processing of WiFi passive radar are established and largely solved *for the
indoor, static case*. We inherit a clear processing recipe (super-resolution front end; CAF and modified-
CAF for traffic vs. beacons; CSI-power tracking to defeat phase instability; EKF back-end). The frontier
this project must push is not "does WiFi PBR work" — it does — but "does it survive a moving receiver,
uncontrolled outdoor illumination, and dynamic clutter," which no Thread-1 paper has tested.

---

## 4. Thread 2 — WiFi CSI Sensing, Imaging & 3D Reconstruction

### 4.1 Why this thread matters: proof that WiFi can produce dense output

If Thread 1 supplies the radar analogue, Thread 2 supplies the evidence that WiFi can yield the *dense,
structured, 3D-like* output a SLAM front end wants — not just a blip, but a reconstruction. The
foundational insight, from the MIT CSAIL line of work, is physical: *"wireless signals in the WiFi
frequencies traverse walls and reflect off the human body,"* so the channel carries recoverable
information about the geometry of what it passed through. **Confirmed 3–0.**

### 4.2 From 2D skeletons to 3D: the MIT RF-Pose line (High confidence)

**RF-Pose (Zhao et al., CVPR 2018)** trained a deep network to estimate 2D human skeletons from radio
signals, including *through walls* — using cross-modal supervision from a co-located camera during
training only. **RF-Pose3D** (SIGCOMM 2018, "RF-Based 3D Skeletons") extended this to full **3D**
skeletons with approximately **4 cm** keypoint error. The essential caveat for citation discipline:
these systems use a **purpose-built FMCW radio operating in the WiFi band (≈5.4–7.2 GHz)**, *not*
decoded 802.11 CSI from a commodity NIC. In the paper we must consistently distinguish "WiFi-band FMCW
radar" (RF-Pose) from "commodity 802.11 CSI" (Section 4.3), or a knowledgeable reviewer will flag the
conflation. **Confirmed 3–0.**

### 4.3 Commodity-WiFi 3D pose, comparable to mmWave radar (High confidence)

**Person-in-WiFi 3D (Yan et al., CVPR 2024, Xi'an Jiaotong University)** is the state-of-the-art
demonstration that *commodity* WiFi CSI — from ordinary Intel 5300 NICs — suffices for **end-to-end
multi-person 3D pose estimation**. It attains 3D joint-localization errors of **91.7 mm (1 person),
108.1 mm (2 persons), and 125.3 mm (3 persons)**, which the authors describe as *"comparable to cameras
and millimeter-wave radars."* This is the strongest single piece of evidence that commodity WiFi carries
enough spatial information for dense 3D reconstruction — the premise the whole project rests on.
**Confirmed 3–0.** Its predecessor, **Person-in-WiFi (Wang et al., ICCV 2019, CMU/XJTU)**, was the first
to obtain body segmentation and 2D pose from commodity 802.11n and supplies the cross-environment
generalization numbers that define the gap (Section 4.6).

### 4.4 Depth images from the channel (High confidence, preprint)

**Wi-Depth (arXiv:2503.06458, 2025)** reconstructs **depth images of moving objects** from WiFi CSI
using a VAE-based teacher–student architecture that explicitly decomposes the target into *"shape, depth,
and position"* components. This is a step beyond skeletons toward the kind of dense depth map a mapping
pipeline could consume. Preprint status applies. **Confirmed 3–0.**

### 4.5 Generative reconstruction: CSI into a diffusion latent space (High confidence, preprint)

**LatentCSI (arXiv:2506.10605, 2025)** maps CSI amplitudes *directly into the latent space of a
pretrained latent-diffusion model* via a lightweight network, then uses text-guided denoising to
synthesize high-resolution environment images — explicitly *avoiding GANs and pixel-space generation*.
For our purposes this is intriguing as a route from raw channel data to environment imagery/scene
reconstruction, though it is the most speculative item in the thread and its relevance to metric 3D
mapping (as opposed to plausible imagery) needs scrutiny. **Confirmed 3–0.**

### 4.6 The gap, quantified — the heart of the novelty case (High confidence)

Every system above works, and every system above is *small, indoor, fixed, and brittle across
environments*. This is not editorializing; the authors say so:

- **Person-in-WiFi 3D** is a *"proof-of-concept system in 4 m × 3.5 m areas"* with receivers mounted at
  the corners, and its own limitations section states that adjusting positions, elevations and antenna
  orientations *"is challenging to ensure cross-location generalization."*
- **Wi-Depth**'s *"sensing zone is 2 m × 4 m,"* covering moving objects at a room entrance.
- **Person-in-WiFi (2019)** trained on 14 scenes and tested on 2; in *untrained* environments its
  segmentation collapses to **mIoU 0.12** and pose to **mPCK@0.20 = 19.34**, and even with GAN-based
  domain adaptation only recovers to **0.24 / 31.06** — with the authors conceding that *"further
  improvement on untrained environment remains."*

Taken together, these three self-reported limitations establish the defensible foundation of the whole
project: **mobile, outdoor, automotive WiFi 3D scanning for SLAM is unaddressed, and the cross-
environment generalization that a moving vehicle demands is an openly acknowledged unsolved problem.**
**Confirmed 3–0** across four merged claims. (Guardrail: state the automotive gap as our inference from
this pattern, not as a claim any of these papers make.)

### 4.7 Thread 2 takeaway

WiFi *can* produce dense 3D-like reconstructions from commodity hardware — that premise is settled. The
uniform limitation — small fixed indoor zones and poor generalization — is precisely the research target.
A moving vehicle is the ultimate stress test of cross-environment generalization, because *every frame is
a new environment*. That reframing (ego-motion as continual domain shift) may itself be a framing
contribution of the paper.

---

## 5. Thread 3 — WiFi/RF-based SLAM and Multipath-Assisted Localization

### 5.1 The transferable back-end: multipath as virtual transmitters (High confidence)

The most valuable import from Thread 3 is **Channel-SLAM** (Gentner, Ulmschneider et al., DLR — *IEEE
Transactions on Wireless Communications*, 2016; and *Mobile Information Systems*, 2017). Its key idea is
directly reusable: it performs SLAM with radio signals by treating **each multipath component as a
line-of-sight signal originating from a "virtual transmitter,"** then *jointly estimating the virtual-
transmitter positions together with the receiver's position, velocity, and clock bias* — crucially,
*without any prior map and without knowledge of the reflectors' locations*. For a WiFi automotive system
this is close to ideal, because the whole point of SLAM is that the map is unknown; the virtual-
transmitter formulation converts the very multipath that corrupts communications into the landmarks the
SLAM back end needs. The 2016 formulation uses a Rao-Blackwellized particle filter. **Confirmed 3–0.**

### 5.2 The field is explicitly nascent (High confidence)

**Gounis, Tegos, Tyrovolas, Diamantoulakis & Karagiannidis (2026)**, "When SLAM Meets Wireless
Communications: A Survey" (arXiv:2602.06995, Aristotle University of Thessaloniki), states plainly that
*"integrated solutions performing joint communications and SLAM appear to be in their infancy:
theoretical and practical advancements are required."* This is exactly the kind of "the door is open"
statement a Q1 introduction wants — but with a guardrail (Section 11): the survey does **not** name an
automotive-specific gap. Cite it for the *general* nascency of joint comms-and-SLAM, then build the
automotive argument ourselves. **Confirmed 3–0.**

### 5.3 Thread 3 takeaway

We do not need to invent a SLAM formulation. The virtual-transmitter landmark model is a proven,
citable foundation, and the field's own survey confirms there is room to contribute. The open technical
question — deferred to Section 7 — is whether that formulation, validated for pedestrian/slow motion,
survives a *fast-moving vehicular receiver* and dynamic outdoor multipath. That question is arguably the
project's core research risk.

### 5.4 The modern multipath-SLAM line (verified 2026-07) — strengthens the back-end case

Two further RF-SLAM sources were verified and turn out to matter more than expected: they are the
*current* continuation of the Channel-SLAM idea, from the Graz (Leitinger/Witrisal) and Lund (Tufvesson)
groups, and both are 2024 work with the virtual-anchor formulation front and centre.

- **Leitinger, Wielandner, Venus & Witrisal (2024)**, "Multipath-based SLAM with Cooperation and Map
  Fusion in MIMO Systems" (arXiv:2405.02126). Models specular reflections as **virtual anchors** (mirror
  images of base stations — the same construct as Channel-SLAM's virtual transmitters) and adds
  cooperation and map fusion across multiple mobile terminals for more robust mapping. Shows the
  virtual-anchor framework is actively developed, not a 2016 relic.
- **Li, Cai, Leitinger & Tufvesson (2024)**, "A Belief Propagation Algorithm for Multipath-based SLAM
  with Multiple Map Features: A mmWave MIMO Application" (arXiv:2403.10095). A **factor-graph /
  belief-propagation** MP-SLAM that adapts specular-reflection *and* point-scatterer map features,
  exploits amplitude to detect weak low-SNR features, and — importantly — is **validated on real mmWave
  MIMO measurements.** The mmWave-MIMO real-data validation is directly relevant if the project takes the
  60 GHz fork (Section 6.6).

Two implications. First, the SLAM back-end we would adopt is a *living* research line with modern,
open-source-adjacent tooling, which lowers implementation risk. Second, the belief-propagation MP-SLAM
formulation with amplitude-aided weak-feature detection is a strong candidate estimator for the
low-SNR, cluttered outdoor regime we face.

> Verification note: a third Round-1 source, **arXiv:2601.20547**, does **not exist** — it was a
> malformed/hallucinated identifier from the harness (confirmed non-resolvable, 2026-07) and has been
> dropped. Two adjacent sources surfaced and are worth a later look: arXiv:2404.15375 (MIMO MP-SLAM for
> non-ideal reflective surfaces) and arXiv:2203.08264 (Neural RF-SLAM from CSI with virtual anchors —
> especially relevant, as it is CSI-native).

---

## 6. Thread 4 — ISAC, IEEE 802.11bf and mmWave/60 GHz WiFi  ✅ VERIFIED (Round 2)

This thread answers the resolution question — the *hardware ceiling* of the whole concept — and it does
so decisively. A dedicated Round-2 verification pass (108 agents) settled it.

### 6.1 The decisive result: sub-7 GHz WiFi cannot reach radar-grade range resolution (High)

Range resolution is bandwidth-limited by the fundamental relation **ΔR = c / (2·B)**. There is no signal-
processing trick around it: two scatterers closer than ΔR fall in the same range cell. Plugging in the
bandwidths available to WiFi:

| Waveform / bandwidth | Range resolution ΔR |
|----------------------|---------------------|
| ~10 MHz | ~15 m |
| 40 MHz (commodity 802.11n/ac channel) | **~3.75 m** |
| 160 MHz (802.11ac/ax/be bonded) | **~0.94 m** |
| 1.76 GHz (802.11ad 60 GHz DMG channel) | **~8.5 cm** |
| 77 GHz automotive radar (reference) | ~4–15 cm |

The 802.11bf overview (below) states plainly that *"Sub-7 GHz and DMG sensing have different BW and thus
distinct range resolutions,"* with DMG at *"1.76 GHz per channel (and larger BW … by aggregating multiple
channels)."* Kumari & Heath put it bluntly: sub-6 GHz WLAN *"cannot achieve cm-level range and cm/s-level
velocity resolution … due to insufficiently low bandwidth."* The DMG-to-bonded ratio (1760/160 ≈ **11×**)
is a full order of magnitude — no amount of sub-7 GHz channel bonding closes it; the jump requires mmWave.

**A precision the paper must get right:** the sources report range *accuracy* (single-target estimation
precision, which at high SNR can beat the resolution cell) as distinct from range *resolution* (two-target
separability, which is bandwidth-bound and SNR-independent). The resolution ceiling holds regardless of
SNR. This is, incidentally, the *correct* and defensible form of the intuition that Round 1's killed claim
got wrong: bandwidth limits **resolution**, not the ability to measure range. **Confirmed unanimously.**

### 6.2 60 GHz 802.11ad reused as an automotive long-range radar (High — but theory/simulation)

**Kumari, Choi, González-Prelcic & Heath (2018)**, "IEEE 802.11ad-Based Radar: An Approach to Joint
Vehicular Communication-Radar System" (*IEEE Trans. Vehicular Technology* 67(4):3012–3027; arXiv:1702.05833;
conference precursor IEEE doc 7390996), is the seminal demonstration that the *communications* waveform can
double as a radar. It reuses the 60 GHz single-carrier PHY frame preamble's **Golay complementary
sequences** as the radar waveform and reports **<0.1 m range resolution, <0.6 m/s velocity resolution
(4.2 ms CPI), cm-level range and cm/s-level velocity accuracy, and >99.9% detection at 10⁻⁶ false-alarm to
200 m (43 dBm EIRP)**, all while sustaining Gbps communications — i.e. it *meets long-range-radar specs on a
WiFi-band waveform*. The ~1.76 GHz bandwidth (ΔR ≈ 8.5 cm) is physically consistent with these figures.

> **Citation discipline (non-negotiable for a Q1 venue):** these results are **analytical / Cramér-Rao-
> bound / Monte-Carlo simulation** (10,000 trials), single-target, under idealized full-duplex self-
> interference cancellation and SCNR > 45 dB. They are **not** a hardware field measurement. Cite as
> theory/simulation, never as a demonstrated system. **Confirmed unanimously.**

### 6.3 The same waveform supports radar imaging (ISAR) in V2I (High)

**Han, Choi & Heath (2022)**, "Radar Imaging Based on IEEE 802.11ad Waveform in V2I Communications"
(arXiv:2208.02473), extends the idea from detection to **imaging**: a roadside unit transmits the standard
802.11ad waveform for communication while simultaneously listening to echoes, estimates round-trip delays
from the preamble's Golay-sequence autocorrelation, and forms **high-resolution inverse-synthetic-aperture-
radar (ISAR) images** of vehicles — with no dedicated radar waveform. The Zhang et al. JCAS survey
(arXiv:2102.12780) corroborates that the *"802.11ad preamble in each packet is the main signal exploited
for radar sensing … mainly investigated for vehicular networks,"* RSU- or vehicle-mounted. **Confirmed.**

### 6.4 IEEE 802.11bf standardizes the plumbing, not the sensing outcome (High)

**"An Overview on IEEE 802.11bf: WLAN Sensing"** (*IEEE Communications Surveys & Tutorials*, 2024;
arXiv:2310.17661) is the anchor reference for the standard. The essential nuance for our positioning:
802.11bf standardizes WLAN sensing across **both sub-7 GHz and 60 GHz DMG**, defining the PHY/MAC
measurement-acquisition and feedback procedures — but it **deliberately leaves the sensing algorithms to
vendors**. In other words, the standard guarantees you can *obtain* channel measurements interoperably; it
does *not* hand you a SLAM-ready sensing result. Our algorithmic contribution lives precisely in that
vendor-left-open space, which is a helpful framing for the Introduction. **Confirmed.**

### 6.5 ISAC theory bounds any WiFi-as-radar design (High)

The ISAC literature supplies the theoretical envelope. **Liu et al. (JSAC 2022)** established the dual-
functional shared-hardware/shared-spectrum framing, and **Xiong, Liu et al. (IEEE Trans. Information Theory
2023)** formalized the fundamental **sensing–communication tradeoff** (subspace and deterministic-random
tradeoffs; the P_SC / P_CS Cramér-Rao-bound-versus-rate corner points). These are the limits to cite when
justifying wherever our design chooses to sit on the sensing-vs-communication frontier. **Confirmed.**

### 6.6 Thread 4 takeaway — the central design fork

Round 2 converts a vague worry into a sharp decision. An outdoor automotive **3D-mapping SLAM** system
built on commodity sub-7 GHz CSI is bandwidth-limited to ~1–4 m range resolution — likely too coarse for
fine 3D scene mapping. The paper must therefore choose, explicitly, between two honest paths:

1. **Stay sub-7 GHz and reframe the claim.** Use 802.11ac/ax/be channel bonding (→ ~0.94 m at 160 MHz) as
   partial mitigation, and position the contribution around *coarse* occupancy/structure mapping plus SLAM
   *pose* estimation — where the information lives in **multipath geometry** (à la Channel-SLAM, Thread 3),
   not in fine range. Do **not** claim radar-grade resolution.
2. **Move to 60 GHz / 802.11bf DMG.** Adopt the 802.11ad Golay-preamble radar approach (§6.2–6.3) to claim
   genuine radar-grade (~8.5 cm) resolution — accepting 60 GHz's shorter range, poorer penetration, and far
   sparser ambient illumination (few outdoor 60 GHz APs exist to exploit passively).

This fork is now the project's pivotal design decision and should be stated as such.

---

## 7. Technical feasibility and risk register

The survey does more than establish novelty; it exposes the specific technical risks the project must
retire. These are the open questions, developed into a research agenda.

### 7.1 Ego-motion and dynamic outdoor multipath (highest risk)

Every pipeline surveyed — Thread 1 PBR, Thread 2 CSI, Thread 3 SLAM — was validated on **static indoor
geometry**. A moving vehicle breaks three assumptions simultaneously: the receiver moves (adding
ego-Doppler that must be separated from target Doppler), the scene contains moving clutter (other
vehicles, pedestrians) with no fixed reference geometry, and the illuminator geometry changes continuously.
The pointed sub-question: **is the Channel-SLAM virtual-transmitter formulation robust to a fast-moving
receiver?** This is the project's core technical risk and should be the first thing prototyped.

### 7.2 Range resolution vs. bandwidth (RESOLVED — Round 2)

This is no longer open. As established in Section 6.1, ΔR = c/2B caps commodity sub-7 GHz WiFi at
**~3.75 m (40 MHz)** to **~0.94 m (160 MHz bonded)** — too coarse for fine automotive 3D mapping — while
only **60 GHz mmWave (~8.5 cm)** reaches radar grade. The consequence is the design fork of Section 6.6:
either stay sub-7 GHz and reframe the contribution around multipath-geometry SLAM rather than fine-range
mapping, or move to 60 GHz / 802.11bf DMG. This decision now gates the system architecture and should be
made before hardware commitments. Guardrail retained: coarse *resolution* does not mean *no range*
(Section 11).

### 7.3 Illumination budget under uncontrolled outdoor geometry

The 5G moving-platform demonstration (Maksymiuk 2024) benefited from a controlled, powerful base-station
illuminator. Ambient WiFi APs are sparse outdoors, low-power, and uncontrolled in position. Whether the
received scattered signal is strong enough for reliable detection at automotive standoff distances is a
link-budget modeling question that should be answered analytically before hardware is built.

### 7.4 Phase instability on commodity hardware

Commodity NIC CSI carries hardware and clock phase offsets that corrupt naive processing. The 2025
CSI-power approach (Section 3.6) shows a concrete mitigation — the self-conjugate `|CSI|²` operation —
that we can adopt, at the cost of discarding absolute phase. Whether power-only processing retains enough
information for metric 3D mapping (as opposed to tracking a single target) is an open design question.

---

## 8. Datasets, toolkits and reproducibility

For building and reproducing prior work. Note the through-line: **none of these contain outdoor or
vehicle-mounted captures**, which is itself evidence for the gap and points to a possible secondary
contribution (a novel dataset).

- **CSIKit** — https://github.com/Gi-z/CSIKit — a CSI parsing/processing toolkit spanning Intel 5300,
  Atheros, ESP32, nexmon and PicoScenes formats. The most practical starting point for a capture and
  processing pipeline.
- **Awesome-WiFi-CSI-Sensing** — https://github.com/NTUMARS/Awesome-WiFi-CSI-Sensing — a curated index
  of datasets and models for orientation.
- **"A Survey on CSI-based Wi-Fi Sensing Datasets and Models with a Focus on Reproducibility"** —
  ResearchGate 401244480 — maps the reproducibility landscape and its gaps.
- **"A Taxonomy of WiFi Sensing: CSI vs Passive WiFi Radar"** — ResearchGate 346954384 — provides the
  CSI-vs-PBR conceptual split that organizes Threads 1 and 2 of this very report.
- Classic extraction toolkits and datasets to name-check for completeness: **Intel 5300 CSI Tool, Atheros
  CSI Tool, ESP32-CSI, PicoScenes, Widar3.0**; and, for passive radar, **SDR/USRP-based** front ends.

**Opportunity:** the absence of any public outdoor or vehicle-mounted WiFi CSI / passive-radar dataset
means that collecting one — even a modest pilot — would be a citable asset and a natural second
contribution alongside the method.

---

## 9. How the literature maps onto the proposed system

Reading the four threads as a system blueprint:

- **Reception & illumination (Thread 1):** ambient AP as illuminator; reference + surveillance channels;
  CAF / modified-CAF for traffic vs. beacon frames; super-resolution (ESPRIT) front end to cope with
  short, low-bandwidth bursts.
- **Signal conditioning (Threads 1 & 2):** self-conjugate CSI-power to defeat commodity phase instability;
  cascaded FFT for delay/AoA/Doppler.
- **Scene reconstruction (Thread 2):** learned mapping from conditioned channel data to a dense 3D / depth
  representation — the automotive analogue of Person-in-WiFi 3D / Wi-Depth, but re-posed for outdoor,
  per-frame-novel environments.
- **SLAM back-end (Thread 3):** Channel-SLAM virtual-transmitter landmarks feeding a particle-filter /
  EKF estimator of vehicle pose and map.
- **Resolution enabler (Thread 4, resolved):** commodity sub-7 GHz resolution *is* insufficient for fine
  mapping (Section 6.1) — so either accept coarse/multipath-geometry mapping, or adopt 60 GHz / 802.11bf
  DMG (802.11ad Golay-preamble radar) for radar-grade resolution.

The paper's contribution lives in the *seams* between these blocks under ego-motion — precisely where no
prior work has operated.

---

## 10. Recommended next actions

1. **Resolve the design fork (Section 6.6)** — decide sub-7 GHz-coarse-SLAM vs. 60 GHz-radar-grade before
   any hardware commitment. This is now the pivotal decision; the resolution question that gated it is
   answered (Section 6). *(Round-2 Thread-4 research: ✅ done.)*
2. **Author-email extraction pass** for the papers we intend to contact — several 2025–2026 arXiv
   preprints (2511.22144, 2506.10605) and the JMST beacon paper did not yield corresponding-author
   emails in this sweep; these live in the PDF author blocks. (A Gmail connector is available for
   drafting the outreach once addresses are confirmed.)
3. ~~**Verify the remaining RF-SLAM sources**~~ ✅ done (2026-07): arXiv:2405.02126 and 2403.10095 verified
   and added (modern virtual-anchor MP-SLAM, Section 5.4); arXiv:2601.20547 does not exist (dropped). ISAC
   citations (Liu 2022, Xiong 2023) vol/pages confirmed. Still open: confirm arXiv:2302.08378 (ISAC/mmWave)
   and the adjacent arXiv:2404.15375 / 2203.08264 if wanted.
4. **Prototype the core risk first:** test whether Channel-SLAM's virtual-transmitter model survives a
   fast-moving receiver (Section 7.1), and settle the range-resolution ceiling (Section 7.2), before
   committing to a hardware path.

---

## 11. Anti-patterns: claims that FAILED verification (keep out of the paper)

Both were adversarially **refuted 0–3** and must not appear as assertions:

1. ❌ **"WiFi's 20–40 MHz bandwidth means passive WiFi radar can extract only Doppler, not range."**
   *False.* Cross-ambiguity processing yields full range–Doppler surfaces (demonstrated as early as
   Chetty 2012). Bandwidth constrains range *resolution*, not the ability to measure range. If the paper
   discusses bandwidth, frame it as a resolution limitation only.

2. ❌ **"The comms-meets-SLAM survey identifies a specific automotive/vehicular gap."**
   *It does not.* The automotive gap is *our* inference from the pervasive indoor/static pattern across
   Threads 1–2. Argue it ourselves; do not attribute it to arXiv:2602.06995.

---

## 12. Global epistemic caveats

- **2025–2026 preprints** (arXiv:2511.22144, 2503.06458, 2506.10605, 2602.06995) are self-reported and
  un-peer-reviewed; treat their numbers as indicative of feasibility and direction, not as validated
  results.
- **Naming error:** the CSI-power framework (arXiv:2511.22144) is **not** called "PowerSense" in the
  paper; do not use that label.
- **Frequency distinction:** RF-Pose / RF-Pose3D use a purpose-built **FMCW radio in the WiFi band**, not
  decoded 802.11 CSI. Keep "WiFi-band FMCW radar" and "commodity 802.11 CSI" separate.
- **Automotive analogue uses 5G, not WiFi** (Maksymiuk 2024) — this is the load-bearing prior art and the
  precise scope of our novelty.
- **Weakest source:** the JMST beacon-ambiguity paper (lightly indexed venue) — hence Medium confidence;
  pair with a higher-tier IEEE citation.
- **Contact completeness:** corresponding-author emails were recovered for the UCL, MIT, DLR and AUTh
  groups but not reliably for several arXiv preprints and the JMST paper — confirm from each PDF's own
  metadata before outreach.
- **Thread 4 now sourced (Round 2):** the ISAC / 802.11bf / mmWave sub-section is verified (Section 6).
  Its key result — the sub-7 GHz resolution ceiling — reframes the killed Round-1 bandwidth claim into a
  correct form (bandwidth limits *resolution*, not range). The 802.11ad-radar figures are theory/simulation,
  not hardware.

---

## 13. Annotated bibliography

Legend: 🟢 open-access / downloadable · 🔴 paywalled (author copy noted where known) · ✉️ contact.

### Thread 1 — Passive WiFi radar

**[R1] Chetty, K., Smith, G. & Woodbridge, K. (2012).** *Through-the-Wall Sensing of Personnel Using
Passive Bistatic WiFi Radar at Standoff Distances.* IEEE Trans. Geoscience & Remote Sensing, 50(4).
Foundational, most-cited demonstration; first TTW detection of moving personnel; range + Doppler,
Doppler matches bistatic theory. 🔴 IEEE Xplore (doc 6020778); 🟢 author copy usually at
discovery.ucl.ac.uk. https://ieeexplore.ieee.org/document/6020778
✉️ **Kevin Chetty, University College London (Dept. of Security & Crime Science)** — `k.chetty@ucl.ac.uk`

**[R2] Pham Duc Su (2021).** *Ambiguity Function Analysis of WiFi Beacon Transmissions for Passive
Bistatic Radar.* J. Military Science & Technology (JMST), Issue 71. Establishes ordinary 802.11b beacons
as viable illuminators; analyzes beacon ambiguity function from real data. 🟢 open-access.
https://online.jmst.info/index.php/jmst/article/view/97
✉️ Military Technical Academy (Vietnam); corresponding email not on landing page — via journal editorial
page. *Pair with a higher-tier IEEE source (Colone/Falcone) when citing.*

**[R3] Yildirim, A., Griffiths, H. et al. (2021).** *Super-resolution passive radars based on 802.11ax
Wi-Fi signals for human movement detection.* IET Radar, Sonar & Navigation, 15(4):323–339.
DOI 10.1049/rsn2.12038. Diagnoses WiFi's short-integration/low-bandwidth handicap; proposes ESPRIT-based
super-resolution. 🟢 open-access (IET/Wiley full text).
https://ietresearch.onlinelibrary.wiley.com/doi/full/10.1049/rsn2.12038
✉️ **Hugh Griffiths, University College London** — `h.griffiths@ucl.ac.uk`

**[R4] Li, W., Piechocki, R., Woodbridge, K., Tang, C. & Chetty, K. (2021).** *Passive WiFi Radar for
Human Sensing Using a Stand-Alone Access Point.* IEEE Trans. Geoscience & Remote Sensing,
59(3):1986–1998. DOI 10.1109/TGRS.2020.3006387. First PBR against an unmodified stand-alone AP; CAF +
modified CAF for idle beacons. 🔴 IEEE; 🟢 author PDF: https://discovery.ucl.ac.uk/id/eprint/10103371
✉️ **Wenda Li** (now University of Dundee) & **Kevin Chetty (UCL)** — `k.chetty@ucl.ac.uk`

**[R5] Wang, Zhang, Wu, Chen, Xu & Guo (2025).** *Bistatic Passive Sensing via CSI Power.* arXiv:2511.22144.
Phase-independent CSI-power tracking (self-conjugate |CSI|²) + cascaded 3D-FFT + EKF; ~0.4 m median
indoor error, <2 ms latency. 🟢 open-access (arXiv). https://arxiv.org/abs/2511.22144
✉️ corresponding email in arXiv metadata — **confirm before contacting.** *Preprint; NOT "PowerSense."*

### Automotive analogue (load-bearing prior art)

**[R6] Maksymiuk, R. et al. (2024).** *5G-based passive radar on a moving platform — Detection and
imaging.* IET Radar, Sonar & Navigation. DOI 10.1049/rsn2.12559. Warsaw University of Technology.
Passive radar on a moving vehicle using an operative 5G base station; moving-target detection + imaging
of surroundings; tested on simulated **and** real data. 🔴 IET; 🟢 author PDF at repo.pw.edu.pl.
https://ietresearch.onlinelibrary.wiley.com/doi/10.1049/rsn2.12559
✉️ WUT Passive Bistatic Radar group (**Krzysztof Kulpa** group) — corresponding email on article page.

### Thread 2 — CSI sensing / imaging / 3D

**[R7] Zhao, M. et al. (2018).** *Through-Wall Human Pose Estimation Using Radio Signals* (RF-Pose,
CVPR 2018) and *RF-Based 3D Skeletons* (RF-Pose3D, ACM SIGCOMM 2018, DOI 10.1145/3230543.3230579).
MIT CSAIL. 2D→3D skeletons, ~4 cm keypoint error. *Purpose-built WiFi-band FMCW radio, not 802.11 CSI.*
🟢 open-access (CVF, MIT DSpace). https://rfpose.csail.mit.edu/
✉️ **Dina Katabi, MIT** — `dk@mit.edu`

**[R8] Yan, Y. et al. (2024).** *Person-in-WiFi 3D: End-to-End Multi-Person 3D Pose Estimation with
Wi-Fi.* CVPR 2024. Xi'an Jiaotong University. Commodity Intel 5300 NICs; 91.7/108.1/125.3 mm joint error
for 1/2/3 persons. 🟢 open-access.
https://openaccess.thecvf.com/content/CVPR2024/papers/Yan_Person-in-WiFi_3D_End-to-End_Multi-Person_3D_Pose_Estimation_with_Wi-Fi_CVPR_2024_paper.pdf

**[R9] Wang, F., Zhou, S., Panev, S., Han, J. & Huang, D. (2019).** *Person-in-WiFi: Fine-grained Person
Perception using WiFi.* ICCV 2019. CMU / XJTU. First commodity-802.11n body segmentation + 2D pose;
source of the cross-environment generalization numbers. 🟢 open-access (CVF / arXiv:1904.00276).
https://www.ri.cmu.edu/app/uploads/2019/09/Person_in_WiFi_ICCV2019.pdf
✉️ **Dong Huang (CMU)** / **Jinsong Han (XJTU)**

**[R10] Wi-Depth (2025).** *Depth reconstruction of moving objects from WiFi CSI* (VAE teacher–student).
arXiv:2503.06458. 🟢 open-access. https://arxiv.org/abs/2503.06458 ✉️ arXiv metadata (confirm).

**[R11] LatentCSI (2025).** *CSI-to-image synthesis via a pretrained latent-diffusion model.*
arXiv:2506.10605. 🟢 open-access. https://arxiv.org/abs/2506.10605 ✉️ arXiv metadata (confirm).

### Thread 3 — RF-SLAM

**[R12] Gentner, C. et al. (2016).** *Multipath-Assisted Positioning with Simultaneous Localization and
Mapping* (Channel-SLAM). IEEE Trans. Wireless Communications. Rao-Blackwellized particle filter;
multipath-as-virtual-transmitter. 🔴 IEEE.
**[R12b] Ulmschneider, M., Gentner, C. et al. (2017).** *Multipath Assisted Positioning for Pedestrians
using LTE Signals.* Mobile Information Systems 2017, Art. 9170746. 🟢 open-access.
https://www.hindawi.com/journals/misy/2017/9170746/
✉️ **Christian Gentner, DLR Institute of Communications and Navigation** — `christian.gentner@dlr.de`

**[R13] Gounis, Tegos, Tyrovolas, Diamantoulakis & Karagiannidis (2026).** *When SLAM Meets Wireless
Communications: A Survey.* arXiv:2602.06995. Aristotle University of Thessaloniki. Confirms joint
comms+SLAM is "in its infancy." 🟢 open-access. https://arxiv.org/abs/2602.06995
✉️ **George K. Karagiannidis, AUTh** — `geokarag@auth.gr`

**[R13a] Leitinger, Wielandner, Venus & Witrisal (2024).** *Multipath-based SLAM with Cooperation and
Map Fusion in MIMO Systems.* arXiv:2405.02126. TU Graz. Virtual-anchor (= virtual-transmitter) MP-SLAM
with cooperation + map fusion. 🟢 https://arxiv.org/abs/2405.02126 ✉️ Erik Leitinger / Klaus Witrisal,
TU Graz SPSC Lab (confirm @tugraz.at).

**[R13b] Li, Cai, Leitinger & Tufvesson (2024).** *A Belief Propagation Algorithm for Multipath-based
SLAM with Multiple Map Features: A mmWave MIMO Application.* arXiv:2403.10095. Lund / TU Graz. Factor-graph
BP MP-SLAM, amplitude-aided weak-feature detection, **validated on real mmWave MIMO measurements.**
🟢 https://arxiv.org/abs/2403.10095 ✉️ Fredrik Tufvesson, Lund (@eit.lth.se). *(Adjacent, unverified:
arXiv:2404.15375, arXiv:2203.08264. Note: Round-1's arXiv:2601.20547 does not exist — dropped.)*

### Thread 4 — ISAC / 802.11bf / mmWave  ✅ verified (Round 2)

**[R14]** *An Overview on IEEE 802.11bf: WLAN Sensing.* IEEE Communications Surveys & Tutorials, 2024.
arXiv:2310.17661. Standardizes sensing across sub-7 GHz + 60 GHz DMG; leaves algorithms to vendors.
🟢 https://arxiv.org/abs/2310.17661 ✉️ Francesca Meneghello / Rui Du group (confirm from PDF).

**[R15] Kumari, P., Choi, J., González-Prelcic, N. & Heath, R. W. (2018).** *IEEE 802.11ad-Based Radar:
An Approach to Joint Vehicular Communication-Radar System.* IEEE Trans. Vehicular Technology,
67(4):3012–3027. Golay-preamble radar; <0.1 m range res; **theory/simulation only.**
🟢 arXiv:1702.05833 · 🔴 IEEE. https://arxiv.org/abs/1702.05833
✉️ **Robert W. Heath Jr., UT Austin / NC State** — `rheath@utexas.edu` (strongest new outreach target).

**[R16] Han, Choi & Heath (2022).** *Radar Imaging Based on IEEE 802.11ad Waveform in V2I Communications.*
arXiv:2208.02473. ISAR imaging from the comms waveform. 🟢 https://arxiv.org/abs/2208.02473 ✉️ Heath group.

**[R17] Zhang et al.** *JCAS survey.* arXiv:2102.12780. 🟢 https://arxiv.org/abs/2102.12780

**[R18] Liu, Cui, Masouros, Xu, Han, Eldar & Buzzi (2022)**, *Integrated Sensing and Communications:
Toward Dual-Functional Wireless Networks for 6G and Beyond*, **IEEE JSAC 40(6):1728–1767**, DOI
10.1109/JSAC.2022.3156632; and **Xiong, Liu, Cui, Yuan, Han & Caire (2023)**, *On the Fundamental
Tradeoff of ISAC under Gaussian Channels*, **IEEE Trans. Information Theory 69(9):5723–5751**, DOI
10.1109/TIT.2023.3284449. Sensing–communication tradeoff / CRB-rate region. *(vol/pages confirmed 2026-07.)*

### Datasets / toolkits / reproducibility

**[R18]** CSIKit — https://github.com/Gi-z/CSIKit (maintainer: Gi-z).
**[R19]** Awesome-WiFi-CSI-Sensing — https://github.com/NTUMARS/Awesome-WiFi-CSI-Sensing.
**[R20]** *A Survey on CSI-based Wi-Fi Sensing Datasets and Models with a Focus on Reproducibility* —
ResearchGate 401244480.
**[R21]** *A Taxonomy of WiFi Sensing: CSI vs Passive WiFi Radar* — ResearchGate 346954384.

### Verification status of previously-pending sources (as of 2026-07)

- RF-SLAM: arXiv:2405.02126 ✅ verified → [R13a]; arXiv:2403.10095 ✅ verified → [R13b];
  **arXiv:2601.20547 ✗ does not exist (malformed ID, dropped).**
- ISAC/mmWave: arXiv:1702.05833 ✅ → [R15]; arXiv:2102.12780 ✅ → [R17]; arXiv:2302.08378 — still to confirm.
- Adjacent, surfaced during verification, not yet claim-verified: arXiv:2404.15375 (MIMO MP-SLAM,
  non-ideal surfaces); arXiv:2203.08264 (Neural RF-SLAM from CSI — CSI-native, high relevance).

---

*End of report. Companion condensed version: `00-literature-foundation.md`. Generated from a
verified deep-research run (106 agents; 23/25 claims confirmed under adversarial verification).*
