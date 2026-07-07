# WiFi-as-Radar for Automotive SLAM — Literature Foundation (Round 1)

**Project:** A WiFi-based radar-replacement system that exploits ambient WiFi signals in the
environment **plus an on-car WiFi antenna** to build a **3D scan of the surroundings**, usable as
an in-place replacement for radar in **SLAM** pipelines.

**Purpose of this doc:** Verified related-work foundation for a Q1 paper. Every load-bearing claim
below was adversarially fact-checked (3-vote verification; a claim needed 2/3 refutations to be
killed). Two claims were **killed** and are listed at the end so we do not repeat them in the paper.

**Method:** Deep-research harness — 5 search angles → 24 sources fetched → 107 claims extracted →
top 25 verified → 23 confirmed / 2 refuted. Dated 2026-07-03.

> ✅ **Update (Round 2, 2026-07-04):** Thread 4 (ISAC / IEEE 802.11bf / mmWave-60 GHz) — a hole in
> Round 1 — is now verified (108-agent pass). **Key result:** commodity sub-7 GHz WiFi is bandwidth-
> limited (ΔR = c/2B: ~3.75 m @40 MHz, ~0.94 m @160 MHz) and cannot reach radar grade; only 60 GHz
> mmWave (~8.5 cm) can. Full findings in `02-round2-isac-80211bf-mmwave.md`; summarized in Thread 4 below.

---

## The headline: the novelty gap is real and defensible

The literature strongly supports all four related-work threads, and — more importantly — it exposes
a clean, defensible gap:

- **Virtually all WiFi/CSI sensing and passive-WiFi-radar work to date is indoor, static, and
  infrastructure-fixed** (fixed AP, fixed receiver, few-metre sensing zone, poor cross-environment
  generalization).
- **The one demonstrated on-vehicle, moving-platform passive radar uses 5G downlink — not WiFi — as
  its illuminator** (Maksymiuk et al., 2024).
- **Joint communications + SLAM is explicitly "in its infancy"** (2026 survey).

> **Defensible contribution statement (draft):** *An on-vehicle, mobile, outdoor WiFi (802.11)
> passive-radar / CSI system that produces 3D scans usable in a SLAM pipeline* — i.e. combining the
> **automotive moving-platform concept** (so far shown only with 5G) with **WiFi-band sensing** (so
> far shown only indoors/static). No prior work occupies that intersection.

**Guardrail (from the two refuted claims):** frame the automotive gap as *our inference from the
indoor/static pattern*, **not** as something prior work explicitly states. And do **not** claim WiFi's
low bandwidth restricts it to Doppler-only / no-range — that is false (cross-ambiguity gives full
range–Doppler surfaces). See "Killed claims."

---

## Thread 1 — Passive WiFi radar (mature, foundational)

The classical-radar analogue: ambient WiFi APs as illuminators of opportunity, a reference + surveillance
channel, cross-ambiguity processing for range/Doppler. This thread is **mature** and gives us solid
footing.

| # | Finding | Confidence |
|---|---------|-----------|
| 1.1 | **First through-wall detection of moving personnel** with passive bistatic WiFi radar; extracts **both range and Doppler**, Doppler matches bistatic theory. (Chetty, Smith & Woodbridge, 2012 — UCL) | High |
| 1.2 | **Ordinary 802.11b WiFi beacons are viable illuminators of opportunity**; beacon ambiguity function analyzable from real data to set correlation processing intervals. (Pham Duc Su, 2021) | Medium* |
| 1.3 | **Standard Fourier processing is inadequate** for WiFi PBR — short burst integration time + low bandwidth — motivating **super-resolution (ESPRIT)** processing; ambient 802.11ax works as privacy-preserving indoor PBR. (Yildirim, Griffiths et al., 2021 — UCL) | High |
| 1.4 | **Passive WiFi radar can run against an unmodified stand-alone AP** (no antenna/firmware swap) via standard CAF + a modified CAF for low-rate beacon frames — claimed as a first. *Transmitter unmodified, but receiver is still a dedicated SDR front end.* (Li, Chetty et al., 2021 — UCL/Bristol) | High |
| 1.5 | **2025 bistatic passive sensing via CSI power** — phase-independent `|CSI|²` (self-conjugate, removes clock/hardware phase), cascaded 3D-FFT (delay/AoA/Doppler) + EKF, **~0.4 m median indoor tracking error, <2 ms latency.** (arXiv:2511.22144) | High** |

\* *Medium only because the venue (JMST, Vietnamese military journal) is lightly indexed; the point is
corroborated by higher-tier IEEE work (Colone/Falcone). Cite a stronger IEEE source alongside it.*
\*\* *Un-peer-reviewed preprint, single indoor lab (1Tx–3Rx), self-reported. **Naming note:** it is
NOT called "PowerSense" — that label was an upstream error.*

**What this buys us:** the physics and signal processing of WiFi PBR are established. Our processing
chain (super-resolution for low bandwidth, CAF variants, CSI-power tracking) has direct precedent —
all indoor/static, which is exactly the point of departure.

---

## Thread 2 — WiFi CSI sensing, imaging & 3D reconstruction

Dense reconstruction from WiFi is feasible — **but every system is a few-metre fixed indoor zone with
poor cross-environment generalization.** This thread simultaneously proves feasibility *and* defines
the gap.

| # | Finding | Confidence |
|---|---------|-----------|
| 2.1 | **WiFi-band signals pass through walls & reflect off bodies; deep nets reconstruct dense human structure.** RF-Pose → 2D through-wall skeletons; RF-Pose3D → 3D skeletons (~4 cm keypoint error). *Uses a purpose-built FMCW radio in the WiFi band — not decoded 802.11 CSI.* (MIT CSAIL, Katabi group, 2018) | High |
| 2.2 | **Commodity-WiFi CSI reaches 3D multi-person pose.** Person-in-WiFi-3D: 91.7 / 108.1 / 125.3 mm joint error for 1/2/3 persons, "comparable to cameras and mmWave radar." (CVPR 2024, Xi'an Jiaotong) | High |
| 2.3 | **Depth-image reconstruction from CSI.** Wi-Depth — VAE teacher-student decomposing shape/depth/position. (arXiv:2503.06458, 2025) | High |
| 2.4 | **CSI → latent-diffusion imagery.** LatentCSI maps CSI amplitudes into a pretrained LDM latent space with text-guided denoising to synthesize high-res environment images (no GANs / pixel-space). (arXiv:2506.10605, 2025) | High |
| 2.5 | **THE GAP, quantified.** Person-in-WiFi-3D = 4 m × 3.5 m corner rig, flags cross-location reconfiguration as unsolved; Wi-Depth = 2 m × 4 m entrance zone; Person-in-WiFi (2019) drops to mIoU 0.12 / mPCK 19.34 in untrained scenes (only 0.24 / 31.06 even with GAN domain adaptation). **No vehicular / mobile / outdoor deployment exists.** | High |

**What this buys us:** proof that WiFi can yield dense 3D-like output → our "3D scan" ambition is not
science fiction. The *limitation* (fixed small indoor zone, poor generalization) is precisely our
research target.

> ⚠️ **Terminology discipline for the paper:** distinguish **"WiFi-band FMCW radar"** (RF-Pose,
> purpose-built radio) from **"commodity 802.11 CSI"** (Person-in-WiFi, Intel 5300 NICs). Reviewers
> will catch conflation.

---

## Thread 3 — WiFi/RF-based SLAM & multipath-assisted localization

The back-end math we need already exists and is transferable to WiFi.

| # | Finding | Confidence |
|---|---------|-----------|
| 3.1 | **Channel-SLAM: multipath-as-virtual-transmitter.** Treats each multipath component as an LoS signal from a "virtual transmitter," jointly estimating VT positions + receiver position/velocity/clock bias — **no prior map, no known reflector locations.** Directly reusable as a WiFi automotive-SLAM back-end. (Gentner/Ulmschneider et al., DLR — 2016 IEEE TWC + 2017 Hindawi) | High |
| 3.2 | **Joint comms + SLAM is explicitly "in its infancy"** — theoretical & practical advances required. Confirms an open gap an RF/WiFi-SLAM contribution can address. (Gounis, Karagiannidis et al., "When SLAM Meets Wireless Communications: A Survey," arXiv:2602.06995, 2026) | High |

**What this buys us:** we do not need to invent a SLAM formulation from scratch — the virtual-transmitter
landmark model is a proven, citable foundation. Open question 3.4 (below) is whether it survives a
fast-moving receiver.

> ⚠️ **Guardrail:** the 2026 survey does **not** name an automotive gap (a claim asserting it did was
> *refuted*). Cite it for the *nascency of comms+SLAM generally*, then argue the automotive-specific
> gap ourselves.

---

## Thread 4 — ISAC / IEEE 802.11bf / mmWave-60 GHz WiFi imaging  ✅ VERIFIED (Round 2)

**Decisive result — answers the project's core feasibility question.** Range resolution is bandwidth-
limited (ΔR = c/2B): commodity sub-7 GHz WiFi reaches only **~3.75 m (40 MHz)** to **~0.94 m (160 MHz
bonded)**; radar grade (**~8.5 cm**) needs **60 GHz mmWave (802.11ad DMG, 1.76 GHz)** — an ~11× gap.

| # | Finding | Confidence |
|---|---------|-----------|
| 4.1 | **Sub-7 GHz WiFi cannot reach radar-grade range resolution** (ΔR = c/2B); only 60 GHz mmWave can. (Kumari/Heath; 802.11bf overview) | High |
| 4.2 | **802.11ad 60 GHz reused as a long-range radar** via preamble Golay sequences: <0.1 m range res, <0.6 m/s velocity res, >99.9% detection to 200 m + Gbps comms. *Theory/simulation, not hardware.* (Kumari, Choi, González-Prelcic & Heath, TVT 2018) | High |
| 4.3 | **Same waveform does radar IMAGING (ISAR)** in V2I from echoes. (Han, Choi & Heath, 2022) | High |
| 4.4 | **802.11bf standardizes the measurement plumbing, not the sensing outcome** — algorithms left to vendors (our contribution's home). (ComST 2024) | High |
| 4.5 | **ISAC theory bounds the design** — dual-functional shared hardware/spectrum + sensing–communication tradeoff (CRB-rate region). (Liu 2022; Xiong/Liu 2023) | High |

**Central design fork this forces:** (1) **stay sub-7 GHz** + reframe as coarse occupancy / SLAM-pose
mapping via multipath geometry (cf. Channel-SLAM), not fine range; or (2) **move to 60 GHz / 802.11bf
DMG** for radar-grade resolution, accepting 60 GHz range/penetration limits and sparse ambient
illumination. State this explicitly in the paper. Full detail: `02-round2-isac-80211bf-mmwave.md`.

---

## Datasets, toolkits & reproducibility

For building/reproducing prior work (all **indoor** — none contain outdoor or vehicle-mounted captures,
which reinforces the gap):

- **CSIKit** — https://github.com/Gi-z/CSIKit — CSI parsing/processing toolkit (Intel 5300, Atheros,
  ESP32, nexmon, PicoScenes). Practical starting point for a capture pipeline.
- **Awesome-WiFi-CSI-Sensing** — https://github.com/NTUMARS/Awesome-WiFi-CSI-Sensing — curated index
  of datasets/models.
- **"A Survey on CSI-based Wi-Fi Sensing Datasets and Models with a Focus on Reproducibility"** —
  ResearchGate 401244480 — reproducibility landscape.
- **"A Taxonomy of WiFi Sensing: CSI vs Passive WiFi Radar"** — ResearchGate 346954384 — frames the
  CSI-vs-PBR distinction that structures our Threads 1 & 2.
- Classic extraction toolkits to name-check: **Intel 5300 CSI Tool, Atheros CSI Tool, ESP32-CSI,
  PicoScenes, Widar3.0**; SDR passive-radar setups on **USRP**.

> **Open dataset gap = a possible secondary contribution:** there is no public outdoor/vehicle-mounted
> WiFi CSI or passive-radar dataset. Collecting one could be a paper asset in its own right.

---

## Open questions → these frame the paper's technical contribution

1. ~~**Thread 4 sourcing**~~ ✅ done (Round 2) — see Thread 4 above.
2. ~~**Range resolution**~~ ✅ **ANSWERED (Round 2):** commodity sub-7 GHz CSI (~0.94–3.75 m) cannot reach
   radar grade; only 60 GHz mmWave (~8.5 cm) can. → **new pivotal question: which side of the design fork
   does the project take** (sub-7 GHz-coarse-SLAM vs. 60 GHz-radar-grade)?
3. **Ego-motion & dynamic outdoor multipath** — every pipeline above was validated on static indoor
   geometry. How do vehicle ego-motion + moving outdoor clutter (no fixed reference geometry) affect
   PBR and CSI processing? **Is the Channel-SLAM virtual-transmitter formulation robust to a
   fast-moving receiver?** ← likely the core technical risk of the whole project.
4. **Illuminator geometry** — the 5G moving-platform work had a controlled base-station geometry; ambient
   WiFi APs are uncontrolled, sparse outdoors, and low-power. Feasibility of the illumination budget is
   an open modeling question.

---

## Killed claims — DO NOT put these in the paper

Both were adversarially **refuted (0–3)**:

1. ❌ *"WiFi's 20–40 MHz bandwidth means passive WiFi radar can extract only Doppler, not range."*
   **False** — cross-ambiguity processing yields full range–Doppler surfaces. (Bandwidth limits range
   *resolution*, not range *extraction*.)
2. ❌ *"The comms-meets-SLAM survey identifies a specific automotive/vehicular gap."*
   **It does not.** The automotive gap is *our* inference — argue it ourselves.

---

## Global caveats

- **2025–2026 preprints** (arXiv:2511.22144, 2503.06458, 2506.10605, 2602.06995) are self-reported and
  un-peer-reviewed — treat numbers as indicative.
- **"PowerSense" naming error** — the CSI-power framework (2511.22144) is not named that in the paper.
- **RF-Pose frequency caveat** — WiFi-band FMCW radio, not decoded 802.11 CSI.
- **The only moving-vehicle passive radar uses 5G**, not WiFi (this is the load-bearing prior art).
- **Author emails**: recovered for UCL / MIT / DLR / AUTh groups; **not reliably extracted** for several
  arXiv preprints (2511.22144, 2506.10605) and the JMST paper — confirm from PDF metadata before
  contacting (see reference section).

---

# Reference section (with links, access status & author contacts)

Legend: 🟢 open-access/downloadable · 🔴 paywalled (author copy noted where known) · ✉️ contact.

### Thread 1 — Passive WiFi radar

**[R1] Chetty, Smith & Woodbridge (2012)** — "Through-the-Wall Sensing of Personnel Using Passive
Bistatic WiFi Radar at Standoff Distances," *IEEE Trans. Geoscience & Remote Sensing* 50(4).
🔴 IEEE Xplore (doc 6020778) — 🟢 author copy usually at discovery.ucl.ac.uk.
Link: https://ieeexplore.ieee.org/document/6020778
✉️ **Kevin Chetty, University College London (Security & Crime Science)** — `k.chetty@ucl.ac.uk`

**[R2] Pham Duc Su (2021)** — "Ambiguity Function Analysis of WiFi Beacon Transmissions for Passive
Bistatic Radar," *J. Military Science & Technology (JMST)* Issue 71.
🟢 open-access. Link: https://online.jmst.info/index.php/jmst/article/view/97
✉️ Military Technical Academy (Vietnam) — corresponding email not on landing page; via journal editorial page.
*(Pair with a higher-tier IEEE source, e.g. Colone/Falcone WiFi-PBR, when citing.)*

**[R3] Yildirim, Griffiths et al. (2021)** — "Super resolution passive radars based on 802.11ax Wi-Fi
signals for human movement detection," *IET Radar, Sonar & Navigation* 15(4):323–339, DOI 10.1049/rsn2.12038.
🟢 open-access (IET/Wiley full text). Link: https://ietresearch.onlinelibrary.wiley.com/doi/full/10.1049/rsn2.12038
✉️ **Hugh Griffiths, UCL** — `h.griffiths@ucl.ac.uk`

**[R4] Li, Piechocki, Woodbridge, Tang & Chetty (2021)** — "Passive WiFi Radar for Human Sensing Using
a Stand-Alone Access Point," *IEEE Trans. Geoscience & Remote Sensing* 59(3):1986–1998, DOI 10.1109/TGRS.2020.3006387.
🔴 IEEE — 🟢 author PDF: https://discovery.ucl.ac.uk/id/eprint/10103371
✉️ **Wenda Li** (now Univ. of Dundee) & **Kevin Chetty (UCL)** — `k.chetty@ucl.ac.uk`

**[R5] Wang, Zhang, Wu, Chen, Xu, Guo (2025)** — "Bistatic Passive Sensing via CSI Power," arXiv:2511.22144.
🟢 open-access (arXiv). Link: https://arxiv.org/abs/2511.22144
✉️ corresponding email in arXiv metadata — **confirm before contacting.** *(Preprint; not "PowerSense.")*

### Automotive analogue (load-bearing prior art)

**[R6] Maksymiuk et al. (2024)** — "5G-based passive radar on a moving platform — Detection and imaging,"
*IET Radar, Sonar & Navigation*, DOI 10.1049/rsn2.12559 (Warsaw Univ. of Technology).
🔴 IET — 🟢 author PDF at repo.pw.edu.pl. Link: https://ietresearch.onlinelibrary.wiley.com/doi/10.1049/rsn2.12559
✉️ WUT Passive Bistatic Radar group (**Krzysztof Kulpa** group) — corresponding email on article page.
*This is the single most important paper for positioning novelty: moving-vehicle passive radar proven, but with 5G, not WiFi.*

### Thread 2 — CSI sensing / imaging / 3D

**[R7] Zhao et al. (2018)** — RF-Pose (CVPR 2018) + **RF-Pose3D "RF-Based 3D Skeletons"** (ACM SIGCOMM
2018, DOI 10.1145/3230543.3230579), MIT CSAIL.
🟢 open-access (CVF, MIT DSpace). Project: https://rfpose.csail.mit.edu/
✉️ **Dina Katabi, MIT** — `dk@mit.edu`  *(WiFi-band FMCW radio, not 802.11 CSI.)*

**[R8] Yan et al. (2024)** — "Person-in-WiFi 3D: End-to-End Multi-Person 3D Pose Estimation with Wi-Fi,"
CVPR 2024 (Xi'an Jiaotong Univ.). Commodity Intel 5300 NICs.
🟢 open-access. Link: https://openaccess.thecvf.com/content/CVPR2024/papers/Yan_Person-in-WiFi_3D_End-to-End_Multi-Person_3D_Pose_Estimation_with_Wi-Fi_CVPR_2024_paper.pdf

**[R9] Wang, Zhou, Panev, Han, Huang (2019)** — "Person-in-WiFi," ICCV 2019 (CMU / XJTU). First
commodity-802.11n body-segmentation + 2D pose.
🟢 open-access (CVF / arXiv:1904.00276). Link: https://www.ri.cmu.edu/app/uploads/2019/09/Person_in_WiFi_ICCV2019.pdf
✉️ **Dong Huang (CMU)** / **Jinsong Han (XJTU)**

**[R10] Wi-Depth (2025)** — depth reconstruction from CSI (VAE teacher-student). arXiv:2503.06458.
🟢 open-access. Link: https://arxiv.org/abs/2503.06458 · ✉️ arXiv metadata (confirm).

**[R11] LatentCSI (2025)** — CSI → latent-diffusion image synthesis. arXiv:2506.10605.
🟢 open-access. Link: https://arxiv.org/abs/2506.10605 · ✉️ arXiv metadata (confirm).

### Thread 3 — RF-SLAM

**[R12] Gentner et al. (2016)** — "Multipath Assisted Positioning with Simultaneous Localization and
Mapping," *IEEE Trans. Wireless Communications* (Rao-Blackwellized particle filter, Channel-SLAM). 🔴 IEEE.
**[R12b] Ulmschneider/Gentner et al. (2017)** — "Multipath Assisted Positioning for Pedestrians using
LTE Signals," *Mobile Information Systems* 2017, Art. 9170746. 🟢 open-access:
https://www.hindawi.com/journals/misy/2017/9170746/
✉️ **Christian Gentner, DLR Institute of Communications and Navigation** — `christian.gentner@dlr.de`

**[R13] Gounis, Tegos, Tyrovolas, Diamantoulakis, Karagiannidis (2026)** — "When SLAM Meets Wireless
Communications: A Survey," arXiv:2602.06995.
🟢 open-access. Link: https://arxiv.org/abs/2602.06995
✉️ **George K. Karagiannidis, Aristotle Univ. of Thessaloniki** — `geokarag@auth.gr`

### Thread 4 — ISAC / 802.11bf / mmWave  ✅ verified (Round 2)

**[R14] "An Overview on IEEE 802.11bf: WLAN Sensing"** — IEEE Comms Surveys & Tutorials, 2024 —
arXiv:2310.17661. Standardizes sensing across sub-7 GHz + 60 GHz DMG; leaves algorithms to vendors.
🟢 https://arxiv.org/abs/2310.17661 ✉️ Francesca Meneghello / Rui Du group (confirm from PDF).

**[R15] Kumari, Choi, González-Prelcic & Heath (2018)** — "IEEE 802.11ad-Based Radar," IEEE TVT
67(4):3012–3027. Golay-preamble radar, <0.1 m res; *theory/simulation only.* 🟢 arXiv:1702.05833 · 🔴 IEEE.
https://arxiv.org/abs/1702.05833 ✉️ **Robert W. Heath Jr., UT Austin / NC State** — `rheath@utexas.edu`

**[R16] Han, Choi & Heath (2022)** — "Radar Imaging Based on IEEE 802.11ad Waveform in V2I" —
arXiv:2208.02473. 🟢 https://arxiv.org/abs/2208.02473 ✉️ Heath group.

**[R17] Zhang et al.** — JCAS survey — arXiv:2102.12780. 🟢 https://arxiv.org/abs/2102.12780

**[R18] Liu et al. (2022)** — IEEE JSAC **40(6):1728–1767**, DOI 10.1109/JSAC.2022.3156632 · **Xiong,
Liu et al. (2023)** — IEEE Trans. Inf. Theory **69(9):5723–5751**, DOI 10.1109/TIT.2023.3284449 — ISAC
dual-function framing + sensing–communication tradeoff. *(vol/pages confirmed 2026-07.)*

### Datasets / toolkits / reproducibility

**[R18]** CSIKit — https://github.com/Gi-z/CSIKit (maintainer: Gi-z)
**[R19]** Awesome-WiFi-CSI-Sensing — https://github.com/NTUMARS/Awesome-WiFi-CSI-Sensing
**[R20]** "A Survey on CSI-based Wi-Fi Sensing Datasets and Models with a Focus on Reproducibility" —
ResearchGate 401244480
**[R21]** "A Taxonomy of WiFi Sensing: CSI vs Passive WiFi Radar" — ResearchGate 346954384

### Additional RF-SLAM sources — verified (2026-07): modern multipath-SLAM / virtual-anchor line

**[R22] Leitinger, Wielandner, Venus & Witrisal (2024)** — "Multipath-based SLAM with Cooperation and
Map Fusion in MIMO Systems" — arXiv:2405.02126 🟢. TU Graz. Virtual-anchor (= virtual-transmitter)
MP-SLAM + cooperation/map fusion. ✉️ Erik Leitinger / Klaus Witrisal, TU Graz (confirm @tugraz.at).

**[R23] Li, Cai, Leitinger & Tufvesson (2024)** — "A Belief Propagation Algorithm for Multipath-based
SLAM with Multiple Map Features: A mmWave MIMO Application" — arXiv:2403.10095 🟢. Lund/Graz. Factor-graph
BP MP-SLAM, **real mmWave MIMO measurements** — relevant if the 60 GHz fork is taken.
✉️ Fredrik Tufvesson, Lund (@eit.lth.se).

*Adjacent (unverified, worth a look): arXiv:2404.15375, arXiv:2203.08264 (Neural RF-SLAM from CSI).
Note: Round-1's arXiv:2601.20547 does NOT exist (malformed ID) — dropped.*
