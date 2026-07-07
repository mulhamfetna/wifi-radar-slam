# Round-2 Verified Findings — Thread 4: ISAC / 802.11bf / mmWave-WiFi

**Compiled:** 2026-07-04. **Method:** deep-research harness, adversarially verified (108 agents).
**Run ID:** `wf_bc779189-343` (resumable). **Full raw output:** task `w2iswd8pm` output file +
`subagents/workflows/wf_bc779189-343/journal.jsonl`.

> ⚠️ This file captures the verified findings as delivered. It has **not yet been folded** into
> `report.html`, `01-detailed-survey-report.md`, `00-literature-foundation.md`, or `references.bib`
> (Thread-4 entries there are still marked VERIFY). That merge is the next session's first task.

---

## Headline: this resolves the paper's central feasibility question

**Commodity sub-7 GHz 802.11 CSI CANNOT reach automotive-radar-grade range resolution.** Only 60 GHz
mmWave (802.11ad/ay DMG) closes the gap. This is a hard physics result, not an engineering detail, and
it should reshape the project's system-design section.

**Range resolution ΔR = c / (2·B):**

| Waveform / bandwidth | Range resolution ΔR |
|---|---|
| ~10 MHz | ~15 m |
| 40 MHz (commodity 802.11n/ac channel) | **~3.75 m** |
| 160 MHz (802.11ac/ax/be bonded) | **~0.94 m** |
| 1.76 GHz (802.11ad 60 GHz DMG single channel) | **~8.5 cm** |
| 77 GHz automotive radar (reference) | ~4–15 cm |

The DMG-to-bonded ratio (1760 / 160 ≈ **11×**) is a full order of magnitude. No amount of channel
bonding at sub-7 GHz reaches radar grade; the jump requires mmWave.

**Nuance to preserve:** papers report range *accuracy* (single-target precision, which can beat the
resolution cell at high SNR) distinct from range *resolution* (two-target separability, bandwidth-
bound). The resolution ceiling holds independently of SNR via the bandwidth argument. (This is the
correct, defensible version of the Round-1 killed claim — bandwidth limits resolution, not range.)

---

## Verified findings

### F1 — Range resolution is bandwidth-limited; sub-7 GHz can't reach radar grade (High)
ΔR = c/2B. Sub-7 GHz WLAN "cannot achieve cm-level range and cm/s-level velocity resolution … due to
insufficiently low bandwidth" (Kumari/Heath). The 802.11bf overview confirms sub-7 GHz and 60 GHz DMG
"have different BW and thus distinct range resolutions," DMG = 1.76 GHz/channel, aggregable higher.
*Verdict: unanimous (merges several 3-0 claims).*

### F2 — 802.11ad 60 GHz can be repurposed as an automotive LONG-RANGE RADAR (High, but theory/sim)
Kumari, Choi, González-Prelcic & Heath, **"IEEE 802.11ad-Based Radar: An Approach to Joint Vehicular
Communication-Radar System,"** *IEEE Trans. Veh. Technol.* 67(4):3012–3027, 2018 (arXiv:1702.05833).
Reuses the SC-PHY frame preamble's **Golay complementary sequences** as the radar waveform:
**<0.1 m range resolution, <0.6 m/s velocity resolution (4.2 ms CPI), >99.9% detection @ 1e-6 FA to
200 m (43 dBm EIRP)**, while sustaining Gbps comms — meets long-range-radar (LRR) specs on a WiFi-band
waveform.
> **CAVEAT (must cite honestly):** results are analytical / Cramér-Rao-bound / Monte-Carlo (10,000
> trials), single-target, idealized full-duplex self-interference cancellation, SCNR > 45 dB. **Not a
> hardware field measurement.** Cite as theory/simulation.

### F3 — The same 802.11ad waveform supports radar IMAGING (ISAR) in V2I (High)
Han, Choi & Heath, **"Radar Imaging Based on IEEE 802.11ad Waveform in V2I Communications"**
(arXiv:2208.02473, 2022). A roadside unit transmits the standard 802.11ad waveform, listens to echoes,
estimates round-trip delay via Golay autocorrelation, forms **high-resolution ISAR images** of vehicles
— no dedicated radar waveform. Corroborated by Zhang et al. JCAS survey (arXiv:2102.12780): "802.11ad
preamble in each packet is the main signal exploited for radar sensing … mainly investigated for
vehicular networks," RSU- or vehicle-mounted.

### F4 — IEEE 802.11bf standardizes the measurement plumbing, NOT a sensing outcome (High)
"An Overview on IEEE 802.11bf: WLAN Sensing," *IEEE Communications Surveys & Tutorials*, 2024
(arXiv:2310.17661). 802.11bf standardizes WLAN sensing across **both sub-7 GHz and 60 GHz DMG**,
defining PHY/MAC measurement-acquisition/feedback procedures but **deliberately leaving sensing
algorithms to vendors.** Implication: the standard guarantees you can *obtain* channel measurements
interoperably; it does **not** hand you a SLAM-ready sensing result. Our algorithmic contribution sits
exactly in that vendor-left-open space.

### F5 — ISAC theory bounds any WiFi-as-radar design (High)
Liu et al. (JSAC 2022) and Xiong/Liu et al. (IEEE Trans. Inf. Theory 2023) supply the dual-functional
shared-hardware/spectrum framing and the fundamental **sensing–communication tradeoff** (subspace +
deterministic-random tradeoffs; P_SC / P_CS Cramér-Rao-bound-vs-rate corner points). These are the
theoretical limits to cite when arguing our design's operating point.

---

## Net implication for the paper (design fork)

An outdoor automotive **3D-mapping SLAM** contribution built on commodity sub-7 GHz CSI is
**bandwidth-limited to ~1–4 m range resolution** — likely too coarse for fine 3D scene mapping. Two
honest paths:

1. **Stay sub-7 GHz, reframe the claim.** Lean on 802.11ac/ax/be channel bonding (→ ~0.94 m at 160 MHz)
   as partial mitigation, and position the contribution around *coarse* occupancy/structure mapping +
   SLAM pose estimation (where multipath geometry, not fine range, carries the information — cf.
   Channel-SLAM). Do **not** claim radar-grade resolution.
2. **Move to 60 GHz / 802.11bf DMG.** Adopt the 802.11ad Golay-preamble radar approach (F2/F3) to claim
   genuine radar-grade (~8.5 cm) resolution — at the cost of 60 GHz range/penetration limits and less
   ubiquitous ambient illumination.

This fork is now the central design decision and should be stated explicitly in the paper.

---

## New references (add to references.bib, replacing VERIFY stubs)

- **[R14] 802.11bf overview** — arXiv:2310.17661 — 🟢 OA. IEEE ComST 2024. ✉ Francesca Meneghello /
  Rui Du group (confirm from PDF).
- **[R15] Kumari, Choi, González-Prelcic, Heath (2018)** — "IEEE 802.11ad-Based Radar," IEEE TVT
  67(4):3012–3027 — arXiv:1702.05833 🟢 OA (IEEE Xplore 🔴). Precursor: IEEE doc 7390996 (VTC/Globecom).
  ✉ **Robert W. Heath Jr. — rheath@utexas.edu** (UT Austin/NC State); lead Preeti Kumari.
- **[R16] Han, Choi, Heath (2022)** — "Radar Imaging Based on IEEE 802.11ad Waveform in V2I" —
  arXiv:2208.02473 🟢 OA. ✉ Heath group.
- **[R17] Zhang et al.** — JCAS survey — arXiv:2102.12780 🟢 OA.
- **[R18] Liu et al. (2022)** ISAC — JSAC; **Xiong/Liu et al. (2023)** — IEEE Trans. Inf. Theory
  (S&C tradeoff / CRB-rate region). Confirm exact cites from full output.
- arXiv:2302.08378 (ISAC/mmWave) — status to confirm from full output.

> Contact emails for several arXiv authors still need confirmation from PDF metadata. Heath
> (rheath@utexas.edu) is the strongest new outreach target — the 802.11ad-radar line is the closest
> mmWave-WiFi automotive prior art.
