# Paper 1 — Dossier

**Title:** *Ambient WiFi as a Radar Replacement for Automotive SLAM: A Physics-Based
Feasibility Study*
**Author:** Mulham Fetna (ORCID 0009-0006-4432-798X)
**Target venue:** IEEE Internet of Things Journal (IoT-J)
**Status:** **SUBMITTED 2026-07-08 — awaiting first decision.** Frozen at tag
`v0.7.1` / branch `paper1-submitted`. Do not develop paper 1 further except for
reviewer revisions (see playbook below).

This dossier is the durable, in-repo record of paper 1's context, progress, and
decisions — independent of Claude's (non-branch-aware) memory. Read it first when
resuming paper 1.

---

## One-paragraph summary
An open, physics-based (Sionna RT ray-traced) feasibility study of ambient
sub-7 GHz WiFi, received on a moving vehicle and processed with commodity CSI, as a
radar-replacement perception front-end for SLAM. **Localization** is centimetre-level
with a clean `ΔR = c/2B` operating envelope, and a **joint 2-D (delay–angle) MUSIC**
front-end lifts realistic commodity-CSI localization to oracle-sensing quality in
sparse multipath. **Mapping** is two-tier: ~25–30 cm with oracle single-bounce
sensing, ~4–5 m with realistic sensing. A 60 GHz + 16-antenna test shows the mapping
floor is **not** a bandwidth/aperture (resolution) limit but a **path-discrimination**
limit of commodity CSI — which we then show is **learnable** (random forest, F1 ≈ 0.9
under realistic noise). Ships the first ray-traced outdoor/vehicular WiFi-CSI dataset
(**WiFiSLAM-Sim**) and a real-CSI (Intel 5300 / nexmon) front-end proof-of-concept.

## Novelty gap (defensible)
Passive WiFi radar / CSI sensing is mature but indoor/static/infrastructure-fixed;
the one moving-platform passive radar uses 5G, not WiFi; comms+SLAM is "in its
infancy." No prior work occupies **on-vehicle, mobile, outdoor, WiFi-band** SLAM.

## Key results (numbers)
- Localization: nominal ATE ~0.03 m; envelope 20→160 MHz gives 0.78→0.038 m.
- Joint MUSIC (controlled, realistic): ATE 0.73→0.027 m (≈ oracle 0.045 m).
- Mapping oracle: controlled 0.25 m acc / 0.51 Chamfer / 0.79 IoU; street 0.30 m acc.
- Mapping realistic: ~4.8 m acc; **unchanged by 60 GHz (1.76 GHz) and 16 antennas**.
- Learned discriminator: F1 1.00 (exact features) → 0.94/0.90/0.86 at (1 m,3°)/(2 m,6°)/(4 m,10°).

## Releases and DOIs
Concept DOI (all versions): **10.5281/zenodo.21247288** (resolves to latest).

| Tag | Milestone | Version DOI |
|-----|-----------|-------------|
| v0.1.0 | Feasibility sim | 10.5281/zenodo.21247363 |
| v0.2.0 | Quantitative mapping | 10.5281/zenodo.21262513 |
| v0.3.0 | Realistic sensing concluded | 10.5281/zenodo.21263134 |
| v0.4.0 | Joint delay-angle MUSIC | 10.5281/zenodo.21263556 |
| v0.5.0 | 60 GHz test (path discrimination) | on Zenodo record |
| v0.6.0 | Path-disc + real-CSI + paper | on Zenodo record |
| v0.7.0 | WiFiSLAM-Sim dataset + discriminator | on Zenodo record |
| v0.7.1 | **Submission-ready IoT-J paper** | on Zenodo record |

## Submission package (this folder)
- `main.tex` / `main.pdf` — manuscript (6-page IEEE two-column; IEEEtran vendored,
  siunitx-free; builds locally with `pdflatex; bibtex; pdflatex x2` or on Overleaf).
- `refs.bib` — bibliography (internal notes stripped for submission).
- `supplementary.tex` / `supplementary.pdf` — reviewer-visible supplement
  (reproducibility recipe + WiFiSLAM-Sim datasheet + real-CSI provenance).
- `cover-letter.md` — confidential comments to the editor.
- Keywords: Internet of Things; SLAM; Wi-Fi; passive radar; ISAC; CSI; autonomous
  vehicles; direction-of-arrival estimation; multipath channels; ray tracing.
- IoT-J topics: Sensor-based Localization (primary); Sensor Signal Processing;
  Unmanned and Autonomous Vehicles; Intelligent Transportation Systems; Vehicular
  Networks; Sensor Modeling and Analysis; Virtualized and Simulated Systems; Pattern
  Recognition and Detection.

## Reviewer-response playbook (how to resume)
Full results live in `../../docs/results-v1.md`; reproduction in `README.md` (this
folder) and the supplement. Common re-runs (prepend `WRS_NUM_SAMPLES=1000000`):
- Localization / envelope: `experiments/run_phase_a.py configs/nominal.yaml`; `run_phase_b.py`.
- Oracle maps: `configs/controlled_oracle.yaml`, `configs/street_metal_oracle.yaml`.
- Realistic joint map: `configs/controlled_music_joint.yaml`.
- 60 GHz / aperture: `configs/controlled_music_60ghz.yaml`, `..._60ghz_16ant.yaml`.
- Dataset: `experiments/make_dataset.py`; discriminator: `experiments/train_discriminator.py`.
- Real CSI: `experiments/fetch_real_csi.sh` then `experiments/run_real_csi.py`.
Runs are short on CPU (Sionna LLVM). If a revision is submitted, tag `v0.7.2`/`v0.8.0`.

## Shared code note
The `wifi_radar_slam` pipeline in `../../src/` is **shared with paper 2**
(WiFi-vs-LiDAR, `../2-wifi-vs-lidar/`). Paper 1 is frozen; paper 2 extends the shared
code. Do not change paper-1 *content* when evolving shared code for paper 2.
