# PROJECT.md — the single source of truth

This file is the one authoritative record of this project: goal, status, findings, data, how to
reproduce, decisions, and open tasks. **If anything else in the repository disagrees with this
file, this file wins, and the other thing is a bug.** Update it in the same commit as any change
to results, status, or direction.

The repository is public. Record facts and decisions here; private correspondence (journal
decision letters) stays in the gitignored `reviews/` folder.

---

## 1. Goal

Publish **one** corrected journal paper:

> *Can Ambient WiFi Support Vehicular Multipath SLAM? Virtual-Anchor Localization, Commodity
> Front-End Limits, and a Delay-Convention Pitfall in Ray-Traced Evaluation*
> — Mulham Fetna (sole author), target **Pervasive and Mobile Computing** (Elsevier).

**Status: SUBMITTED 2026-10-22 17:57** to *Pervasive and Mobile Computing* through Elsevier's new
submission service; article type *Research Paper*; sole and corresponding author. Awaiting the
confirmation email, which carries the manuscript number; the author must then sign in to the
journal account (or set a password) to track the submission and handle revisions.

**Compliance, checked against the journal's guide for authors (read 2026-10-22):**

| Requirement | State |
|---|---|
| Abstract at most 250 words, non-standard abbreviations defined | 249 words; SLAM, CSI, MUSIC defined |
| 1 to 7 keywords | 6 |
| Highlights: separate file named with "highlights", 3-5 bullets of at most 85 characters | `manuscript/highlights.txt`, 5 bullets, max 83 |
| Numbered references in order of appearance; DOIs where available | `elsarticle-num`; DOIs added from Crossref (exact title/year match only) |
| Software cited as software with version and identifier | Sionna RT v2.0.1 cited as software plus its paper |
| CRediT statement; funding sentence | present |
| Generative-AI declaration, section before references, journal wording | present; **author must confirm** |
| Competing interests via the journal's declarations tool (Word file uploaded at submission) | at submission |
| Research data (Option C): deposit in a repository and cite with a persistent identifier | Zenodo v1.0.0, 10.5281/zenodo.23560036, cited in the data statement and references |
| Figures supplied as separate files | `manuscript/figures/*.pdf` (vector, fonts embedded) |
| Single-anonymized review | author names stay on the manuscript |
| Free SSRN preprint option at submission | author's choice |

## 2. Repository layout

| Path | What it is |
|---|---|
| `PROJECT.md` | This file |
| `manuscript/` | The paper: `main.tex`, `main.pdf`, `refs.bib`, `figures/`, `highlights.txt`, `cover-letter.md`, Elsevier class; `SUBMISSION-GUIDE.md` (step-by-step submission) and `make-submission.sh` (upload package) |
| `src/wifi_radar_slam/` | Library: scenes, channel simulation, MUSIC front-end, FastSLAM, LiDAR baseline, ESP32 CSI parsing |
| `experiments/` | Entry points (section 5) |
| `configs/` | Scene and RF configurations |
| `tests/` | Test suite (`CUDA_VISIBLE_DEVICES= pytest -q`) |
| `results/` | Committed result files the paper reads |
| `data/` | KITTI anchor result, hardware captures; regenerable caches (gitignored) |
| `firmware/` | ESP32-S3 transmitter/receiver firmware |
| `docs/` | Hardware how-tos only (`hardware-build.md`, `esp32-s3-usb-vs-com-port.md`) |
| `literature/` | Literature survey notes (reference material) |

## 3. Findings (authoritative numbers)

Setup: Sionna RT 2.0.1 ray tracing at 5.2 GHz; 4-element half-wavelength ULA; 128 subcarriers;
20 dB SNR; joint 2-D MUSIC, 3 paths per AP; three APs. Scenes: controlled metal wall (30 m,
120 frames), metal street canyon (60 m, 240 frames, straight), same canyon on a 2.5 m lane-change
S-curve. Wheel-grade odometry (1 % scale, 0.5° heading, 0.2°/√s heading walk, 0.05 m/s white);
WiFi and LiDAR runs share odometry draws. Mean ± std over 10 seeds at 160 MHz.

### F1. Localization (ATE, m) — `results/corrected_eval.json`, `results/lidar_noisy_odom_geo.json`

| Scene | Dead reckoning | WiFi oracle | WiFi MUSIC | MUSIC two-branch | LiDAR (ICP) |
|---|---|---|---|---|---|
| Controlled wall | 0.211 ± 0.109 | **0.072 ± 0.026** | 0.340 ± 0.123 | 0.390 ± 0.110 | 0.172 ± 0.117 |
| Street canyon | 0.470 ± 0.260 | 0.101 ± 0.045 | 1.202 ± 0.241 | 0.527 ± 0.167 | **0.027 ± 0.001** |
| Street, S-curve | 0.472 ± 0.259 | 0.100 ± 0.047 | 1.065 ± 0.199 | 0.488 ± 0.266 | **0.028 ± 0.000** |

Exact multipath parameters cut odometry drift 66–79 % (3 and 7 virtual anchors confirmed, the true
counts). Realistic MUSIC is worse than dead reckoning everywhere: it confirms 28 (wall) and
101–116 (canyon) anchors.

### F2. Front-end composition, 160 MHz (%)

| | Controlled | Street | S-curve |
|---|---|---|---|
| True facade path | 15.7 | 15.5 | 15.2 |
| Other real path (LOS, floor, multi-bounce) | 29.2 | 19.0 | 18.2 |
| Real path, ULA front/back folded | 19.3 | 30.8 | 34.1 |
| Unexplained | 35.8 | 34.6 | 32.4 |
| Facade range error, median (m) | +0.03 | −0.06 | −0.06 |

Matching rule: per frame, same AP, cost √((Δρ/3 m)² + (Δφ/10°)²) ≤ 3.

### F3. Mapping at ground-truth poses (occupancy IoU, 1 m cells)

| | Controlled | Street | S-curve |
|---|---|---|---|
| Oracle | 0.767 | 0.075 | 0.075 |
| MUSIC | 0.349 ± 0.017 | 0.031 ± 0.002 | 0.028 ± 0.003 |

### F4. Bandwidth (MUSIC; 5 seeds)

| Bandwidth | Controlled unexplained / IoU | Street unexplained / IoU |
|---|---|---|
| 20 MHz | 44.4 % / 0.474 | 37.7 % / 0.012 |
| 40 MHz | 36.9 % / 0.341 | 43.3 % / 0.020 |
| 80 MHz | 35.9 % / 0.331 | 33.3 % / 0.027 |
| 160 MHz | 35.8 % / 0.349 | 34.6 % / 0.031 |

### F5. Robustness (oracle ATE, m; 5 seeds) — `results/robustness_corrected.json`

Dead reckoning for these seeds: 0.158 (controlled), 0.370 (street).

| AP error std (m) | 0 | 0.5 | 1 | 2 | 4 | 8 |
|---|---|---|---|---|---|---|
| Controlled | 0.057 | 0.242 | 0.308 | 0.393 | 0.456 | 0.508 |
| Street | 0.083 | 0.339 | 0.479 | 0.668 | 0.987 | 1.204 |

| Blackout (frames) | 0 | 10 | 20 | 40 | 80 |
|---|---|---|---|---|---|
| Controlled | 0.057 | 0.053 | 0.058 | 0.074 | 0.130 |
| Street | 0.083 | 0.095 | 0.092 | 0.106 | 0.164 |

AP positions must be known to sub-metre accuracy; blackouts are tolerated.

### F6. Anchors that are unaffected

- LiDAR ICP back-end on real KITTI sequence 04: 1.16 m ATE over 394 m (`data/kitti_results.json`).
- ESP32-S3 first light: valid HT40 CSI, 128 subcarriers with 114 non-null, LOS tap aligned at 0 m
  (`data/hw_captures/first_light_los_2026-07-17.bin`).

## 4. Defects found in 2026-10 and claims withdrawn

| Defect | Effect |
|---|---|
| **D1** Every SLAM run used ground-truth velocity | Trajectory error measured filter process noise only |
| **D2** Legacy particle filter scored particles against a reflector triangulated from its own mean pose | Measurements carried no position information |
| **D3** Sionna `Paths.cfr()` references delays to the first arrival (`normalize_delays=True`); the back-end read them as absolute path lengths | Spurious 6.45 m range bias; ≈89 % "phantoms"; map IoU 0 |

Fixes: `odometry.py`, `slam/fastslam.py` (virtual anchors, `excess_delay`, `ambiguous_azimuth`),
`eval/scenes.py`; the simulator passes `normalize_delays=True` explicitly with the convention
documented. A regression test asserts WiFi beats dead reckoning under noisy odometry.

**Withdrawn** (earlier drafts and the IEEE Access consolidation): WiFi localization parity with
LiDAR; robustness flat to 32 m of AP error; fusion design rule; cost-normalized accuracy; the
"phantom ceiling" of ≈89 % with a 6.45 m bias, its bandwidth/aperture immunity, and the learned-
filter result; the radar ablation is not re-verified and not used.

## 5. Reproduce

```sh
pip install -e '.[sim,dev]'
export CUDA_VISIBLE_DEVICES=        # Sionna's CUDA backend needs compute capability >= 7.0
.venv/bin/python experiments/cache_channels.py configs/controlled_music_joint.yaml
.venv/bin/python experiments/cache_channels.py configs/street_metal_music.yaml
.venv/bin/python experiments/cache_channels.py configs/street_metal_music_scurve.yaml \
    data/cache_street_canyon_metal_scurve.npz
.venv/bin/python experiments/make_music_detections.py     # -> results/dets/ (gitignored)
.venv/bin/python experiments/run_corrected_eval.py        # F1-F4
.venv/bin/python experiments/run_robustness_corrected.py  # F5
.venv/bin/python experiments/run_lidar_noisy_odom.py geo  # LiDAR column of F1
.venv/bin/python experiments/make_corrected_figures.py    # manuscript/figures/
(cd manuscript && pdflatex main && bibtex main && pdflatex main && pdflatex main)
```

Runtime on a 12-core CPU: caches about 20 min, detections about 10 min, evaluation about 15 min.
Other entry points: `run_lidar_kitti.py` / `fetch_kitti.py` (F6), `run_bench.py` and
`run_hw_phantom.py` (ESP32 pipeline).

## 6. Hardware

Two ESP32-S3 boards (one HT40 transmitter, one CSI receiver), firmware in `firmware/`, build and
wiring in `docs/hardware-build.md`; flash and log through the COM/FTDI port, not native USB
(`docs/esp32-s3-usb-vs-com-port.md`). First light achieved on a static bench. **Next hardware
step:** reflector measurements at surveyed ranges in a corridor of at least 15 m, to measure
detection composition (F2) on real silicon. Blocked on access to such a space.

## 7. Decision log

| Date | Decision |
|---|---|
| 2026-08-15 | IEEE IoT-J rejected the feasibility paper (no resubmission) |
| 2026-09-06 | Four component papers consolidated into one IEEE Access submission |
| 2026-10 | IEEE Access rejected it without review |
| 2026-10-21 | Re-verification found D1–D3; plan: fix, re-run everything, retarget Elsevier |
| 2026-10-22 | Corrected study written; target Pervasive and Mobile Computing; radar ablation dropped (CPU re-run impractical, raw output never committed) |
| 2026-10-22 | Repository reset to one manuscript and this file; old papers and drafts removed (see section 9) |
| 2026-10-22 | **Submitted** to Pervasive and Mobile Computing (Research Paper, 17:57); manuscript number pending |
| 2026-10-22 | Release v1.0.0 archived on Zenodo (10.5281/zenodo.23560036) and cited in the manuscript |
| 2026-10-22 | Language/accuracy pass; "as published" relabelled "earlier reading" (the earlier work was never published); CRediT and generative-AI declarations added |

## 8. Open tasks

1. When the confirmation email arrives: sign in to the journal account, then record the manuscript
   number here (status and decision log).
2. ~~Tag the submitted state~~ done: tag `pmc-submitted` (pushed). Revisions happen on `main`.
3. Wait for the editor's assessment; on a decision follow `manuscript/SUBMISSION-GUIDE.md`, Step 5.
4. Optional, useful for a revision: corridor reflector measurement (section 6).
5. Optional: re-run the radar ablation on a capable host (retrieve `run_ablation.py` from the
   `archive/ieee-access-submission` tag).

## 9. History and archives

Everything removed in the 2026-10 reset is in git history:

| Tag | State |
|---|---|
| `archive/ieee-access-submission` | Full repository before the reset: IEEE Access manuscript, `papers/1-4`, old docs, radar code |
| `archive/paper2-resolution-fix` | The one branch that was never merged |
| `v0.7.1` | Paper 1 as submitted to IEEE IoT-J |
| `paper2-v1.0.0` | Paper 2 frozen draft |
| `paper3-v0.1.0`, `paper3-v0.2.0` | Radar sub-projects |

Retrieve any old file with `git show archive/ieee-access-submission:<path>`.

## 10. Releases (Zenodo)

Repository id `1292636094`; the README DOI badge is wired to it. Zenodo archives each GitHub
release automatically once the GitHub integration is enabled for the repository. Before a
release: tests pass, `CITATION.cff` version and date updated, this file's status updated.
