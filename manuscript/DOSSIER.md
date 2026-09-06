# Consolidated Journal Paper — Dossier

**Working title:** *Ambient WiFi as a Cheap Radar for SLAM: From Physics Feasibility to $30
Hardware — Why Localization Works, Mapping Hits a Phantom Ceiling, and What That Ceiling Is*
**Author:** Mulham Fetna (ORCID 0009-0006-4432-798X)
**Status:** **SUBMISSION DRAFT (9 pp, 13 sections, 7 figures, 4 tables; builds).** Consolidates the four component papers (`../papers/1..4/`) into one
substantial journal submission, in response to the IoT-J reject-with-strong-reviews on paper 1.

This is the authoritative record for the consolidated paper. The four `papers/N/DOSSIER.md`
files remain the records of their own component drafts and submission history.

---

## Why this exists

IEEE IoT-J **hard-rejected** paper 1 (IoT-70358-2026): *preliminary, 5 pages, fragmented; no
resubmission.* Reviewer 1 gave 8 constructive points, mostly about **mapping** and **physical
validation** — exactly the ground papers 2–4 later covered. **The fix is not a revision; it is
consolidation:** one comprehensive paper that is not preliminary, not fragmented, has a solid
methodology, and includes real-hardware validation.

## The one-paper arc (four stages, one story)

1. **Feasibility (from paper 1).** Physics-based, ray-traced (Sionna RT): ambient sub-7 GHz WiFi
   on a moving vehicle as a radar-replacement SLAM front-end. Localization is cm-level; mapping is
   two-tier (good with oracle sensing, poor with realistic CSI). *Uses the corrected numbers*
   (ATE 0.098 ± 0.028 m, not the retracted 0.027 — see paper 1 erratum).
2. **Mechanism + LiDAR comparison (from paper 2).** The mapping ceiling is **≈89 % phantom
   detections + a 6.45 m range bias**, *not* path discrimination (2–8 %). A learned filter cannot
   invent real paths or correct the bias. Anchored to real KITTI LiDAR (1.16 m ATE / 394 m).
3. **Radar comparison / universality (from paper 3).** Same substrate vs a 77 GHz FMCW radar.
   **The carrier does nothing; geometry is everything** (monostatic 0.1 % vs bistatic 18.2 %
   phantoms); more bandwidth makes phantoms *worse*. Scored under GT poses (the point-ICP back-end
   cannot recover radar yaw — a documented negative result).
4. **$30 hardware validation (from paper 4).** Two ESP32-S3 boards; **first light** — real HT40
   CSI captured, parsed, delay-profiled on real silicon. Moves the phantom-rate claim off
   simulation. (Static bench; validates the reading, not a full SLAM system.)

**Thesis:** WiFi localization rivals LiDAR/radar at 1–2 orders lower cost; WiFi *mapping* is capped
by a front-end + geometry phantom ceiling that is **not** carrier-specific and **not** WiFi-specific
— demonstrated in simulation across three sensors and confirmed on real hardware.

## Reviewer-response ledger (IoT-J R1)

| pt | ask | disposition |
|----|-----|-------------|
| R1.1 | novelty vs prior art | **drafted** — Sec. I Contributions + Related Work |
| R1.2 | simulation-only; no physical validation | **answered** by stage 4 (real ESP32 CSI) |
| R1.3 | fragmented contributions | **answered** by consolidation itself |
| R1.4 | 2D-MUSIC complexity / real-time | **partly** — on-chip budget (paper 4) + MUSIC-is-the-problem (2/3) |
| R1.5 | landmark promotion / when mapping fails | **answered** by stage 2 (phantom + bias mechanism) |
| R1.6 | AP-position uncertainty tolerance | **done** — ATE flat to 32 m; map degrades gently; Sec. V-F |
| R1.7 | blockage → PF resilience to measurement loss | **done** — dead-reckons a 40-frame blackout; Sec. V-F |
| R1.8 | rewrite conclusions | **drafted** — Sec. VIII |

Two genuinely-new experiments (R1.6, R1.7) are tracked as GitHub issues (Sub-project B).

## Status

- **A — repo/GitHub structure:** ✅ done (PR #10 **merged to `main`** 2026-09-06 as `12922d9`; CI green).
- **B — experiments (R1.6, R1.7):** ✅ done — results in `docs/results-ap-sensitivity.md`,
  `docs/results-blockage-robustness.md`; issues #6/#7 commented.
- **C — manuscript assembly:** ✅ **SUBMISSION READY** — `main.tex` on the official
  `ieeeaccess.cls`; 13 sections, 7 figures, 4 tables, 9 pp; author biography + photo in place;
  `cover-letter.md` written (includes the required self-similarity disclosure). Read and
  confirmed by the author 2026-09-06.
- **Venue:** IEEE Access (its class and author-guidance requirements are what the manuscript is
  built against). **Not** IoT-J — the paper-1 rejection bars resubmission there.
- **D — submission:** package assembled and audited against
  `../IEEE-Access-Submission-Checklist.pdf`; procedure, portal field values and the compliance
  table live in `SUBMISSION.md`. Checklist item 11 (acronyms defined at first use in the body,
  not just the abstract) had 14 violations — fixed 2026-09-06.
- **Open (post-submission):** the hardware phantom-rate SLOT (Sec. VI) is written as a
  simulation-anchored claim with the static-bench first-light result; the full Rung 1 number
  needs a >=15 m corridor and is scoped as a follow-up, not a submission blocker.

## Guardrails

- Never touch the frozen tags/branches (paper1-submitted, paper2-held, paper3/4, all `vX`).
- Use the **corrected** paper-1 numbers everywhere.
- Repo is public — record facts, not private deliberation (venue strategy stays out of repo files).
