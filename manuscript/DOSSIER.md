# Consolidated Journal Paper — Dossier

**Working title:** *Ambient WiFi as a Cheap Radar for SLAM: From Physics Feasibility to $30
Hardware — Why Localization Works, Mapping Hits a Phantom Ceiling, and What That Ceiling Is*
**Author:** Mulham Fetna (ORCID 0009-0006-4432-798X)
**Status:** **ASSEMBLING.** Consolidates the four component papers (`../papers/1..4/`) into one
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
| R1.1 | novelty vs prior art | **write** — articulate the phantom-ceiling universality + hardware |
| R1.2 | simulation-only; no physical validation | **answered** by stage 4 (real ESP32 CSI) |
| R1.3 | fragmented contributions | **answered** by consolidation itself |
| R1.4 | 2D-MUSIC complexity / real-time | **partly** — on-chip budget (paper 4) + MUSIC-is-the-problem (2/3) |
| R1.5 | landmark promotion / when mapping fails | **answered** by stage 2 (phantom + bias mechanism) |
| R1.6 | AP-position uncertainty tolerance | **experiment (open)** — sensitivity sweep; monostatic sidesteps it |
| R1.7 | blockage → PF resilience to measurement loss | **experiment (open)** — occlusion robustness |
| R1.8 | rewrite conclusions | **write** |

Two genuinely-new experiments (R1.6, R1.7) are tracked as GitHub issues (Sub-project B).

## Status

- **A — repo/GitHub structure:** in progress (this branch, `consolidation`).
- **B — experiments (R1.6, R1.7):** not started; filed as issues.
- **C — manuscript assembly:** not started (this dossier is the seed).
- **Venue:** TBD — **not** IoT-J (rejection bars it). Candidates to weigh later.

## Guardrails

- Never touch the frozen tags/branches (paper1-submitted, paper2-held, paper3/4, all `vX`).
- Use the **corrected** paper-1 numbers everywhere.
- Repo is public — record facts, not private deliberation (venue strategy stays out of repo files).
