# Paper 2 — Dossier (stub)

**Working title:** *WiFi Sensing as a Drop-in LiDAR Replacement for SLAM*
**Author:** Mulham Fetna (ORCID 0009-0006-4432-798X)
**Status:** **ACTIVE — just started.** Branch `paper2-wifi-vs-lidar` (developed
openly, merged to `main` as it matures). No experiments designed yet.

This dossier is paper 2's durable record. Update it as work proceeds.

## Premise
The original intent behind this project was a **LiDAR** replacement; paper 1
(radar-framing feasibility, submitted to IoT-J — see `../1-wifi-radar-slam/DOSSIER.md`)
established that ambient WiFi is a viable SLAM sensing modality with a clear
localization/mapping profile. Paper 2 takes the direct step: **can WiFi sensing be a
drop-in replacement for LiDAR in SLAM?** The headline motivation is **cost** — a full
WiFi sensing package is far cheaper than a single LiDAR unit.

## Research questions (to refine in the paper-2 brainstorming cycle)
1. Can ambient WiFi sensing **efficiently and fully replace LiDAR** for SLAM?
2. Is **pure WiFi** enough, or is **deep-learning enhancement** needed to reach
   LiDAR-equivalent results?
3. Is WiFi **more / equally / less accurate** than LiDAR as a replacement?
4. Does running **WiFi + LiDAR side by side (fusion)** improve efficiency/accuracy
   **significantly, marginally, or not at all**?
5. Quantify the **cost/efficiency** advantage (WiFi package vs one LiDAR) as the
   central value proposition.

## Relationship to paper 1 / shared code
Extends the shared `wifi_radar_slam` pipeline in `../../src/` (sensing → mapping →
SLAM → metrics → WiFiSLAM-Sim dataset → learned discriminator). New, additive shared
modules expected: a **LiDAR baseline** (point-cloud SLAM in the same simulated
scenes), a **WiFi/LiDAR fusion** path, and a **cost model**. This is an extension, not
replication: same substrate, new comparative + cost + fusion research questions.

## Next step
Run a dedicated **brainstorming cycle** to design paper 2's experiments (LiDAR model
choice, comparison metrics, fusion approach, DL enhancement, cost methodology, target
venue), then a spec and implementation plan. Do not start experiments before that
design is approved.

## Do-not-mix reminders
- Paper 1 is frozen (`v0.7.1` / `paper1-submitted`); do not alter its *content* when
  evolving shared code for paper 2.
- Keep paper-2 Claude-memory notes in `paper2-*` files (see `MEMORY.md`).
