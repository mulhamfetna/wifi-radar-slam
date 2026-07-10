# Two-paper repo layout — design

**Date:** 2026-07-09
**Status:** approved (brainstorming), pending execution
**Context:** Paper 1 (*Ambient WiFi as a Radar Replacement for Automotive SLAM*) is
**submitted to IEEE IoT-J** and frozen at tag `v0.7.1`. Paper 2 (*WiFi sensing as a
drop-in LiDAR replacement for SLAM* — accuracy/coverage vs LiDAR, pure-WiFi vs
deep-learning, WiFi+LiDAR fusion, and the cost argument) is starting. Paper 2
**extends the same pipeline** (WiFi sensing → mapping → SLAM → metrics → dataset →
discriminator); it adds a LiDAR baseline, a fusion path, and a cost model. It is an
extension, not replication, and shares code — so **one repository**, not a new one.

## Goals
- Keep paper 1's submitted state and its full context **pinned and immutable**.
- Let paper 2 **reuse and grow the shared code** without forking.
- Ensure the two manuscripts and their progress records **never mix**.
- Survive the fact that Claude's auto-memory is **not branch-aware** (shared across
  branches, lives outside git).

## Decision (chosen model: hybrid — directories + a pinned branch)

### Layout (on `main`, the shared trunk)
```
src/ experiments/ configs/ tests/ docs/ literature/   # shared code & assets (BOTH papers)
papers/
  README.md                       # convention: how papers are organized here
  1-wifi-radar-slam/              # paper 1 (moved from paper/)
    main.tex main.pdf refs.bib supplementary.{tex,pdf} cover-letter.md
    IEEEtran.cls IEEEtran.bst README.md
    DOSSIER.md                    # durable progress/context/status record
  2-wifi-vs-lidar/                # paper 2 (created at kickoff)
    DOSSIER.md                    # paper-2 record (stub at first)
```
Shared code is never duplicated; both manuscripts `import wifi_radar_slam`. Paper 2's
new modules (`lidar`, `fusion`, cost tooling) land in the shared `src/`.

### The pin (three layers, strongest first)
1. **`v0.7.1` tag** — immutable snapshot of exactly what was submitted (keeps the old
   `paper/` path; never disturbed by the move).
2. **`paper1-submitted` branch** — a frozen convenience branch at the submission
   state; not developed on, only revisited for reviewer revisions.
3. **`papers/1-wifi-radar-slam/DOSSIER.md`** — committed, human-readable record:
   status (submitted to IoT-J, awaiting decision), narrative + key findings/decisions,
   the v0.1.0→v0.7.1 release + Zenodo DOI table, timeline, and a **reviewer-response
   playbook** (what to re-run, which files/configs matter). This makes paper 1's
   context durable independent of Claude memory.

### Memory hygiene (Claude auto-memory, outside git)
- Prefix the existing paper-1 memory files with `paper1-`.
- `MEMORY.md` header states: **paper 1 = SUBMITTED/FROZEN; durable record =
  `papers/1-wifi-radar-slam/DOSSIER.md`**; reserve a `paper2-*` space with a paper-2
  section pointing at `papers/2-wifi-vs-lidar/DOSSIER.md`.
- Future sessions read the repo dossier for paper 1 and keep paper 2's live memory
  separate.

### Paper-2 kickoff (develop openly)
- `git checkout -b paper2-wifi-vs-lidar` off `main`; add `papers/2-wifi-vs-lidar/`
  with a DOSSIER stub; point live memory at `paper2-*` files.
- Develop on the branch, **pushed to the public repo** as it matures, merged to
  `main` when ready (same feature-branch flow used throughout paper 1). Paper-2 WIP
  never lands on `main` until merged.

### Cleanup
- Delete the stale `origin/feature/sim-dataset` remote branch.
- Fold the untracked `notes.md` into the paper-1 dossier (or gitignore it).
- Fix `\graphicspath` in the moved paper (`../docs/assets` → `../../docs/assets`) and
  **rebuild `main.pdf` to verify identical rendering**. Update relative paths in the
  paper README/supplementary as needed.

## Non-goals
- No new repository; no code fork; no change to paper 1's *content* (only its path).
- Not designing paper 2's research/experiments here — that is its own brainstorming
  cycle once this layout exists.

## Acceptance
- `papers/1-wifi-radar-slam/main.pdf` rebuilds byte-for-content-identical to `v0.7.1`.
- `paper1-submitted` branch + `v0.7.1` tag both resolve to the submitted state.
- `papers/1-wifi-radar-slam/DOSSIER.md` and `papers/README.md` exist and are committed.
- Memory files namespaced `paper1-*`; `MEMORY.md` has paper-1 and paper-2 sections.
- Repo tests still pass (`pytest`, excluding Sionna smoke tests).
