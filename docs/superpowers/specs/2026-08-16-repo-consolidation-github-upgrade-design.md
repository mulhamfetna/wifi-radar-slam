# Repo consolidation & GitHub upgrade — design

**Date:** 2026-08-16
**Branch:** `consolidation` (off `main`)
**Status:** approved (brainstorming); Pages explicitly excluded.
**Context:** Paper 1 was **hard-rejected** by IEEE IoT-J (IoT-70358-2026): no resubmission,
"extends to any revisions based on this manuscript." The AE called it *preliminary, 5 pages,
fragmented*; Reviewer 1 gave 8 constructive points (mostly mapping + physical validation).
Decision: **consolidate the four papers into one substantial journal paper.** This spec covers
**Sub-project A only** — the repo/GitHub structure that will hold the consolidated work.
Sub-project B (the two open experiments) and Sub-project C (the manuscript itself) are separate.

---

## Goal

Turn a four-thin-papers repo into a **one-strong-paper** repo, exploiting GitHub features to
drive the reviewer-response work — **without touching the frozen submission history.**

## Non-negotiable invariant

Every `vX` tag, `paper2-v*`, `paper3-v*`, and the branches `paper1-submitted`, `paper2-held`,
`paper2-*`, `paper3-*`, `paper4-hardware-testbed` are the **permanent record of what was
submitted.** They are never modified, rebased, or deleted. Consolidation is purely additive.

## 1. Branch & history model

- `paper4-hardware-testbed` already **contains all of paper 3** (verified: paper3 is an ancestor
  of paper4). So unification is a single merge.
- Work on a new `consolidation` branch off `main`; `git merge paper4-hardware-testbed` brings
  papers 3+4 onto the 1+2 line. (Done — one trivial `.gitignore` conflict, resolved.)
- Restructure on `consolidation`; open a **PR into `main`**. `main` stays stable until merge.

## 2. File layout

- **`manuscript/`** — the single consolidated journal paper (LaTeX) + its own `DOSSIER.md`.
  The primary deliverable. (Content is Sub-project C; this spec scaffolds the folder + dossier.)
- **`papers/1..4/`** — retained as **visible lineage**. Each gets a top banner: *"Superseded —
  folded into `../../manuscript/`. Retained as the component draft and frozen submission record."*
- **`src/`, `firmware/`, `experiments/`, `configs/`, `data/`, `tests/`, `docs/`, `literature/`** —
  unchanged, now unified.

## 3. Issues, milestone, labels, templates

- **Milestone:** *Consolidated journal submission.*
- **Labels:** `type:experiment`, `type:analysis`, `type:writing`, `type:structure`,
  `reviewer-response`, `area:sim`, `area:hardware`, `area:mapping`, `priority:high`, `blocked`.
- **Issues:**
  - Reviewer points **R1.1–R1.8**, each stating what already addresses it (paper 2/3/4) or that
    it is open.
  - The two experiments: **AP-position sensitivity (R1.6)** and **blockage/occlusion robustness
    (R1.7)**.
  - Consolidation writing tasks (novelty articulation R1.1, conclusions R1.8, the merge itself).
  - The structure-upgrade tracking issue.
- **`.github/`:** issue templates (`experiment`, `reviewer-response`, `writing`) + PR template.

## 4. CI (GitHub Actions)

- `.github/workflows/tests.yml`: on push/PR to `main`/`consolidation`, set up Python 3.11,
  `pip install -e '.[dev]'` (or minimal deps), run `pytest`. **README status badge.**
- The Sionna `sim` extra is heavy; CI installs only what the pure-NumPy tests need (the hw/radar/
  eval tests do not import Sionna). Verify the suite runs without the `sim` extra.

## 5. Projects board

- A GitHub Project (v2) board: **Backlog / In progress / In review / Done**, populated from the
  issues, associated with the milestone.

## 6. Repo identity & metadata

- **Keep** the name `wifi-radar-slam` (Zenodo DOIs and paper links depend on it).
- Update the GitHub **description** and **topics** to the consolidated scope.
- Rewrite the top-level **README** around the one-paper story: feasibility (P1) → mechanism &
  LiDAR comparison (P2) → radar comparison (P3) → **$30 ESP32 hardware validation (P4)**; with
  CI + DOI badges, repo map, reproduction, and the lineage note.
- Refresh **`CITATION.cff`** to the consolidated work.

## 7. Excluded (this pass)

- **GitHub Pages** — explicitly skipped (keeps the repo's outputs off a published website).

## Verification

- `pytest` green on `consolidation` (currently 199 passed, 4 skipped).
- All frozen tags/branches still resolve to their original commits (verified post-merge).
- CI workflow goes green on push.
- `main` unchanged until the PR merges.

## Out of scope (later sub-projects)

- **B:** the blockage-robustness and AP-sensitivity experiments (code + results).
- **C:** writing the consolidated manuscript (merging the four papers' narratives, new figures,
  the corrected paper-1 numbers, the venue choice).
