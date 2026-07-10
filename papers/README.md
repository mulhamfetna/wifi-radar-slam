# Papers in this repository

This repository hosts **one shared codebase** (`../src`, `../experiments`,
`../configs`, `../tests`, `../docs`, `../literature`) and **multiple manuscripts**,
each isolated in its own folder here. The code is shared and evolves; the papers do
not mix. Layout and conventions are set by
`../docs/superpowers/specs/2026-07-09-two-paper-repo-layout-design.md`.

## Papers
| Folder | Paper | Status |
|--------|-------|--------|
| `1-wifi-radar-slam/` | Ambient WiFi as a Radar Replacement for Automotive SLAM (IEEE IoT-J) | **Submitted 2026-07-08**, frozen at tag `v0.7.1` / branch `paper1-submitted` |
| `2-wifi-vs-lidar/` | WiFi sensing as a drop-in LiDAR replacement for SLAM (accuracy/cost/fusion vs LiDAR) | In progress on branch `paper2-wifi-vs-lidar` |

## Conventions
- **Shared code stays on `main`.** Both papers `import wifi_radar_slam`. Paper-specific
  new modules (e.g. a LiDAR baseline, fusion) also live in the shared `../src` — they
  are additive and available to both papers.
- **Each paper folder has a `DOSSIER.md`** — the durable, in-repo record of that
  paper's status, findings, releases/DOIs, and a resume/reviewer-response playbook.
  Read the dossier first when resuming a paper. (This exists because Claude's memory is
  shared across branches and not a reliable per-paper record.)
- **Pinning a submitted paper:** an immutable **tag** (`vX.Y.Z`) marks the exact
  submitted state; a frozen **branch** (`paperN-submitted`) is a convenience pointer.
  Do not develop on these; revisions happen on `main`, then a new tag.
- **Starting a new paper:** `git checkout -b paperN-<slug>` off `main`; add
  `papers/N-<slug>/` with a `DOSSIER.md` stub; develop on the branch and merge to
  `main` as it matures (WIP never lands on `main` until merged, so papers never mix).
- **Claude memory hygiene:** per-paper memory files are namespaced (`paper1-*`,
  `paper2-*`); `MEMORY.md` separates the papers and points frozen papers at their
  in-repo dossier.
