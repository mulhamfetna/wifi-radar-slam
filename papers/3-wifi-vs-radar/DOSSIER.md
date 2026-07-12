# Paper 3 — Dossier (kickoff stub)

**Working title:** *WiFi vs Automotive Radar for SLAM* (to refine)
**Author:** Mulham Fetna (ORCID 0009-0006-4432-798X)
**Status:** **ACTIVE — just started (2026-07-12).** Branch `paper3-wifi-vs-radar`, cut from
`main`. No experiments designed yet — a design cycle (brainstorm → spec → plan) comes first.

This dossier is paper 3's durable record. Update it as work proceeds.

## Premise

Paper 1 framed ambient WiFi as a **radar replacement** but never simulated an actual
automotive radar. Paper 2 compared WiFi against **LiDAR** and found: parity on localization,
total failure on mapping, at 84–600× lower cost. Paper 3 completes the sensor triangle by
running the **same comparison substrate** against the sensor WiFi was always implicitly
being measured against — a **77 GHz FMCW automotive radar**.

**Why this is the sharpest of the three comparisons.**

- **Sionna models radar natively.** Radar is electromagnetic, so the ray tracer handles it
  properly. Paper 2's LiDAR Model B needed a diffuse-scattering workaround to make an EM
  tracer behave *optically*; no such hack is needed here. This is arguably the simulator's
  intended use.
- **It is a real contest, not a walkover.** WiFi vs LiDAR pitted a \$50 sensor against a
  \$20 k one. Radar costs **~\$50–200** — the same order as the WiFi package (\$40–95). The
  cost argument that carried paper 2 **stops being decisive**, and the comparison has to be
  won on physics.
- **Radar is WiFi's closest relative.** Both are RF, both bandwidth-limited, both cheap. The
  differences are precisely the interesting variables: radar has ~25× the bandwidth
  (\SI{4}{GHz} vs \SI{160}{MHz} → \SI{3.75}{cm} vs \SI{0.94}{m} range resolution), a
  **monostatic** geometry (no bistatic-ellipse ambiguity), and its **own transmitter** —
  which it must pay for, whereas WiFi's is ambient and free.

## Candidate research questions (to refine in the design cycle)

1. **Can WiFi match radar** for SLAM — localization *and* mapping — on the same scenes?
2. **Ablation: where does radar's advantage actually come from?** Bandwidth, monostatic
   geometry, or the active transmitter? These are separable in simulation, and separating
   them is a contribution in itself.
3. **Does paper 2's mapping ceiling afflict radar too?** *(Potentially the key question.)*
   Paper 2 showed WiFi's mapping floor is ≈89 % **phantom detections** plus a **6.45 m range
   bias** — a *front-end* limit. Radar, with 25× the bandwidth and monostatic geometry, is
   the perfect control. If radar maps cleanly, the ceiling is **specific to WiFi's bandwidth
   and bistatic geometry**. If radar *also* drowns in phantoms, the ceiling is a property of
   **superresolution estimation on limited data** — a much broader claim about RF sensing.
4. **Cost, honestly.** With radar at \$50–200, WiFi's price advantage nearly evaporates. So
   what, if anything, is WiFi's remaining value proposition?
5. **Fusion:** WiFi + radar — does anything remain to be gained, given they are physically
   similar?

## The thesis this is likely to arrive at (to be tested, not assumed)

If radar and WiFi are physically similar and radar is barely more expensive, then WiFi's
*only* defensible edge is that **its hardware is already in the vehicle and its transmitter
is free**. Everything else — bandwidth, geometry, SNR — favours radar. That would be a sharp,
quotable, and slightly uncomfortable conclusion: *if you are going to spend \$100 on a
sensor, buy a radar; the case for WiFi is a zero-marginal-cost case, not a performance case.*

We must be prepared to publish that if the data says it. Paper 2's habit — reporting the
negative result and the mechanism rather than tuning for a flattering one — carries over.

## Assets inherited from `main`

Same scenes and footprint ground truth · the six metrics (`eval/metrics.py`) ·
WiFi front-end (CSI → joint 2-D MUSIC → bistatic triangulation → particle-filter SLAM) ·
`lidar/` substrate (Scan, KD-tree multi-core ICP, scan-to-map SLAM with adaptive motion
model — the radar can reuse the point-cloud back-end) · `fusion.py` · `cost.py` ·
`map_filter.py` · the isolation experiment (`isolate_mapping_floor.py`) — directly reusable
to test RQ3 on radar. 85 tests pass.

## Next step

Run a **design cycle** (brainstorm → spec → plan) covering: the radar model and its
parameters, how it is simulated in Sionna (monostatic, FMCW, range–Doppler vs direct path
extraction), the realistic-vs-oracle processing tiers, the ablation design for RQ2, cost
sourcing, and the target venue. **Do not start experiments before the design is approved.**

## Do-not-mix reminders

- Papers 1 and 2 are frozen (`v0.7.1`/`paper1-submitted`; `paper2-v1.0.0`/`paper2-held`).
  Do not alter their content when evolving shared code for paper 3.
- **Paper 2 must not be submitted** until paper 1's erratum is resolved — see
  `../1-wifi-radar-slam/DOSSIER.md` and `../2-wifi-vs-lidar/SUBMISSION.md`.
- Keep paper-3 Claude-memory notes in `paper3-*` files.
- The repo is **public and linked from the papers**: record facts and actions, never private
  deliberation.
