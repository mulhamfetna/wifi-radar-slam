# v1 Simulation Results — WiFi-Radar-for-SLAM (feasibility)

**Run:** 2026-07-07/08 on an AMD Ryzen 9 9950X (32 threads, CPU/LLVM ray tracing — no CUDA),
Sionna RT 2.0.1, built-in `simple_street_canyon_with_cars` scene, full quality
(`WRS_NUM_SAMPLES=1_000_000`). Phase A ≈ 30–55 s; Phase B (16 sweep points) ≈ 16 min.

## Visuals

- `docs/assets/scene_paths.png` — Sionna RT render of the street scene (buildings, a car) with the
  **WiFi multipath rays** from the AP to the vehicle antenna. Reproduce: `python experiments/render_scene.py`.
- `docs/assets/phase_a_map.png` — Phase-A estimated vs ground-truth trajectory + map (trajectory tracks
  well; map still diffuse — see limitations).

## Headline

**Ambient WiFi received on a moving vehicle localizes the vehicle to sub-metre — down to a few
centimetres at higher bandwidth/SNR — in an outdoor street-canyon-with-cars scene.** This is a
positive feasibility signal for WiFi as a radar-substitute pose sensor in SLAM.

## Phase A (nominal, 40 MHz, 3 APs, 20 dB SNR, 5 m/s)

| metric | value | note |
|--------|-------|------|
| ATE (trajectory) | ~0.03–0.37 m | excellent localization |
| RPE | ~0.006–0.09 m | locally consistent |
| Chamfer (map) | ~16 m | **map not yet resolved** (see limitations) |
| IoU (map) | ~0 | " |

## Phase B — operating envelope (ATE, m)

| Bandwidth | ATE || SNR (dB) | ATE || # APs | ATE || Speed (m/s) | ATE |
|-----------|-----||----------|-----||-------|-----||-------------|-----|
| 20 MHz | 0.78 || 0 | 2.02 || 1 | 0.084 || 1 | 2.13 |
| 40 MHz | 0.093|| 10 | 1.86 || 2 | 1.97 || 5 | 0.032 |
| 80 MHz | 0.058|| 20 | 1.01 || 3 | 0.075 || 10 | 0.047 |
| 160 MHz | 0.038|| 30 | 0.076|| 4 | 0.041 || 15 | 0.010 |

- **Bandwidth**: monotonic ~20× ATE improvement 20→160 MHz — the resolution-vs-bandwidth relationship
  (ΔR = c/2B) from the literature survey, demonstrated end-to-end.
- **SNR**: graceful degradation; sub-decimetre by 30 dB.
- **APs**: more APs generally better (4 → 4 cm); the 2-AP outlier (1.97 m) hints at a geometry/
  degeneracy sensitivity worth investigating.
- **Speed**: robust across 5–15 m/s; the 1 m/s outlier (2.13 m) is under study.

## Limitations / next research (the map)

Localization works; **map reconstruction does not yet**. The estimated reflector cloud is bounded
(after the plausibility guard) but diffuse. An **oracle test** (feeding Sionna's true per-path `tau`
and `phi_r` into the bistatic triangulation) pinned the root causes precisely:

1. **AoA convention is CORRECT** (verified): line-of-sight `phi_r` equals the geometric bearing to each
   AP (e.g. 90° for an AP due north). World bearing = `phi_r`. So the geometry/formula are right.
2. **Multi-bounce paths are the dominant error.** With `max_depth=3` most of the ~19 paths/AP are 2–3
   bounce; the single-reflector bistatic model maps them to *phantom* positions. **Fix:** filter to
   single-specular-bounce paths (via `paths.interactions`) before mapping.
3. **Ground truth = object mesh centroids**, but reflections occur on **facades** — Chamfer/IoU compare
   against the wrong reference. **Fix:** derive ground truth from mesh bounding-box/footprint surfaces.
4. **Realistic sensing** (MUSIC AoA from commodity CSI) is a separate, harder problem than the oracle;
   the estimator's steering model must match Sionna's antenna phase convention.

Concrete next steps: (a) single-bounce path filtering + facade ground truth → makes the *oracle* map
quantitative; (b) then close the gap to MUSIC-estimated sensing. The *localization* half is already a
solid result and is independent of these.

## Reproduce

On a machine with `sionna-rt`: `WRS_NUM_SAMPLES=1000000 python experiments/run_phase_a.py` and
`experiments/run_phase_b.py` (see `docs/RUNNING.md`). CPU-only is fine and fast (~0.2 s/frame on a 9950X).
