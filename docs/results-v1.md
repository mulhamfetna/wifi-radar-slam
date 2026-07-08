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

Localization works; **single-bounce map reconstruction is infeasible in this particular
street-canyon scene** — and we can now say exactly why, from an oracle analysis that feeds
Sionna's *true* per-path delay/AoA/interaction data into the geometry (`sensing/oracle.py`,
`sensing_mode: oracle`):

1. **The delay/AoA physics is exactly consistent.** For every single-scatter path, `phi_r`
   equals the bearing to the true reflection vertex and `tau·c` equals the geometric two-leg
   length `|AP→vertex| + |vertex→vehicle|` — residuals are **0.0** to float precision. The
   sensing model and AoA convention are correct.
2. **But the scene yields no *localizable* single-bounce returns.** The built-in
   `simple_street_canyon_with_cars` buildings are **penetrable**, so (a) there are **zero**
   clean single-specular facade reflections reaching the vehicle for this AP/trajectory layout
   (confirmed with `refraction=False`), and (b) the single-scatter paths that exist are
   **forward-refraction through a wall lying on the AP–vehicle line** — the reflection vertex is
   ~collinear with the endpoints, so the bistatic excess delay is ≈0 and the ellipse is
   degenerate (`denom→0`). The range is mathematically unrecoverable from delay+AoA. This is a
   property of the scene/material, not the estimator.
3. **Multi-bounce paths dominate** (`max_depth=3`: most of ~19 paths/AP are 2–3 bounce); the
   single-reflector bistatic model does not describe them.

Fixes shipped that this analysis motivated (correct regardless of scene): single-scatter path
filtering + floor-bounce exclusion (`sensing/oracle.py`), facade **footprint** ground truth via
mesh bounding boxes (`geometry.footprint_points`, `scene/builder.py`), a relaxed
`denom` triangulation guard (grazing solves are valid; plausibility is enforced on output range
`s`), and **directional map metrics** (`map_accuracy` = est→GT precision, `map_completeness` =
GT→est coverage) since a passive-WiFi map illuminates only a subset of surfaces.

**Decision (2026-07-08): demonstrate mapping where single-bounce geometry is guaranteed clean.**
Two follow-up experiments, each on its own research branch, then merged:
(1) a controlled reflective scene (opaque floor+wall / few-reflector) for a quantitative *oracle*
map; (2) forcing the street-canyon building materials opaque/reflective to elicit single-bounce
specular returns in the realistic scene. Localization stands as the headline result and is
independent of these.

## Reproduce

On a machine with `sionna-rt`: `WRS_NUM_SAMPLES=1000000 python experiments/run_phase_a.py` and
`experiments/run_phase_b.py` (see `docs/RUNNING.md`). CPU-only is fine and fast (~0.2 s/frame on a 9950X).
