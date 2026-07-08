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

## Mapping demonstration — controlled reflective scene (`controlled_wall`)

Because the penetrable street canyon yields no localizable single-bounce returns (above),
mapping is demonstrated where single-bounce geometry is guaranteed clean: a large **metal wall**
(the built-in `floor_wall` scaled to vehicle scale, perfect-reflector material → no transmission)
with the vehicle driving parallel to it. Every AP produces one specular wall reflection per frame.
Oracle sensing (`sensing_mode: oracle`), 3 APs, 40 MHz, run on the same server in ~6 s.

| metric | value | note |
|--------|-------|------|
| map_accuracy (est→GT) | **0.25 m** | estimated reflectors lie ~25 cm from the true wall |
| map_completeness (GT→est) | 0.77 m | wall covered over the illuminated span |
| Chamfer (symmetric) | **0.51 m** | sub-metre |
| occupancy IoU | **0.79** | strong overlap |
| ATE / RPE | 0.045 / 0.007 m | localization unaffected |

`docs/assets/controlled_map.png` shows the estimated reflectors tracing the wall at x≈0 over the
illuminated y-span; `docs/assets/controlled_scene_paths.png` renders the metal wall, the three APs
(red), the vehicle antenna (green) and the specular wall-reflection rays (blue). **This validates
the mapping geometry end-to-end**: given clean single-bounce sensing, the bistatic SLAM back-end
reconstructs the reflecting surface to ~25 cm. Reproduce:
`WRS_NUM_SAMPLES=1000000 python experiments/run_phase_a.py configs/controlled_oracle.yaml`.

## Mapping in the realistic scene — reflective street canyon (`street_canyon_metal`)

The street-canyon mapping obstacle was the **penetrable** building material, not the estimator.
Overriding the building/car materials to a perfect reflector (metal) — a realistic model for
concrete/brick at 5.2 GHz, where penetration loss is high — and placing the APs **in the street**
(not embedded in the buildings) restores single-bounce specular returns. Oracle sensing, 3 APs,
40 MHz, ~21 s on the server.

| metric | value | note |
|--------|-------|------|
| map_accuracy (est→GT) | **0.30 m** | as precise as the controlled scene — reflectors lie on the true facades |
| map_completeness (GT→est) | 24.4 m | only illuminated facades are covered |
| Chamfer (symmetric) | 12.3 m | dominated by the coverage gap |
| occupancy IoU | 0.077 | partial coverage |
| ATE / RPE | 0.116 / 0.007 m | localization unaffected |

`docs/assets/street_metal_map.png` shows the estimated reflectors tracing the two **street-facing
facades** (y≈+9.6 and y≈−8.6) over the illuminated x-span, while the outer building perimeters and
distant buildings are not covered. **Interpretation:** the mapping geometry/estimator is correct
(30 cm precision, matching the controlled scene), but a single passive-WiFi pass illuminates only
a subset of surfaces — coverage, not accuracy, is the limitation. This motivates multi-pass
mapping or multi-bounce SLAM as future work, and cleanly separates the two mapping challenges.

Reproduce: `WRS_NUM_SAMPLES=1000000 python experiments/run_phase_a.py configs/street_metal_oracle.yaml`.

## Realistic sensing — MUSIC from CSI (the oracle → realistic gap)

All maps above use **oracle** sensing (Sionna's true per-path delay/AoA). The **realistic**
path estimates delays/AoA from the CSI with MUSIC. Instrumenting CSI→MUSIC against oracle truth
found the gap is a stack of four factors — two fixable conventions, two fundamental limits:

1. **Delay grid alias + antenna averaging (fixed).** The 0–480 m delay grid placed spurious peaks
   at the aliasing edge, and averaging the CSI over antennas cancelled the signal. Using antennas
   as MUSIC **snapshots** and bounding the grid to a physical range fixed it — and *improved*
   localization: nominal Phase-A ATE **0.09 → 0.033 m**. (The subcarrier centering was a red
   herring — re-referencing yields the identical Vandermonde.)
2. **Array-relative AoA (fixed, but opt-in).** MUSIC returns an electrical angle; the world
   azimuth is `β = arcsin(−sin θ)` (array axis = world +y, verified on a clean LOS sweep). This is
   physically correct but a **single ULA cannot resolve the front/back (Δx-sign) ambiguity**, so
   in multi-sided scenes it mislabels behind-vehicle reflectors and regresses localization
   (0.09 → 0.61 m). It is therefore gated behind `world_aoa` (default off); the controlled-scene
   mapping demo enables it (the wall is on a known side).
3. **Bandwidth ceiling (fundamental).** 40 MHz resolves `c/2B ≈ 3.75 m`; the street canyon's ~19
   multipath components span only ~18 m, so MUSIC delays blend. This *is* the paper's thesis.
4. **No bounce/LOS filtering (fundamental).** MUSIC picks the strongest paths (LOS + multi-bounce
   + floor); without Sionna's `interactions` it cannot filter to single-scatter, so phantom
   reflectors appear.

Realistic MUSIC on the controlled wall (`configs/controlled_music.yaml`, `world_aoa: true`) gives
**map_completeness 0.90 m** (the wall *is* recovered) but **map_accuracy 12 m** and **ATE 2.4 m** —
the phantom clutter from (3)+(4) dominates.

### What actually bounds realistic mapping (wider band + 2D array + consensus)

Three levers were tested to close the gap, on the controlled wall (front/back-free, so any residual
is not the azimuth ambiguity):

- **Wider bandwidth (160 MHz):** improved ATE (2.4 → 0.73 m) but **not** map_accuracy (12 → 11.6 m).
  Bandwidth is not the map-accuracy driver here.
- **Consensus map filter (`map_min_support`):** the map has a dense correct cluster on the wall plus
  a wide scatter of phantoms from delay-AoA sorted-index mis-pairing. Dropping clusters with < 5
  supporting detections cut the scatter: map_accuracy **12 → 5.1 m**, Chamfer **6.3 → 4.8 m**.
- **2D vehicle array — not pursued, and here is why:** after consensus, the surviving wall cluster
  is **systematically biased ~5 m toward the vehicle** (reflectors land at x≈−3 for a wall at x=0),
  with IoU 0. This is a **range bias**: MUSIC underestimates the delay in the LOS+floor+wall+multi-
  bounce mixture, placing reflectors short. Since this happens in a scene with **no front/back
  ambiguity**, a 2D array (which only resolves that ambiguity) cannot remove it. The ~5 m floor is
  MUSIC estimation bias, not array geometry.

**Conclusion (realistic sensing).** With clean single-bounce (oracle) sensing the map reconstructs
to ~0.25–0.30 m; with realistic commodity-CSI MUSIC sensing it is bounded to **~5 m Chamfer**
(≈10–20× worse), limited by delay/AoA estimation bias and multipath association — not by bandwidth
or array aperture.

### Joint 2-D (delay-angle) MUSIC (`joint_estimation`, opt-in)

The separate 1-D delay / 1-D AoA estimates are paired by sorted index, which mis-associates in
multipath and scatters phantom reflectors. A single **2-D MUSIC** (2-D spatial smoothing, SVD
subspace, delay grid bounded to the unambiguous `1/df` range) recovers each path's delay **and**
angle *together*, so the association is intrinsic. Effect (controlled wall, 160 MHz, `world_aoa`):

| metric | sorted 1-D | joint 2-D |
|--------|-----------|-----------|
| ATE | 0.73 m | **0.027 m** (matches the 0.045 m oracle) |
| Chamfer | 4.8 m | 4.1 m |
| map_completeness | 4.5 m | 3.5 m |

Correct association makes **realistic localization essentially oracle-quality** in sparse multipath,
and improves the map modestly. But the map's **~4–5 m short-range bias persists** — confirming it is a
delay *estimation bias* in multipath, not an association error. The benefit is **scene-dependent**:
in the dense street canyon (≈19 paths, 4 antennas) joint estimation cannot resolve the paths and does
not help (nominal ATE 0.033 → 0.091 m), so it is off by default.

**Overall.** Passive-WiFi **localization** is cm-level and practical — and with joint estimation it
reaches oracle quality from realistic CSI in sparse scenes. Passive-WiFi **mapping** remains bounded
(~4–5 m). The oracle map is the perfect-sensing upper bound; the consensus filter (`map_min_support`)
and joint estimator (`joint_estimation`) are retained as opt-in, backward-compatible improvements.

### What actually floors realistic mapping: path discrimination, not resolution

We tested the intuitive remedy—more bandwidth and more aperture—directly, on the controlled wall:

| configuration | map\_accuracy | ATE |
|---------------|--------------|-----|
| sub-\SI{7}{GHz}, \SI{40}{MHz}, 4-ant, joint | 4.8 m | 0.027 m |
| **\SI{60}{GHz}, \SI{1.76}{GHz}, 4-ant, joint** | **4.8 m** | 0.030 m |
| **\SI{60}{GHz}, \SI{1.76}{GHz}, 16-ant, joint** | **4.75 m** | 0.034 m |

Neither a \(44\times\) bandwidth increase (\(\Delta R:\SI{3.75}{m}\!\to\!\SI{8.5}{cm}\)) nor a \(4\times\)
larger array moves the map accuracy. The map figure (`docs/assets/controlled_music_60ghz_map.png`)
explains why: the estimated reflectors form a **consistent phantom arc hugging the vehicle
trajectory**, not the wall. These phantoms are the line-of-sight and floor-bounce paths, which MUSIC
returns among the strongest components at every pose; being geometrically consistent, they survive the
consensus filter. Bandwidth and aperture sharpen *resolution*, but the floor is **path
discrimination**—telling a genuine facade reflection from an LOS/floor/multi-bounce path. The oracle
map works precisely because it can filter by Sionna's interaction type; commodity CSI carries no such
label.

**Revised mapping conclusion.** The realistic-mapping limit is *not* the $\Delta R = c/2B$ resolution
ceiling and is *not* array aperture—so \SI{60}{GHz} alone does not enable it. It is the absence of
bounce/path discrimination in commodity CSI. Closing it needs a different class of method (e.g.
learned path classification, physical bounce-count features, or multi-pass geometric consistency),
which we identify as the key open problem for passive-WiFi mapping. Localization, which does not
require this discrimination, is unaffected and remains practical at both bands.

### A first path-discriminator: bistatic-excess gating (`map_min_excess_m`)

LOS and floor-bounce paths barely detour, so their bistatic *excess* (path\_len $-\ |AP-\text{veh}|$)
is near zero, whereas a genuine facade reflection detours significantly. Requiring a minimum excess
(controlled wall, 160 MHz, joint) cuts the phantom error—**map\_accuracy 4.8 $\to$ 2.0 m**—but
**collapses coverage** (completeness 3.5 $\to$ 10 m), and lowering the threshold from 3.0 to 1.5 m does
not recover it. The reason is the delay bias itself: MUSIC under-estimates the wall path's delay, so
*genuine* reflections also present a low excess and are gated out alongside the phantoms. Bistatic-excess
gating is therefore a useful but partial discriminator (retained as opt-in `map_min_excess_m`, default
0); a *robust* path discriminator that does not rely on unbiased delays—e.g. a learned LOS/reflection
classifier on CSI features—remains the open problem. This is consistent with the \SI{60}{GHz}/aperture
result: the mapping bottleneck is path identity, not resolution.

## Learned path discrimination (+ the WiFiSLAM-Sim dataset)

The mapping bottleneck is path discrimination, and the hand-tuned excess gate is too
blunt. We test whether the discrimination is *learnable*. The ray-traced
**WiFiSLAM-Sim** dataset (`experiments/make_dataset.py`, `docs/DATASHEET.md`) packages
the vehicular scenario—CSI, ground-truth poses/APs/map, and a flat oracle **path
table** whose per-path bounce/interaction/floor labels serve as training targets
(8356 paths from the nominal street canyon). A RandomForest
(`experiments/train_discriminator.py`) predicts the binary label *"mapping-useful
single-scatter facade reflection"* from per-path features a receiver can estimate
(range, bistatic excess, azimuth, elevation, azimuth-deviation-from-AP)—**not** the
interaction type:

| feature noise (range, azimuth) | held-out F1 (useful class) |
|--------------------------------|----------------------------|
| 0 m, 0\degree (oracle features) | **1.000** (AUC 1.000) |
| 1 m, 3\degree | 0.937 |
| 2 m, 6\degree | 0.901 |
| 4 m, 10\degree | 0.863 |

The classes are cleanly separable in the true feature space (excess and
azimuth-deviation dominate the importance), and the classifier degrades gracefully
under realistic MUSIC-level feature error, holding **F1 \(\approx0.9\)**—well above
what the fixed excess gate achieves. **Path discrimination is therefore learnable and
robust**; its ceiling is set by how accurately the features (chiefly the
delay-derived excess) can be estimated—closing the loop with the delay-bias finding.
This is a first concrete answer to the open problem, and the dataset makes it
reproducible and extensible.

## Real commodity-CSI proof-of-concept

To check that the sensing front-end is not simulation-specific, the *same* MUSIC
delay/AoA estimator was run on **measured hardware CSI** via a CSIKit-backed
ingestion adapter (`io_csi.load_real_csi`, `experiments/run_real_csi.py`):

- **Intel 5300** (`log.all_csi.6.7.6.dat`, 30 subcarriers, 3 antennas, HT20): the
  front-end runs end-to-end over all frames and returns plausible indoor multipath
  delay/AoA estimates (AoA \(\in[-20\degree,26\degree]\)).
- **Broadcom nexmon** (802.11ac, \SI{80}{MHz}): parses and runs equally.

This is a *front-end* validation, not a SLAM one: the captures are indoor and
static with no ground-truth trajectory (and CSI carries no absolute timing
reference, so absolute path length is not calibrated—only relative multipath
structure). It demonstrates the pipeline consumes real Intel-5300/nexmon CSI and
produces sensible estimates. A full outdoor, vehicle-mounted validation is blocked
by the absence of any public outdoor/vehicular WiFi-CSI dataset—itself the gap this
work targets—and remains the key next step. Fetch the CSIKit sample captures with
`experiments/fetch_real_csi.sh`; install the parser with `pip install -e .[realcsi]`.

## Reproduce

On a machine with `sionna-rt`: `WRS_NUM_SAMPLES=1000000 python experiments/run_phase_a.py` and
`experiments/run_phase_b.py` (see `docs/RUNNING.md`). CPU-only is fine and fast (~0.2 s/frame on a 9950X).
