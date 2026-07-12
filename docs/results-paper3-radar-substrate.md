# Paper 3 · Sub-project 1 — radar substrate: validation results

**Date:** 2026-07-12 · **Branch:** `paper3-sub1-radar-substrate` · **Server:** amd
**Artifact:** `results/radar_substrate_validation.json` (produced by
`experiments/validate_radar_sensor.py`, Sionna RT 2.0.1, `WRS_NUM_SAMPLES=1000000`)

These are facts that cannot be re-derived without the simulator. Every one of them was
*measured*, not assumed — several contradicted the assumption.

---

## 1. Diffuse scattering is not optional — it is the difference between blind and seeing

| `diffuse_reflection` | valid paths (street canyon) |
|---|---|
| `False` (specular only) | **4** |
| `True` | **883,957** |

A specular-only wall is a mirror: it reflects the radar's own signal *away* from the sensor,
not back to it. A monostatic 77 GHz radar in a specular-only scene is therefore effectively
blind. This reproduces, far more starkly, what paper 2 measured for LiDAR (1 return specular
vs 8,417 diffuse). **Pitfall #2 confirmed.**

## 2. The Sionna angle convention for co-located TX/RX is NOT mirrored

The open question (NVlabs/sionna-rt#5) was whether Sionna flips azimuth when TX and RX sit at
the same point, as they do in a monostatic radar. If it did, every reflector would land on the
wrong side of the road and the paper's geometry ablation would be uninterpretable.

**It does not.** Measured on the *asymmetric* single-wall scene:

| convention | median detection → nearest real surface |
|---|---|
| assumed (`phi_r - yaw`) | **0.08 m** |
| mirrored (`-(phi_r - yaw)`) | 0.18 m — **2.33× worse** |

**The asymmetric scene is essential and the symmetric one is useless for this.** A street
canyon has walls on *both* sides, so it is left-right symmetric: there it scored 1.24 m
assumed vs 1.30 m mirrored — indistinguishable. A first attempt to settle the convention in
the canyon was therefore worthless, and a first attempt before that scored the radar against a
wall at −92°, i.e. *outside* its own ±90° field of view.

## 3. Sionna array layouts (RT 2.0.1)

```
paths.tau / paths.phi_r / paths.valid : (n_rx, n_tx, n_paths)
paths.objects / paths.interactions    : (depth, n_rx, n_tx, n_paths)
paths.a                               : a TUPLE (real, imag) of tensors, each
                                        (n_rx, n_rx_ant, n_tx, n_tx_ant, n_paths)
```

Two traps here, both of which would have been silent:

- **`n_tx` is 4, not 1.** The scene carries the WiFi APs as transmitters alongside our
  `radar_tx` (index 3). Flattening the arrays mixes the APs' **bistatic** paths into the
  radar's **monostatic** ray set — destroying the very geometry premise of the sensor.
- **`paths.a` is a tuple, not a tensor**, and it holds a gain *per RX antenna* of the
  scene's 4-element WiFi array. The radar's aperture is our own 16-element virtual MIMO ULA —
  a different array. We take the amplitude at the reference element and synthesize the array
  response from each path's angle of arrival. Using Sionna's per-antenna gains would impose
  the *WiFi* array's geometry on the radar.

## 4. ITU materials are not defined at 77 GHz — and the scene raises rather than lying

Setting `scene.frequency = 77e9` **raises**. Sionna's ITU material models are only defined
over their published ITU-R P.2040 bands:

| material | valid band | used by the scene? |
|---|---|---|
| concrete, metal | 1–100 GHz | **yes** |
| marble | 1–**60** GHz | no |
| brick | 1–**40** GHz | no |

Sionna re-evaluates *every registered* material when the frequency is set, so `marble` raises
even though nothing in the scene is made of it. `radar/sensor.py:retune_scene` freezes the
**unused** out-of-band materials (a material no ray hits has no physics to get wrong) and
deliberately leaves used ones alone, so a *used* out-of-band material still fails loudly.
Silently extrapolating ITU parameters past their validity band would fabricate the
permittivity of every surface the radar sees.

**The materials the scene actually uses (concrete, metal) are valid at 77 GHz.**

## 5. 61 % of the radar's rays are ground bounces, and they are dropped

| | street canyon |
|---|---|
| valid paths (diffuse) | 883,957 |
| ground-bounce paths **dropped** | **536,386 (61 %)** |
| rays fed to the detection chain | 347,571 |

The comparison plane is a 2-D bird's-eye view of **building footprints**, and the ground-truth
map contains facades only. A road return therefore has nothing to be scored against: it would
be charged against map accuracy as though the radar had hallucinated a wall in the middle of
the roadway. Paper 2's LiDAR model B drops floor hits for exactly this reason, and radar now
drops them the same way, so the two sensors' maps mean the same thing.

**This is a stated modelling choice, not a silent one.** A real automotive radar does see road
clutter; suppressing it is part of what real radar processing does.

## 6. Detection quality — the sensor sees real walls, in the right places

| scene | detections | best → nearest real surface | median |
|---|---|---|---|
| controlled_wall (asymmetric) | 1 | **0.08 m** | 0.08 m |
| street_canyon (symmetric) | 4 | **0.50 m** | 1.24 m |

---

## ⚠ The open issue sub-project 3 must resolve: detection density

**The sensor produces only ~1–5 detections per frame.** That is far too sparse for
scan-to-map ICP (which needs ≥ 3 points *and* enough coverage to constrain a pose), and it
would make the radar map nearly empty.

This is not obviously a bug — it is what CA-CFAR *does* in a diffuse scene. CFAR is designed
to find **point targets in noise**. Under full diffuse scattering a street canyon returns a
near-**continuum**: the local background against which a wall cell is compared *is the wall
itself*, so almost nothing exceeds its own neighbourhood by the CFAR margin.

Real radar odometry knows this. **CFEAR — the SOTA radar-odometry baseline we anchor
against — does not use CFAR at all; it takes the k-strongest returns per azimuth bin**,
precisely because radar targets are *extended*, not point-like.

So sub-project 3 must decide, and state, the front-end:

- **Option A — k-strongest per azimuth** (the CFEAR-style front-end). Matches the anchor,
  gives dense scans, and is defensible as "what radar odometry actually does".
- **Option B — keep CFAR** and accept sparse maps as a genuine finding about CFAR-on-diffuse.
- **Option C — both**, reporting CFAR as the detection-theoretic baseline and k-strongest as
  the odometry front-end.

Whichever is chosen, it **must be applied identically to every ablation cell** (A–D), or the
front-end becomes confounded with the physics it is supposed to isolate.

This is a genuine open question, discovered by building the thing, and it is exactly the kind
of problem the sub-project-2 credibility gate exists to catch before the ablation is built on
top of it.
