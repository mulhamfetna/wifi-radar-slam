# Corrected results (2026-10): three pipeline defects and what survives them

This document is the authoritative record of the October 2026 re-verification of the
WiFi-SLAM pipeline. It supersedes the localization, mapping-mechanism and robustness numbers
in `results-v1.md`, `results-paper2.md`, `results-ap-sensitivity.md` and
`results-blockage-robustness.md`, which are kept unchanged as the historical record.

Every number below is regenerated from the cached ray-traced channels by:

```sh
export CUDA_VISIBLE_DEVICES=        # Sionna's CUDA backend needs sm_70+
.venv/bin/python experiments/cache_channels.py configs/controlled_music_joint.yaml
.venv/bin/python experiments/cache_channels.py configs/street_metal_music.yaml
.venv/bin/python experiments/cache_channels.py configs/street_metal_music_scurve.yaml \
    data/cache_street_canyon_metal_scurve.npz
.venv/bin/python experiments/make_music_detections.py       # results/dets/ (gitignored)
.venv/bin/python experiments/run_corrected_eval.py          # results/corrected_eval.json
.venv/bin/python experiments/run_robustness_corrected.py    # results/robustness_corrected.json
.venv/bin/python experiments/run_lidar_noisy_odom.py geo    # results/lidar_noisy_odom_geo.json
```

## The three defects

### D1. Ground-truth odometry

Every SLAM caller built its motion input with `velocity_from_poses(ground_truth)`. The WiFi
particle filter, the LiDAR ICP prior, fusion and both robustness experiments therefore drove
along the true trajectory, and trajectory error measured only the filter's own process noise.
With **zero** WiFi detections the legacy filter scored 0.049 ± 0.015 m on the nominal run, the
same as the published oracle WiFi figure.

*Fix:* `wifi_radar_slam.odometry` (1 % scale error, 0.5° initial heading error, 0.2°/√s heading
random walk, 0.05 m/s white noise). Dead reckoning is reported beside every trajectory number,
and the WiFi and LiDAR runs use identical odometry draws (`default_rng(1000 + seed)`).

### D2. Self-referential measurement update

`slam/particle_filter.run_slam` triangulated each reflector from the filter's own mean pose and
then scored every particle against that same reflector. No landmark persisted between frames,
so a detection carried no information about absolute position. With noisy odometry and
**perfect** synthetic detections the legacy filter did not beat dead reckoning
(0.459 vs 0.444 m); the replacement does (0.109 m).

*Fix:* `slam/fastslam.py`, a virtual-anchor FastSLAM. The landmark is the access point
mirrored in the reflector, which is fixed in the world; the bistatic length is the range to
it. Landmark promotion is explicit (5 associations), stale tentative anchors are pruned.
A regression test asserts that WiFi beats dead reckoning under noisy odometry.

### D3. Excess delay read as absolute path length

Sionna RT's `Paths.cfr()` defaults to `normalize_delays=True`, so the simulated CSI carries
delays relative to the **first arrival**. That is realistic, because commodity CSI has no
absolute time of flight. The back-end, however, treated those excess lengths as absolute
bistatic path lengths, under-ranging every detection by the line-of-sight distance |AP − p|.
The radar chain reads `paths.tau` directly and is unaffected.

*Fix:* detections stay in the excess convention and are converted with the known AP position,
ρ = c·τ_ex + |AP − p| (per particle inside the filter; at the true pose for
ground-truth-pose scoring). The simulator now passes `normalize_delays=True` explicitly, with
the convention documented at the call.

*What D3 explains:* the published **6.45 m "range bias"** (controlled scene) is D3 entirely,
and so is most of the **≈89 % "phantom" rate**. The bandwidth- and aperture-independence of the
published mapping floor follows too: no amount of resolution corrects an offset equal to the
line-of-sight distance.

## Results

Scenes: controlled metal wall (120 frames, 30 m), metal street canyon (240 frames, 60 m,
straight) and the same canyon on a lane-change S-curve. 5.2 GHz, 4-element ULA, 20 dB SNR,
joint 2-D MUSIC with 3 paths per AP. Mean ± std over 10 seeds at 160 MHz (5 seeds for the
bandwidth sweep). Source: `results/corrected_eval.json`, `results/lidar_noisy_odom_geo.json`.

### R1. Localization under realistic odometry (ATE, m)

| Scene | Dead reckoning | WiFi oracle | WiFi MUSIC | MUSIC, two-branch | LiDAR-A |
|---|---|---|---|---|---|
| Controlled wall | 0.211 ± 0.109 | **0.072 ± 0.026** | 0.340 ± 0.123 | 0.390 ± 0.110 | 0.172 ± 0.117 |
| Street canyon | 0.470 ± 0.260 | 0.101 ± 0.045 | 1.202 ± 0.241 | 0.527 ± 0.167 | **0.027 ± 0.001** |
| Street, S-curve | 0.472 ± 0.259 | 0.100 ± 0.047 | 1.065 ± 0.199 | 0.488 ± 0.266 | **0.028 ± 0.000** |

- **The geometry works.** With exact multipath parameters (oracle, excess-delay convention,
  the same filter), ambient WiFi cuts wheel-odometry drift by 66 % (controlled) and 79 %
  (both street runs), confirming 3 and 7 virtual anchors respectively. In the controlled
  scene it beats LiDAR-A, which cannot constrain motion along a single wall.
- **Commodity sensing does not deliver it.** With joint MUSIC on a 4-element ULA the same
  filter is *worse than dead reckoning* in every scene. It confirms 28 (controlled) and
  101-116 (street) anchors where the scene has 3 and 7: many unexplained detections persist
  from frame to frame and pass the 5-observation promotion test. The two-branch azimuth model
  removes the excess error in the street but only reaches dead-reckoning parity.
- **The published parity claim does not survive.** The as-published chain (legacy filter,
  excess read as absolute) gives 0.122 ± 0.045 m (controlled) only because it was fed the
  true velocity; with the same noisy odometry it gives 0.234 ± 0.086 m, no better than dead
  reckoning.

### R2. What the realistic front-end actually returns (160 MHz, manuscript matching rule)

| | Controlled | Street | S-curve |
|---|---|---|---|
| "Phantom" rate, as published | 88.8 % | 89.7 % | 88.5 % |
| "Phantom" rate, corrected | 55.1 % | 65.5 % | 66.6 % |
|  of which unexplained | 35.8 % | 34.6 % | 32.4 % |
|  of which ULA front/back folded | 19.3 % | 30.8 % | 34.1 % |
| True facade path | 15.7 % | 15.5 % | 15.2 % |
| Other real path (LOS, floor, multi-bounce) | 29.2 % | 19.0 % | 18.2 % |
| Median facade range error, as published | −6.35 m | −2.19 m | −6.62 m |
| Median facade range error, corrected | **+0.03 m** | −0.06 m | −0.06 m |

The "range bias" was the delay convention (D3). Of the corrected "phantoms", roughly a third
to a half are real paths reported at the mirrored azimuth, which a single ULA cannot resolve;
about a third of all detections remain genuinely unexplained.

### R3. Mapping under ground-truth poses (occupancy IoU, 1 m cells)

| | Controlled | Street | S-curve |
|---|---|---|---|
| Oracle | 0.767 | 0.075 | 0.075 |
| MUSIC, as published | 0.000 | 0.000 | 0.001 |
| MUSIC, corrected | **0.349 ± 0.017** | 0.031 ± 0.002 | 0.028 ± 0.003 |

Realistic WiFi does map the controlled wall once delays are interpreted correctly; the
published "IoU ≈ 0" was D3. In the street canyon even the oracle covers little of the footprint
ground truth (only illuminated facades can be mapped), so IoU there measures coverage more than
accuracy.

### R4. Bandwidth (MUSIC, corrected)

| Bandwidth | Controlled phantom | Controlled GT-pose IoU | Street phantom | Street GT-pose IoU |
|---|---|---|---|---|
| 20 MHz | 52.7 % | 0.474 | 65.9 % | 0.012 |
| 40 MHz | 49.9 % | 0.341 | 69.8 % | 0.020 |
| 80 MHz | 53.3 % | 0.331 | 64.4 % | 0.027 |
| 160 MHz | 55.1 % | 0.349 | 65.5 % | 0.031 |

The phantom rate is flat in bandwidth: the limit is the 4-element aperture and model mismatch,
not range resolution.
