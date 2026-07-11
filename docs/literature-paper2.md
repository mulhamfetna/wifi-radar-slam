# Paper 2 — literature & market synthesis (WiFi vs LiDAR for SLAM)

Deep-research pass (2026-07-11): 5 search angles → 20 sources → 83 extracted claims →
25 adversarially verified (3-vote, ≥2/3 to survive). Cost claims were sourced from
real market articles but fell outside the top-25 verification cut, so they are marked
**sourced, not vote-verified** below. This document feeds paper 2's related-work
section and the RQ5 cost model.

## Novelty gap (the headline, high confidence)

**No published work demonstrates commodity-CSI WiFi sensing as a *validated drop-in
LiDAR replacement* for on-vehicle / outdoor automotive SLAM, with a head-to-head
WiFi-vs-LiDAR accuracy comparison.** The two nearest works each miss on a distinct
axis, and everything else positions WiFi as an *augmentation* to a primary ranging
sensor (camera or LiDAR):

- **P2SLAM** (T-RO 2022) — standalone WiFi/**CSI** SLAM, but **indoor only** and
  benchmarked against *visual* SLAM, not LiDAR.
- **Radio-fingerprint SLAM** (IEEE Pervasive 2023) — WiFi/LTE as the *primary*
  modality and demonstrated **outdoors on a vehicle**, but uses **RSS fingerprinting
  from smartphones (not CSI)**, a slow research UGV (not road-speed), and its best
  accuracy needs **Radio+LiDAR fusion**.

Paper 2 occupies the open cell: **on-vehicle, outdoor, commodity-CSI, head-to-head
WiFi-vs-LiDAR** (our A/B LiDAR envelope + KITTI real-LiDAR anchor). This reinforces
paper 1's novelty gap and extends it from the radar framing to the LiDAR-replacement
framing.

## Thread 1 — Novelty / prior WiFi-SLAM (verified)

| Work | What it is | Why it doesn't close the gap | Cite |
|------|-----------|------------------------------|------|
| **P2SLAM** | Standalone WiFi SLAM: two-way AoA bearings from COTS **CSI** (not RSSI) in a GTSAM GraphSLAM backend. Median **26.9 cm** tracking (90th 54.7 cm), 1.28° orientation, ≥6× better than odometry, on par with SOTA visual SLAM. | Indoor only (25×30 m, 50×40 m, 5/7 fixed APs, Turtlebot2). Baseline is **camera (RTAB-Map), not LiDAR**. Not automotive. | ieeexplore 9691786; wcsng p2slam.pdf |
| **Radio-fingerprint SLAM** | WiFi/LTE radio features as the **primary** sensing modality for AVs; standalone Radio SLAM (<10 m) + a Radio+LiDAR fusion mode generating an occupancy map. Demonstrated **outdoor + indoor + semi-indoor** on a Clearpath Husky. | **RSS fingerprinting via 5 smartphones**, not CSI. Slow research **UGV**, not an automobile at road speed. Best accuracy needs **LiDAR fusion**. | arXiv 2305.13635 |
| **ViWiD** | WiFi+visual dual-layer; WiFi *replaces compute-intensive loop closure*. 4.3× compute / 4× memory reduction vs SOTA visual & LiDAR SLAM, on-par accuracy over 1500+ m. | WiFi augments **vision**; indoor; no WiFi-vs-LiDAR comparison. | arXiv 2209.08091 |
| **RSS-augmented visual SLAM** | WiFi RSS improves visual SLAM ~11%. | Complement to vision; no LiDAR; no comparison. | arXiv 1903.06687 |
| **DLoc / LocAP** | WiFi localization layered on a LiDAR/RGBD RTAB-Map occupancy map. | LiDAR builds the map; WiFi adds positioning. Indoor. | wcsng NSDI poster |

## Thread 2 — WiFi/RF + LiDAR fusion (RQ4, verified)

Well-populated; **fusion consistently beats either single modality**, but LiDAR stays
in the winning stack (so these are RQ4 references, not novelty threats):

- **Two-level graph SLAM** (arXiv 2206.08733): WiFi fingerprint-sequence loop closures
  + LiDAR scan-matching. **WiFi-only 2.7 m → 0.88 m** with LiDAR fusion. Indoor carpark, Husky.
- **EKF WiFi-RSSI(DNN)+LiDAR-Gmapping+IMU** (arXiv 2509.23118): fused **0.24–0.38 m**
  vs WiFi-only up to **1.34 m** vs LiDAR/IMU-only **0.62–2.88 m**, across all paths.
- **Laser SLAM + WiFi fingerprint** re-localization (PMC7570627).
- **Radio+LiDAR** occupancy mapping (arXiv 2305.13635).

These are the concrete anchors for paper 2's RQ4 (does fusion lift accuracy — and by how much).

## Thread 3 — Deep-learning RF/WiFi sensing (RQ2, verified)

Strong and **directly supportive** of the cheaper-than-LiDAR framing (authors use LiDAR
only as ground truth):

- **CSI → 3D point clouds** via a transformer on temporal CSI amplitude+phase; authors
  frame WiFi as an attractive alternative to more expensive, power-intensive sensors
  (Ouster LiDAR used only as GT); ICP RMSE ~0.01 m. arXiv 2410.16303.
- **Dense human pose from commodity WiFi** (DensePose UV, 24 regions, multi-subject).
  arXiv 2301.00250.
- **RF-Pose** (WiFi-band FMCW, teacher-student cross-modal supervision): 2D skeletal
  pose from RF alone, **through walls** — AP 62.4 (visible) / 58.1 (through-wall) vs
  vision 68.8. CVPR 2018 (Zhao et al.).
- **Outdoor geometry from RF propagation** via U-Net and CLIP+ViT (WAIR-D synthetic
  dataset) — explicitly positioned as an alternative to vision/LiDAR reconstruction.
- **NeRF²**: RF radiance field for indoor localization / 5G MIMO; synthetic-RF
  "turbo-learning" ~+50%.

Precedent that DL can lift RF sensing toward geometric mapping — the basis for paper 2's
RQ2 (can DL close the WiFi mapping-coverage gap). None targets on-vehicle outdoor SLAM.

## Thread 4 — Cost (RQ5) — **sourced, not vote-verified**

Extracted from market articles (verify against primary sources when writing the cost
model). Confirms the envelope you chose (high-end → cheap solid-state):

**Automotive/robotics LiDAR**
- Legacy high-end spinning: **~$75–80 k** (Velodyne HDL-64 class).
- Budget 2D scanner: **~$99** (Slamtec RPLIDAR A1).
- Emerging automotive **solid-state**: **~$200–600** — MicroVision Movia S ~$200
  (long-term $100 goal), Hesai automotive-grade <$200, Luminar Halo ~$500 (2026),
  general series solid-state $500–600. Sources: AOL "$200 Lidar Could Reshuffle Auto
  Sensor Economics"; Electronic Design "$500 Price Point".

**Commodity WiFi-CSI receiver**
- **ESP32**: ~**$5–15** (CSI via Espressif IDF).
- **Raspberry Pi 4 + nexmon_csi**: ~**$35–75**.
- Intel 5300 (iwl5300 CSI tool): legacy commodity NIC.

**Implication for RQ5.** The cost gap is **enormous vs high-end/legacy LiDAR
(~1000–10000×)** and remains **large vs mid solid-state (~10–100×)**, but **narrows to
single-digit×** against the very cheapest emerging solid-state (~$100–200). The
envelope framing is exactly right — and honest: the WiFi advantage is decisive on
price, most dramatically against the LiDAR grades that actually deliver the mapping
quality WiFi cannot yet match. **No prior sensor-cost/BOM comparison study for
WiFi-vs-LiDAR SLAM surfaced** — the cost-parity analysis itself appears novel.

## Net implications for paper 2

1. **Novelty is defensible** — the on-vehicle/outdoor/commodity-CSI WiFi-vs-LiDAR cell
   is open; frame paper 2 explicitly against P2SLAM (indoor/CSI) and radio-fingerprint
   SLAM (outdoor/RSS/needs-LiDAR).
2. **RQ4 fusion** has strong prior art showing fusion > single modality — position our
   fusion study as the *first* on-vehicle WiFi+LiDAR-envelope fusion with a cost lens.
3. **RQ2 DL** is well-precedented (CSI→point-cloud, RF→outdoor geometry) — cite as
   feasibility basis for closing the mapping gap.
4. **RQ5 cost** — the comparison study itself looks novel; source the prices to primary
   vendor/press citations with dates in the cost-model spec.
