# Potential future papers — open threads from papers 1 & 2

A living record of research directions this work has opened. Each is a *candidate*, not a
commitment. Update as threads are taken up or closed.

Status legend: 🟢 strong / ready · 🟡 viable, needs scoping · 🔵 blocked on something

---

## 🟢 P-A. Fixing the front-end: phantom suppression and range-bias correction

**The gap.** Paper 2 proved the WiFi mapping ceiling is *not* path discrimination but a
**front-end** limit: ≈89 % of MUSIC detections correspond to **no real propagation path**,
and a **6.45 m median range bias** (far beyond the 0.94 m resolution limit at 160 MHz)
corrupts the rest. It also proved that *selecting* among detections cannot fix this — a
heuristic, a random forest and an MLP all fail identically.

**The paper.** Act where the ceiling actually is. Two formulations:
- **Regress, don't classify** — a learned model that *corrects* estimated delay/AoA (or
  rejects phantoms) at the estimator output.
- **Bypass the estimator** — end-to-end CSI → geometry, the form the successful RF-sensing
  literature already takes (transformer CSI→3-D point cloud; U-Net/ViT RF→outdoor geometry).

**Why it is strong.** Paper 2 *names this as the next step* and hands it a precise target:
the geometry is recoverable (76.7 % of correctly-matched facade paths triangulate within 1 m
on the street), so the ceiling is the front-end, not the physics. A positive result here
would overturn paper 2's "mapping No".

**Assets in hand.** WiFiSLAM-Sim dataset, the isolation experiment
(`isolate_mapping_floor.py`), the full ladder, all six metrics, and the LiDAR envelope as
the target to beat.

---

## 🟢 P-B. Confidence-adaptive WiFi+LiDAR fusion

**The gap.** Paper 2's fusion result is **conditional on sensor parity**: tight fusion beats
both solo sensors in 3 of 4 configurations, but *degrades the stronger sensor by 8×* when the
pair is badly mismatched (street/LiDAR-A: 0.027 m alone → 0.218 m fused). We deliberately did
**not** tune the weighting, which is exactly what exposed the condition.

**The paper.** Weight each modality by its **online-estimated reliability** (e.g. particle
spread, ICP residual, detection-count / phantom-rate proxies) so a weak sensor cannot drag
down a strong one. Then re-run the 4-configuration matrix and show the regression is removed.

**Why it is strong.** Small, self-contained, and it converts a *reported limitation* into a
contribution. The cost framing carries over directly: WiFi is a +0.5 % addition, so a fusion
that is *safe* under mismatch is immediately deployable.

---

## 🟡 P-C. Real on-vehicle CSI

**The gap.** Everything in papers 1–2 is ray-traced. The single most valuable missing
measurement is whether the **89 % phantom rate survives contact with a real channel** — it is
currently a property of MUSIC operating on *simulated* CSI.

**The paper.** Collect real vehicular CSI (commodity NIC / nexmon / ESP32), run the same
front-end, and measure the phantom rate and range bias against a LiDAR-derived ground truth.

**Why it is blocked-ish.** Needs hardware, a vehicle, and a LiDAR for ground truth. Highest
scientific value, highest logistical cost. Would validate (or demolish) the central mechanism
of paper 2.

---

## 🟡 P-D. WiFi vs automotive RADAR *(candidate paper 3 — see below)*

**The idea.** Re-run the *same* comparison substrate — same scenes, same ground truth, same
six metrics, same SLAM back-ends, same cost model — but with an **automotive radar** baseline
in place of LiDAR, compared against WiFi.

**Why it fits the pipeline unusually well.** Radar is **electromagnetic**, so Sionna RT
models it *natively* — unlike LiDAR, which required the diffuse-scattering workaround
(Model B) to make an EM ray tracer behave optically. A 77 GHz FMCW radar is arguably a more
honest use of this simulator than the LiDAR models were.

**Why it is interesting.** It completes the sensor triangle. Paper 1 framed WiFi as a *radar*
replacement but never simulated an actual automotive radar; paper 2 compared against LiDAR.
Radar is also the *closest* competitor to WiFi — both are RF, both are cheap, both are
bandwidth-limited — so the comparison is far less lopsided than WiFi-vs-LiDAR and the cost
argument becomes genuinely contested (radar is ~$50–200, not $8–24 k).

**Open question.** If WiFi and radar are physically similar, is WiFi's advantage merely that
the transmitter is free (ambient), while radar must supply its own? That would be a sharp,
quotable thesis.

---

## Assets any of these inherit (all on `main`)

`lidar/` (LidarConfig, Scan, KD-tree multi-core ICP, scan-to-map SLAM with adaptive motion
model, three sensor models, six WiFi-comparable metrics) · `fusion.py` (symmetric tight PF +
loose baseline) · `cost.py` (sourced prices, cost-normalized $·m and $/IoU) ·
`map_filter.py` (learned filter ladder + `run_slam(map_filter=)` hook) · WiFiSLAM-Sim ·
the two ray-traced scenes with footprint ground truth · 85 tests.
