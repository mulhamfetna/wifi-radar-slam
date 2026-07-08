# Confidential comments to the Editor — IEEE Internet of Things Journal

*(For editorial consideration only; not shown to peer reviewers and not part of the
published article. Paste into the ScholarOne "Comments to the Editor" field.)*

---

Dear Editor,

Please consider the enclosed manuscript, **"Ambient WiFi as a Radar Replacement for
Automotive SLAM: A Physics-Based Feasibility Study,"** for publication in the *IEEE
Internet of Things Journal*.

**Fit and contribution.** The work sits squarely in IoT-J's scope: it repurposes the
ubiquitous, already-deployed WiFi/ISAC connectivity fabric (access points, roadside
units, connected vehicles) as an opportunistic perception sensor for automotive SLAM.
Using physics-based ray tracing we build a complete, open pipeline and establish, end
to end, what ambient sub-7 GHz WiFi can and cannot do: it *localizes* a moving vehicle
to centimetre level (and, with a joint 2-D delay–angle MUSIC front-end, to
oracle-sensing quality from realistic commodity CSI), while environment *mapping* is
bounded to a few metres. A central, non-obvious result is that this mapping bound is
**not** a bandwidth or aperture (resolution) limit — a 44× bandwidth increase (60 GHz)
and a 4× larger array both fail to move it — but a **path-discrimination** limit of
commodity CSI, which we then show is *learnable*.

**Two points I would like to flag transparently for editorial consideration:**

1. **Scope: this is a simulation-primary study.** The channel is ray-traced
   (Sionna RT), complemented by a real-CSI *front-end* proof-of-concept on measured
   Intel 5300 and Broadcom nexmon captures. A full outdoor, vehicle-mounted
   validation is not included because, to our knowledge, **no public outdoor/vehicular
   WiFi-CSI dataset exists** — which is itself part of the gap the paper identifies and
   which we partially address by releasing the first ray-traced such dataset
   (WiFiSLAM-Sim). We state this limitation and the real-capture next step explicitly
   in the manuscript, and would welcome reviewers with WiFi-sensing / CSI-measurement
   expertise who can assess the modeling fidelity.

2. **Some headline results are rigorous negative/nuanced findings by design.** The
   demonstrations that 60 GHz bandwidth and a larger array do *not* improve realistic
   mapping are intentional, evidence-backed results that reframe the problem, not
   omissions or shortcomings. We would be grateful if this framing is conveyed so the
   contribution is evaluated as a characterization of a limit and its cause, followed
   by a first constructive step, rather than as an incomplete positive result.

**Open science / single-blind note.** In the interest of reproducibility, the complete
pipeline, all configurations, result artifacts, and the WiFiSLAM-Sim dataset are
publicly available on GitHub and archived on Zenodo (concept DOI
10.5281/zenodo.21247288), under AGPL-3.0. As IoT-J review is single-blind this is
consistent with policy, but we flag that author identity is discoverable via the
repository. The manuscript itself has **not** been posted as a preprint.

**Declarations.** This is original work, has not been published previously, and is not
under consideration at any other venue. There are no conflicts of interest. The work
is single-authored and received no external funding. Suggested reviewer expertise:
passive WiFi radar / CSI sensing, integrated sensing and communication (ISAC),
multipath-based / RF-SLAM, and automotive/vehicular perception.

Thank you for considering the manuscript.

Sincerely,
Mulham Fetna (independent researcher)
contact@mulhamfetna.com · ORCID 0009-0006-4432-798X
