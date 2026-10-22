# Cover letter

**To:** The Editor, *Pervasive and Mobile Computing*\
**Article:** *Can Ambient WiFi Support Vehicular Multipath SLAM? Virtual-Anchor Localization,
Commodity Front-End Limits, and a Delay-Convention Pitfall in Ray-Traced Evaluation*\
**Author:** Mulham Fetna, Department of Mechatronics Engineering, University of Aleppo,
Aleppo, Syria · ORCID 0009-0006-4432-798X · contact@mulhamfetna.com

---

Dear Editor,

I submit the above research article for consideration in *Pervasive and Mobile Computing*.

**What the article answers.** Commodity WiFi receivers are cheap and ambient access points are
everywhere, so WiFi is an attractive aid for vehicle localization. The article separates two
questions that are usually mixed: whether the multipath geometry in WiFi channels is informative
for a moving vehicle, and whether a commodity front-end can extract it. Using ray-traced channels
in three outdoor scenarios, wheel-grade odometry as the baseline, paired LiDAR runs, and a
virtual-anchor FastSLAM back-end, it shows that exact multipath parameters reduce odometry drift
by 66–79 %, while joint MUSIC on a four-element array returns too few facade reflections and too
many persistent false detections, making WiFi-aided SLAM worse than odometry alone. It quantifies
why (composition of the detections, front/back folding, bandwidth independence), shows that
access-point positions must be known to sub-metre accuracy, and identifies a delay-convention
pitfall that affects any study synthesizing CSI from a ray tracer.

**Why it fits the journal.** The article is about what commodity wireless sensing can and cannot
deliver for localization on a mobile platform, and it gives practitioners concrete requirements:
array geometry that resolves front/back ambiguity, front-ends with model-order control, anchor
promotion based on geometric consistency, and surveyed access points.

**Relationship to the author's earlier work, and corrections.** This article supersedes two of my
earlier **unpublished** manuscripts on the same simulation framework. They are publicly readable in
the open repository below, so a similarity check may match against them. On re-verification I
found that those analyses drove the estimator with ground-truth odometry, used a measurement update
that carried no position information, and read first-arrival-referenced CSI delays as absolute path
lengths. Their localization-parity, robustness, fusion and cost results are withdrawn, and the
article documents the delay-convention error and its effect openly (Section 5 and the Discussion).
A consolidated version of the earlier work was submitted to *IEEE Access* and rejected before
review; the present article is a substantially different, corrected study.

**Reproducibility.** All code, configurations, regenerable ray-traced channel caches, result files
and ESP32 firmware are archived on Zenodo (release v1.0.0, https://doi.org/10.5281/zenodo.23560036)
under AGPL-3.0, with development at https://github.com/mulhamfetna/wifi-radar-slam.
Every number in the article is produced by the scripts listed in the repository's
`PROJECT.md`.

**Originality and exclusivity.** The work is original, has not been published, and is not under
consideration at any other journal. There are no competing interests and no funding to declare.

Sincerely,
**Mulham Fetna**
ORCID 0009-0006-4432-798X · contact@mulhamfetna.com
