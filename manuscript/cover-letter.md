# Cover letter — IEEE Access submission

**To:** The Editors, *IEEE Access*
**Article:** *Ambient WiFi as a Cheap Radar for Automotive SLAM: Centimetre Localization, a
Phantom Ceiling on Mapping, and What That Ceiling Is — in Simulation and on $30 of Silicon*
**Author:** Mulham Fetna, Department of Mechatronics Engineering, University of Aleppo,
Aleppo, Syria · ORCID 0009-0006-4432-798X · contact@mulhamfetna.com

---

Dear Editors,

I am pleased to submit the above article for consideration in *IEEE Access*.

**What the article asks and answers.** It asks whether ambient sub-7 GHz WiFi, received on a
moving vehicle and processed with commodity channel state information (CSI), can replace radar or
LiDAR as a front-end for simultaneous localization and mapping. The answer is a boundary rather
than a yes or no, and that boundary is the contribution: WiFi localizes to centimetre level — at
parity with a mid-range LiDAR for roughly one percent of its cost — while WiFi *mapping* hits a
hard ceiling. The article locates that ceiling precisely: approximately 89 % of realistic-CSI
detections correspond to no real propagation path, accompanied by a several-metre range bias. It
is a front-end and geometry limit, not a path-discrimination limit, not a resolution limit, and
not repairable by a learned filter.

**Why it should interest a broad readership.** Running the same substrate against real LiDAR
(KITTI) and a simulated 77 GHz FMCW radar shows the ceiling is neither carrier-specific nor
WiFi-specific: the carrier does not matter, geometry does — a monostatic, on-vehicle transmitter
reduces the phantom rate from 18.2 % to 0.1 % — and wider bandwidth makes phantoms worse. The
article also reports a cost analysis (the WiFi package is 84–600× cheaper than the simulated LiDAR
class), a symmetric fusion study yielding a design rule (fusion reinforces at sensor parity and
degrades the stronger sensor eightfold when mismatched), a robustness characterization, and a
hardware demonstration on two $30 ESP32-S3 boards.

**Reproducibility.** All code, configurations, result data, and the ESP32 firmware are released
openly under AGPL-3.0 at https://github.com/mulhamfetna/wifi-radar-slam and archived with
persistent DOIs. Every reported number is regenerable from that artifact.

**Disclosure of similarity to my own prior work.** This article consolidates and substantially
extends earlier **unpublished** manuscripts of my own — a physics-based feasibility study and a
WiFi-versus-LiDAR comparison. Those drafts are cited in the article and are publicly readable in
the open-source repository above, so an automated similarity check may match text against them. To
be explicit: they are my own unpublished work, not prior publications, and this submission is a
consolidated and considerably extended treatment that adds the radar comparison, the robustness
experiments, the hardware validation, and a corrected analysis. I note also that a numerical result
reported in the earlier feasibility draft did not reproduce on re-verification; the corrected value
(absolute trajectory error 0.098 ± 0.028 m) is the one used throughout this article.

**Originality and exclusivity.** The work is original, is not under consideration for peer review
at any other journal, and has not been published elsewhere.

Thank you for considering this submission. I would be glad to provide any further information.

Sincerely,
**Mulham Fetna**
ORCID 0009-0006-4432-798X · contact@mulhamfetna.com
