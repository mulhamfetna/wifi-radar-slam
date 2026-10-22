# WiFi-Radar-for-SLAM

[![tests](https://github.com/mulhamfetna/wifi-radar-slam/actions/workflows/tests.yml/badge.svg)](https://github.com/mulhamfetna/wifi-radar-slam/actions/workflows/tests.yml)
[![DOI](https://zenodo.org/badge/1292636094.svg)](https://zenodo.org/badge/latestdoi/1292636094)
[![License: AGPL v3](https://img.shields.io/badge/License-AGPL_v3-blue.svg)](LICENSE)

**Can ambient WiFi help a vehicle localize and map?** A ray-traced study (Sionna RT) with
wheel-grade odometry baselines, a virtual-anchor FastSLAM back-end, a LiDAR comparison, and
ESP32-S3 hardware.

- With exact multipath parameters, ambient WiFi cuts odometry drift by 66–79 %.
- A commodity four-element MUSIC front-end does not deliver that geometry: only about 15 % of its
  detections are wall reflections, and WiFi-aided SLAM ends up worse than odometry alone.
- Access-point positions must be known to sub-metre accuracy.
- Ray-traced CSI is referenced to the first arrival, like real hardware; reading it as absolute
  path length creates a false range bias. Earlier versions of this work made that error, and
  their localization-parity and "phantom ceiling" claims are withdrawn.

**Start with [`PROJECT.md`](PROJECT.md)**: the single source of truth for goal, status, results,
reproduction, and history. The paper is in [`manuscript/`](manuscript/).

## Citation

See [`CITATION.cff`](CITATION.cff). Author: Mulham Fetna ·
[ORCID 0009-0006-4432-798X](https://orcid.org/0009-0006-4432-798X) · contact@mulhamfetna.com

## License

AGPL-3.0-or-later. See [`LICENSE`](LICENSE).
