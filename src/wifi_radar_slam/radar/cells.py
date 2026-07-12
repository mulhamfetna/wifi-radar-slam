"""The ablation's cells -- the single place paper 3's physics is defined.

Each cell runs the IDENTICAL detection chain (beat -> range FFT -> azimuth beamforming ->
CA-CFAR). Only the physics changes, and only ONE variable at a time, so each rung of the ladder
isolates exactly one mechanism:

    A -> B   GEOMETRY   (bistatic ambient WiFi  ->  monostatic active WiFi)
    B -> C   CARRIER    (5.2 GHz  ->  77 GHz, bandwidth held at 160 MHz)
    C -> D   BANDWIDTH  (160 MHz  ->  4 GHz, carrier held at 77 GHz)

If any step ever changed two things at once the decomposition would be meaningless -- which is
why the tests pin one-variable-at-a-time rather than merely checking the values.

The fifth row -- WiFi + joint 2-D MUSIC, papers 1-2's front-end -- is deliberately NOT a Cell:
it does not use this chain. It is run separately (experiments/run_ablation.py) precisely so the
superresolution-vs-FFT axis stays VISIBLE rather than silently confounded with the physics.
"""
from __future__ import annotations
from dataclasses import dataclass

from .config import RadarConfig, RADAR_77G_4G, RADAR_77G_160M, WIFI_5G2_160M


@dataclass(frozen=True)
class Cell:
    key: str
    label: str
    config: RadarConfig
    monostatic: bool          # False -> bistatic (an ambient AP illuminates; ellipse solve)
    front_end: str            # "cfar" for every cell -- the chain is held fixed
    isolates: str


CELLS: dict[str, Cell] = {
    "A": Cell("A", "WiFi baseline (bistatic, ambient)", WIFI_5G2_160M,
              monostatic=False, front_end="cfar", isolates="-- (the baseline)"),
    "B": Cell("B", "WiFi monostatic (active)", WIFI_5G2_160M,
              monostatic=True, front_end="cfar", isolates="GEOMETRY (A->B)"),
    "C": Cell("C", "Radar narrowband (77 GHz, 160 MHz)", RADAR_77G_160M,
              monostatic=True, front_end="cfar", isolates="CARRIER (B->C)"),
    "D": Cell("D", "Radar full (77 GHz, 4 GHz)", RADAR_77G_4G,
              monostatic=True, front_end="cfar", isolates="BANDWIDTH (C->D)"),
}
