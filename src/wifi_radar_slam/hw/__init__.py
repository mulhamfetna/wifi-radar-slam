"""Paper 4 — the ESP32 CSI hardware sensor (the static bench).

Pure NumPy/SciPy, fully testable on **synthetic CSI** — no hardware required to develop
or CI. This is rung 0 of the ladder in ``docs/paper4-restart-static-bench.md``: prove the
delay-domain pipeline recovers a tap we injected ourselves, before a single ESP32 is bought.

Modules
-------
config   : ``CSIConfig`` — the ESP32 HT40 physical constants (128 subcarriers, 312.5 kHz,
           40 MHz, the 3-bin DC notch, the 11 edge guard bins). All derived resolution
           numbers (25 ns cell, 7.5 m path, 3.75 m monostatic range) live here.
synth    : ``synth_csi`` — build CSI from a known set of (complex amplitude, delay) taps,
           with the notch, guard bins, an S-shaped receiver distortion, per-packet random
           common phase + STO, and 8-bit quantisation. The ground truth for rung 0.
delay    : the pipeline — de-ramp (STO) -> LOS-reference -> interpolate the notch -> window
           -> zero-pad -> IFFT -> |CIR|. Plus ``differential`` (plate-in minus plate-out).
           THE ORDER IS NOT OPTIONAL — see Part 5 of the design doc.
detect   : ``cfar_1d`` — a 1-D CA-CFAR on the delay profile using the SAME threshold
           formula as ``radar.processing.cfar_2d`` (alpha = N(Pfa^(-1/N)-1)), so the
           hardware detector is the same family as the simulation. Comparability is the
           point: a different detector would confound the hardware result with an algorithm
           change.
"""
from .config import CSIConfig, ESP32_HT40

__all__ = ["CSIConfig", "ESP32_HT40"]
