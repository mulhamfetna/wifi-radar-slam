"""77 GHz FMCW automotive-radar sensor model (paper 3).

`config` and `processing` are pure NumPy/SciPy and test locally. `sensor` is the only
module that touches Sionna, and it imports it lazily inside methods — so importing this
package never requires Sionna.
"""
from .config import RadarConfig, RADAR_77G_4G, RADAR_77G_160M, WIFI_5G2_160M

__all__ = ["RadarConfig", "RADAR_77G_4G", "RADAR_77G_160M", "WIFI_5G2_160M"]
