"""Schema + loader for the ray-traced outdoor/vehicular WiFi-CSI dataset.

No public outdoor/vehicular WiFi-CSI dataset exists; this packages the Sionna-RT
simulated scenario as an openly-reusable artifact for SLAM and path-discrimination
research. A record bundles, for one vehicular pass:

- ``csi``          (n_frames, n_ap, n_rx_ant, n_subcarriers) complex64 — noisy CSI
- ``poses``        (n_frames, 3) float — ground-truth vehicle x, y, yaw
- ``ap_positions`` (n_ap, 3) float — access-point coordinates
- ``gt_map``       (M, 2) float — ground-truth facade footprints
- ``paths``        (P, 9) float — a flat oracle path table, one row per ray-traced
                   path: [frame, ap, delay_s, phi_r, theta_r, n_bounce,
                          first_interaction_type, object_id, is_floor].
                   ``n_bounce``/``first_interaction_type``/``is_floor`` are the
                   labels enabling learned LOS/reflection/floor discrimination.
- ``meta``         JSON string: carrier, bandwidth, subcarriers, antennas, scene,
                   SNR, units, generator, license.
"""
from __future__ import annotations
from dataclasses import dataclass
import json
import numpy as np

PATH_COLUMNS = ["frame", "ap", "delay_s", "phi_r", "theta_r", "n_bounce",
                "first_interaction_type", "object_id", "is_floor"]


@dataclass
class CsiDataset:
    csi: np.ndarray
    poses: np.ndarray
    ap_positions: np.ndarray
    gt_map: np.ndarray
    paths: np.ndarray            # (P, 9), columns = PATH_COLUMNS
    meta: dict

    def save(self, path: str) -> None:
        np.savez_compressed(path, csi=self.csi, poses=self.poses,
                            ap_positions=self.ap_positions, gt_map=self.gt_map,
                            paths=self.paths, meta=json.dumps(self.meta))

    @staticmethod
    def load(path: str) -> "CsiDataset":
        z = np.load(path, allow_pickle=False)
        return CsiDataset(csi=z["csi"], poses=z["poses"],
                          ap_positions=z["ap_positions"], gt_map=z["gt_map"],
                          paths=z["paths"], meta=json.loads(str(z["meta"])))

    def path_frame(self) -> dict:
        """Return the path table as a column-keyed dict (convenient for ML)."""
        return {c: self.paths[:, i] for i, c in enumerate(PATH_COLUMNS)}
