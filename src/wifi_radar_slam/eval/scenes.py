"""Sionna-free access to scenes cached by ``experiments/cache_channels.py``.

Detections follow the back-end contract ``[path_length_m, azimuth_rad, ap_index]``.
``horizontal`` projects 3-D path lengths onto the ground plane using the known AP and
receiver heights (a wall bounce preserves the AP height in its virtual anchor), which is
what the 2-D filters and the 2-D bistatic solve assume.
"""
from __future__ import annotations

import json
from dataclasses import dataclass

import numpy as np

from ..config import RFConfig
from ..dataset import PATH_COLUMNS
from ..geometry import RX_HEIGHT_M

C = 299792458.0
_COL = {c: i for i, c in enumerate(PATH_COLUMNS)}


@dataclass
class SceneCache:
    poses: np.ndarray
    ap_positions: np.ndarray
    gt_map: np.ndarray
    paths: np.ndarray
    cfr: dict
    meta: dict

    @classmethod
    def load(cls, path: str) -> "SceneCache":
        z = np.load(path, allow_pickle=False)
        cfr = {int(k[4:]): z[k] for k in z.files if k.startswith("cfr_")}
        return cls(z["poses"], z["ap_positions"], z["gt_map"], z["paths"], cfr,
                   json.loads(str(z["meta"])))

    @property
    def n_frames(self) -> int:
        return self.poses.shape[0]

    @property
    def dt(self) -> float:
        return float(self.meta["timestep_s"])

    def rf(self, bw_mhz: int) -> RFConfig:
        m = self.meta
        return RFConfig(carrier_hz=float(m["carrier_hz"]), bandwidth_hz=bw_mhz * 1e6,
                        n_subcarriers=int(m["n_subcarriers"]),
                        n_rx_antennas=int(m["n_rx_antennas"]),
                        antenna_spacing_frac=float(m["antenna_spacing_frac"]))

    def frame_paths(self, f: int) -> np.ndarray:
        return self.paths[self.paths[:, _COL["frame"]] == f]


def noisy_csi(cfr: np.ndarray, snr_db: float, rng: np.random.Generator) -> np.ndarray:
    """AWGN at ``snr_db`` relative to the mean channel power (as ``simulate_csi``)."""
    sig_p = np.mean(np.abs(cfr) ** 2) + 1e-30
    noise_p = sig_p / (10 ** (snr_db / 10))
    n = (rng.normal(size=cfr.shape) + 1j * rng.normal(size=cfr.shape)) * np.sqrt(noise_p / 2)
    return cfr + n


def oracle_detections(sc: SceneCache) -> list[np.ndarray]:
    """True single-scatter, non-floor paths (the oracle front-end)."""
    out = []
    for f in range(sc.n_frames):
        P = sc.frame_paths(f)
        m = (P[:, _COL["n_bounce"]] == 1) & (P[:, _COL["is_floor"]] == 0)
        out.append(np.column_stack([P[m, _COL["delay_s"]] * C, P[m, _COL["phi_r"]],
                                    P[m, _COL["ap"]]]) if m.any() else np.empty((0, 3)))
    return out


def music_detections(sc: SceneCache, bw_mhz: int, snr_db: float, rng,
                     n_paths: int = 3) -> list[np.ndarray]:
    from ..sensing.frontend import extract_detections
    csi = noisy_csi(sc.cfr[bw_mhz], snr_db, rng)
    return extract_detections(csi, sc.rf(bw_mhz), n_paths=n_paths, world_aoa=True, joint=True)


def fold_forward(phi):
    """Map a world azimuth onto the single-ULA forward branch (what the array reports)."""
    return np.arcsin(np.clip(np.sin(np.asarray(phi, float)), -1, 1))


def horizontal(dets: list[np.ndarray], ap_positions: np.ndarray,
               rx_height: float = RX_HEIGHT_M) -> list[np.ndarray]:
    aps = np.asarray(ap_positions, float)
    out = []
    for d in dets:
        if not len(d):
            out.append(d); continue
        dh = aps[d[:, 2].astype(int), 2] - rx_height
        e = d.copy()
        e[:, 0] = np.sqrt(np.maximum(d[:, 0] ** 2 - dh ** 2, 1e-4))
        out.append(e)
    return out


def excess_oracle_detections(sc: SceneCache) -> list[np.ndarray]:
    """Oracle paths in the commodity-CSI convention: length EXCESS over the first arrival
    of that AP (exactly what Sionna's ``normalize_delays`` removes from the CSI)."""
    out = []
    for f in range(sc.n_frames):
        P = sc.frame_paths(f)
        rows = []
        for ap in np.unique(P[:, _COL["ap"]]):
            T = P[P[:, _COL["ap"]] == ap]
            first = T[:, _COL["delay_s"]].min()
            m = (T[:, _COL["n_bounce"]] == 1) & (T[:, _COL["is_floor"]] == 0)
            for t in T[m]:
                rows.append([(t[_COL["delay_s"]] - first) * C, t[_COL["phi_r"]], ap])
        out.append(np.array(rows) if rows else np.empty((0, 3)))
    return out


def excess_to_absolute(dets: list[np.ndarray], poses: np.ndarray, ap_positions: np.ndarray,
                       rx_height: float = RX_HEIGHT_M) -> list[np.ndarray]:
    """Excess length -> absolute 3-D bistatic length at the given poses (LOS assumed first)."""
    aps = np.asarray(ap_positions, float)
    out = []
    for d, p in zip(dets, poses):
        if not len(d):
            out.append(d); continue
        a = aps[d[:, 2].astype(int)]
        los = np.sqrt(np.sum((a[:, :2] - p[:2]) ** 2, 1) + (a[:, 2] - rx_height) ** 2)
        e = d.copy(); e[:, 0] = d[:, 0] + los
        out.append(e)
    return out
