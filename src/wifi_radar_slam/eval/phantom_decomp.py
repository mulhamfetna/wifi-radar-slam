"""Phantom decomposition with the manuscript's matching rule (3 m / 10 deg / cost 3,
per frame, same AP), split into: matched, ULA front/back *folded*, and unexplained.

Detections must carry ABSOLUTE 3-D path lengths (column 0) so they are compared like with
like against the ray tracer's ``delay_s * c``.
"""
from __future__ import annotations

import numpy as np

from ..dataset import PATH_COLUMNS
from .scenes import SceneCache, fold_forward

C = 299792458.0
COL = {c: i for i, c in enumerate(PATH_COLUMNS)}
KEYS = ("matched_facade", "matched_other", "folded_facade", "folded_other", "unexplained")


def _cost(rng_t, phi_t, r, a):
    return np.hypot((rng_t - r) / 3.0, np.angle(np.exp(1j * (phi_t - a))) / np.deg2rad(10))


def decompose(sc: SceneCache, dets) -> dict:
    n = dict.fromkeys(KEYS, 0)
    err = []
    for f, D in enumerate(dets):
        P = sc.frame_paths(f)
        for r, a, ap in np.asarray(D).reshape(-1, 3):
            T = P[P[:, COL["ap"]] == ap]
            if not len(T):
                n["unexplained"] += 1; continue
            rt, pt = T[:, COL["delay_s"]] * C, T[:, COL["phi_r"]]
            facade = (T[:, COL["n_bounce"]] == 1) & (T[:, COL["is_floor"]] == 0)
            c0 = _cost(rt, pt, r, a); j0 = int(np.argmin(c0))
            if c0[j0] <= 3.0:
                n["matched_facade" if facade[j0] else "matched_other"] += 1
                if facade[j0]:
                    err.append(r - rt[j0])
                continue
            c1 = _cost(rt, fold_forward(pt), r, a); j1 = int(np.argmin(c1))
            if c1[j1] <= 3.0:
                n["folded_facade" if facade[j1] else "folded_other"] += 1
            else:
                n["unexplained"] += 1
    tot = max(sum(n.values()), 1)
    out = {k: v / tot for k, v in n.items()}
    out["n_detections"] = sum(n.values())
    out["phantom_rate_manuscript_def"] = (n["folded_facade"] + n["folded_other"]
                                          + n["unexplained"]) / tot
    out["median_signed_range_err_facade_m"] = float(np.median(err)) if err else float("nan")
    out["median_abs_range_err_facade_m"] = float(np.median(np.abs(err))) if err else float("nan")
    return out
