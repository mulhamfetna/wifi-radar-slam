"""The ray tracer's TRUE paths -- the ground truth the phantom rate is measured against.

A phantom is "a detection matching no real propagation path", so this module defines what counts
as a real path. It is pure given already-solved Sionna arrays, which makes it unit-testable with
fakes and keeps Sionna out of the test path entirely.

The array layouts here were MEASURED on the server, not read off a wiki
(docs/results-paper3-radar-substrate.md):

    tau / phi_r / valid      : (n_rx, n_tx, n_paths)
    objects / interactions   : (depth, n_rx, n_tx, n_paths)

and n_tx is 4 -- the scene's three WiFi APs PLUS our radar_tx -- so indexing the wrong
transmitter silently mixes bistatic AP paths into a monostatic radar's ray set.
"""
from __future__ import annotations
import numpy as np

C = 299792458.0


def true_paths_for_tx(paths, tx_index: int, yaw: float, floor_ids,
                      monostatic: bool) -> dict:
    """True paths of one transmitter -> {range_m, azimuth_world_rad, n}.

    `range_m` is the quantity a detection of this geometry is comparable to:
      * monostatic -> the ROUND-TRIP range, tau*c/2
      * bistatic   -> the FULL path length, tau*c   (AP -> reflector -> vehicle)

    Halving the bistatic one would make every WiFi detection look like a phantom and rig RQ1 in
    radar's favour, so the distinction is explicit and tested.

    Ground-bounce paths are dropped. 61 % of a monostatic radar's rays hit the road, and the
    ground-truth map contains building footprints only -- a road return has nothing to be scored
    against. Paper 2's LiDAR dropped them the same way, which is what keeps the maps comparable
    across papers.

    `yaw` is accepted so callers can be explicit about frames, but the returned azimuth is the
    WORLD azimuth Sionna already reports: it is *not* subtracted.
    """
    tau = np.asarray(paths.tau.numpy())[0, tx_index]
    phi = np.asarray(paths.phi_r.numpy())[0, tx_index]
    valid = np.asarray(paths.valid.numpy())[0, tx_index].astype(bool)

    keep = valid & np.isfinite(tau) & np.isfinite(phi) & (tau > 0)
    if floor_ids:
        objs = np.asarray(paths.objects.numpy())[:, 0, tx_index]        # (depth, n_paths)
        inter = np.asarray(paths.interactions.numpy())[:, 0, tx_index]
        hits_floor = np.any((inter != 0) & np.isin(objs, list(floor_ids)), axis=0)
        keep &= ~hits_floor

    tau, phi = tau[keep], phi[keep]
    rng_m = tau * C / (2.0 if monostatic else 1.0)
    return {"range_m": rng_m, "azimuth_world_rad": phi, "n": int(rng_m.size)}
