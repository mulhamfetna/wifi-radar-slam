"""Virtual-anchor FastSLAM (Rao-Blackwellized particle filter) for bistatic WiFi.

Why this replaces ``particle_filter.run_slam`` for localization claims
---------------------------------------------------------------------
The legacy filter triangulated each detection from the filter's *own* mean pose and then
scored the particles against that same reflector, so a measurement carried no outside
information about position. Its trajectory accuracy came entirely from odometry.

Here the landmark is the **virtual anchor** (VA): the access point mirrored in the
reflecting surface. For a single specular bounce the VA is fixed in the world, the bistatic
path length equals the range |VA - p|, and the arrival azimuth is the bearing to the VA.
That is the standard multipath-assisted / Channel-SLAM formulation. Each particle carries
its own EKF map of VAs (FastSLAM 1.0); weights come from the innovation of a detection
against a VA that was initialized at an *earlier* time, so the update genuinely constrains
position.

Landmark promotion is explicit: a VA is *confirmed* once associated ``promote_min_obs``
times; tentative VAs not re-observed within ``prune_after`` frames are deleted. The map is
the set of specular reflection points of detections associated to confirmed VAs, located
from the posterior pose (so mapping is downstream of corrected localization).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .particle_filter import _triangulate_bistatic, _cluster


@dataclass(frozen=True)
class FastSlamParams:
    n_particles: int = 200
    pos_noise: float = 0.02             # per-step process noise (m), covers odometry error
    sigma_range: float = 0.5            # path-length measurement std (m)
    sigma_aoa_deg: float = 3.0          # azimuth measurement std
    gate_chi2: float = 9.21             # 99 % gate, 2 dof
    new_landmark_lik: float = 1e-3      # likelihood assigned to an unexplained detection
    max_landmarks: int = 256
    promote_min_obs: int = 5            # observations needed to confirm a VA
    prune_after: int = 10               # frames a tentative VA may go unseen
    map_cluster_radius: float = 0.5


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def _horizontal_length(excess, pose_xy, ap, rx_height):
    """Commodity-CSI excess delay -> horizontal bistatic path length, per pose.

    CSI only fixes delays relative to the first (line-of-sight) arrival, so the full 3-D
    path length is ``excess + |AP - p|_3D``; dropping the AP-receiver height difference
    (preserved by a wall bounce) gives the 2-D unfolded length the filter works in.
    """
    dh = ap[2] - rx_height
    los3 = np.sqrt(np.sum((ap[:2] - pose_xy) ** 2, axis=-1) + dh ** 2)
    return np.sqrt(np.maximum((excess + los3) ** 2 - dh ** 2, 1e-4))


def run_fastslam(detections, ap_positions, velocity, timestep_s, rng,
                 init_pose=None, params: FastSlamParams = FastSlamParams(),
                 excess_delay: bool = False, rx_height: float = 1.5):
    """Returns ``(est_traj (n,3), est_map (k,2), info)``.

    Detections are ``[length_m, azimuth_rad, ap_index]``. With ``excess_delay=False`` the
    length is the *horizontal* bistatic path length (absolute delay, i.e. a synchronized
    transmitter). With ``excess_delay=True`` it is the 3-D excess over the LOS arrival --
    what commodity CSI actually provides -- and is converted per particle using the known
    AP position.

    ``info`` holds ``landmarks`` (confirmed VA positions of the best particle),
    ``n_confirmed`` and ``n_detections_used``.
    """
    P, M = params.n_particles, params.max_landmarks
    R = np.diag([params.sigma_range ** 2, np.deg2rad(params.sigma_aoa_deg) ** 2])
    n = len(detections)
    pose = np.zeros((P, 2))
    if init_pose is not None:
        pose[:] = np.asarray(init_pose, dtype=float)[:2]
    w = np.full(P, 1.0 / P)
    L = np.zeros((P, M, 2)); S = np.zeros((P, M, 2, 2))
    act = np.zeros((P, M), bool); cnt = np.zeros((P, M), int)
    last = np.zeros((P, M), int); lap = np.full((P, M), -1); lid = np.full((P, M), -1)
    next_id = 0
    est = np.zeros((n, 3))
    records = []                                   # (frame, landmark id, reflection point)
    used = 0
    ar = np.arange(P)
    aps = np.asarray(ap_positions, dtype=float)

    for f in range(n):
        if f > 0:
            pose += velocity[f] * timestep_s + rng.normal(0, params.pos_noise, (P, 2))
        dets = detections[f]
        assoc = np.full((P, dets.shape[0]), -1)
        logw = np.zeros(P)
        taken = np.zeros((P, M), bool)             # one detection per landmark per frame
        for j, (rho_in, phi, api) in enumerate(dets):
            api = int(api)
            rho = (_horizontal_length(rho_in, pose, aps[api], rx_height) if excess_delay
                   else np.full(P, rho_in))
            cand = act & (lap == api) & ~taken
            cols = np.where(cand.any(0))[0]
            hit = np.zeros(P, bool)
            k = np.zeros(P, int)
            if cols.size:
                Lc, Sc = L[:, cols], S[:, cols]
                d = Lc - pose[:, None, :]
                r2 = np.maximum(np.sum(d ** 2, -1), 1e-6); r = np.sqrt(r2)
                nu = np.stack([rho[:, None] - r, _wrap(phi - np.arctan2(d[..., 1], d[..., 0]))], -1)
                H = np.empty(d.shape[:2] + (2, 2))
                H[..., 0, 0] = d[..., 0] / r; H[..., 0, 1] = d[..., 1] / r
                H[..., 1, 0] = -d[..., 1] / r2; H[..., 1, 1] = d[..., 0] / r2
                Sm = H @ Sc @ np.swapaxes(H, -1, -2) + R
                det = Sm[..., 0, 0] * Sm[..., 1, 1] - Sm[..., 0, 1] * Sm[..., 1, 0]
                Si = np.empty_like(Sm)
                Si[..., 0, 0] = Sm[..., 1, 1] / det; Si[..., 1, 1] = Sm[..., 0, 0] / det
                Si[..., 0, 1] = -Sm[..., 0, 1] / det; Si[..., 1, 0] = -Sm[..., 1, 0] / det
                md = np.einsum('pmi,pmij,pmj->pm', nu, Si, nu)
                md = np.where(cand[:, cols], md, np.inf)
                kc = np.argmin(md, axis=1)
                hit = md[ar, kc] < params.gate_chi2
                k = cols[kc]
            # EKF update for associated particles
            if hit.any():
                p_, kc_ = ar[hit], kc[hit]
                k_ = k[hit]
                Hk, Sik, nuk = H[p_, kc_], Si[p_, kc_], nu[p_, kc_]
                Sigk = S[p_, k_]
                K = Sigk @ np.swapaxes(Hk, -1, -2) @ Sik
                L[p_, k_] += np.einsum('pij,pj->pi', K, nuk)
                S[p_, k_] = (np.eye(2) - K @ Hk) @ Sigk
                cnt[p_, k_] += 1; last[p_, k_] = f
                taken[p_, k_] = True
                logw[p_] += -0.5 * md[p_, kc_] - np.log(2 * np.pi * np.sqrt(det[p_, kc_]))
                assoc[p_, j] = k_
            # initialize a new tentative VA for the rest
            miss = ~hit
            if miss.any():
                p_ = ar[miss]
                logw[p_] += np.log(params.new_landmark_lik)
                free = ~act[p_]
                slot = np.where(free.any(1), np.argmax(free, 1),
                                np.argmin(np.where(act[p_], cnt[p_], 1 << 30), 1))
                u = np.array([np.cos(phi), np.sin(phi)])
                rp = rho[p_]
                G = np.zeros((len(p_), 2, 2))
                G[:, 0, 0] = u[0]; G[:, 0, 1] = -rp * u[1]
                G[:, 1, 0] = u[1]; G[:, 1, 1] = rp * u[0]
                L[p_, slot] = pose[p_] + rp[:, None] * u
                S[p_, slot] = G @ R @ np.swapaxes(G, -1, -2)
                act[p_, slot] = True; cnt[p_, slot] = 1; last[p_, slot] = f
                lap[p_, slot] = api
                ids = next_id + np.arange(len(p_)); next_id += len(p_)
                lid[p_, slot] = ids
                assoc[p_, j] = slot
                taken[p_, slot] = True
        # prune stale tentative landmarks
        stale = act & (cnt < params.promote_min_obs) & (f - last > params.prune_after)
        act &= ~stale
        if dets.shape[0]:
            used += dets.shape[0]
            logw -= logw.max()
            w *= np.exp(logw); w += 1e-300; w /= w.sum()
        mean = np.average(pose, axis=0, weights=w)
        est[f, :2] = mean
        est[f, 2] = float(init_pose[2]) if init_pose is not None and len(init_pose) > 2 else 0.0
        # record reflection points from the posterior mean pose, tagged by the best
        # particle's landmark id (resolved to confirmed/unconfirmed at the end)
        b = int(np.argmax(w))
        for j, (rho_in, phi, api) in enumerate(dets):
            if assoc[b, j] < 0:
                continue
            rho = (float(_horizontal_length(rho_in, mean, aps[int(api)], rx_height))
                   if excess_delay else rho_in)
            pt = _triangulate_bistatic(mean, aps[int(api)][:2], rho, phi)
            if pt is not None:
                records.append((int(lid[b, assoc[b, j]]), pt))
        if dets.shape[0] and 1.0 / np.sum(w ** 2) < P / 2:
            idx = rng.choice(P, P, p=w)
            pose, L, S, act, cnt, last, lap, lid = (x[idx] for x in
                                                    (pose, L, S, act, cnt, last, lap, lid))
            w = np.full(P, 1.0 / P)

    b = int(np.argmax(w))
    conf = act[b] & (cnt[b] >= params.promote_min_obs)
    conf_ids = set(lid[b, conf].tolist())
    pts = np.array([pt for i, pt in records if i in conf_ids])
    emap = (_cluster(pts, radius=params.map_cluster_radius) if len(pts)
            else np.empty((0, 2)))
    info = {"landmarks": L[b, conf].copy(), "n_confirmed": int(conf.sum()),
            "n_detections_used": used}
    return est, emap, info
