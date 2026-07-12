"""Cell A -- ambient BISTATIC WiFi, on the SAME detection chain as the radar cells.

WHY THE SAME CHAIN IS LEGITIMATE. Cell A is passive: there is no FMCW chirp. But a beat signal
and an OFDM CSI vector are the SAME measurement. An FMCW sweep sampled at N instants across
bandwidth B measures the channel at N frequencies spanning B; an OFDM CSI vector across N
subcarriers spanning B measures exactly the same thing. A Fourier transform of either yields the
delay profile. So `beat_matrix` serves cell A verbatim -- given BISTATIC delays -- and the chain
is held genuinely fixed across every cell, not merely "analogous".

WHAT DIFFERS IS THE GEOMETRY, AND THAT IS THE POINT (the A->B ablation):

  monostatic (B, C, D):  delay is a round trip, tau = 2R/c
                         detection -> world is a plain polar projection
  bistatic   (A):        delay is a PATH LENGTH, tau = (|AP->R| + |R->veh|)/c
                         detection -> world needs an ELLIPSE SOLVE, with the AP and the vehicle
                         at the foci, whose conditioning degrades with the ellipse's eccentricity

That asymmetry is not an implementation detail -- it is the mechanism the ablation isolates, and
it is why paper 2's WiFi maps carried a 6.45 m range bias against a 0.94 m resolution limit.

`SionnaBistaticSensor` lazily imports Sionna inside its methods, so this module imports fine
without it.
"""
from __future__ import annotations
import numpy as np

from ..geometry import RX_HEIGHT_M
from ..slam.particle_filter import _triangulate_bistatic
from .processing import beat_matrix, range_fft, azimuth_beamform, cfar_2d, cluster_detections

C = 299792458.0


def bistatic_path_len(reported_range_m):
    """Undo the chain's MONOSTATIC range convention.

    `RadarConfig.range_bins()` maps a beat frequency to range assuming tau = 2R/c. Fed a
    bistatic delay tau = L/c, it therefore reports L/2. Multiply by two to recover the true path
    length.

    This is a one-line function on purpose: inlining it is exactly how one silently halves every
    WiFi range in the paper, and a halved range is a plausible-looking wrong answer -- the worst
    kind.
    """
    return 2.0 * np.asarray(reported_range_m, dtype=float)


def bistatic_detections_to_world(reported_ranges, azimuths_local, pose, ap_xy) -> np.ndarray:
    """The ELLIPSE SOLVE: (bistatic path length, bearing) -> reflector, in world coordinates.

    Args:
        reported_ranges: what the chain reported (HALF the path length -- see above).
        azimuths_local:  sensor-local bearings (rad).
        pose:            vehicle (x, y, yaw).
        ap_xy:           the illuminating AP's (x, y).

    Returns (M, 2) world points. Detections with no valid solution -- notably the DIRECT
    line-of-sight path, which has no reflector on it at all -- are dropped rather than mapped.

    Reuses `_triangulate_bistatic`, the same solve papers 1-2 used, so cell A is faithful to the
    prior work rather than a fresh re-derivation that might quietly differ from it.
    """
    r = np.asarray(reported_ranges, dtype=float).ravel()
    a = np.asarray(azimuths_local, dtype=float).ravel()
    if r.size == 0:
        return np.empty((0, 2))
    pose = np.asarray(pose, dtype=float)
    veh_xy = pose[:2]
    yaw = float(pose[2]) if pose.size > 2 else 0.0

    lens = bistatic_path_len(r)
    out = []
    for L, az in zip(lens, a):
        world_az = float(np.arctan2(np.sin(az + yaw), np.cos(az + yaw)))
        R = _triangulate_bistatic(veh_xy, ap_xy, float(L), world_az)
        if R is not None:
            out.append(R)
    return np.array(out) if out else np.empty((0, 2))


class SionnaBistaticSensor:
    """Cell A: the scene's ambient WiFi APs illuminate; the vehicle receives.

    Emits, per frame, the detections the SHARED chain produces -- one pass per AP, pooled.

    ALL the APs are used. A real ambient deployment has several free illuminators, and that is a
    genuine part of WiFi's geometry. Crippling cell A to a single AP to "match" the monostatic
    cells would be tuning WiFi DOWN to make radar look better -- which the spec forbids exactly
    as firmly as it forbids tuning WiFi up.
    """

    def __init__(self, built, cfg, rng, max_depth: int = 3, scattering: float = 0.7):
        import sionna.rt as rt                       # lazy: server only
        from .sensor import retune_scene
        self.built, self.cfg, self.rng = built, cfg, rng
        self.max_depth = max_depth
        self.scene = built.scene
        self.frozen_materials = retune_scene(self.scene, cfg.carrier_hz)
        for m in self.scene.radio_materials.values():
            try:
                m.scattering_coefficient = scattering
            except Exception:
                pass
        self.solver = rt.PathSolver()
        self.rx = self.scene.receivers["veh"]
        # the APs are the scene's own transmitters -- everything that is NOT our radar TX
        names = list(self.scene.transmitters.keys())
        self.ap_idx = [i for i, n in enumerate(names) if n != "radar_tx"]
        self.ap_xy = [np.asarray(p, dtype=float)[:2] for p in built.ap_positions]
        self.floor_ids = {o.object_id for n, o in self.scene.objects.items()
                          if "floor" in n.lower()}

    def _solve(self, pose):
        import os
        import mitsuba as mi
        px, py = float(pose[0]), float(pose[1])
        self.rx.position = mi.Point3f(px, py, RX_HEIGHT_M)
        ns = int(os.environ.get("WRS_NUM_SAMPLES", "1000000"))
        return self.solver(self.scene, max_depth=self.max_depth, samples_per_src=ns,
                           diffuse_reflection=True,
                           seed=int(self.rng.integers(1, 2 ** 31 - 1)))

    def _touches_floor(self, paths) -> np.ndarray:
        """(n_tx, n_paths) mask of paths that bounce off the ground at any depth."""
        objs = np.asarray(paths.objects.numpy())[:, 0]          # (depth, n_tx, n_paths)
        inter = np.asarray(paths.interactions.numpy())[:, 0]
        if not self.floor_ids:
            return np.zeros(objs.shape[1:], dtype=bool)
        return np.any((inter != 0) & np.isin(objs, list(self.floor_ids)), axis=0)

    def __call__(self, pose):
        """Solve and detect. See `detect` for the return contract."""
        return self.detect(self._solve(pose), pose)

    def detect(self, paths, pose):
        """-> (world_points (M,2), det_pathlen (K,), det_azimuth_world (K,), ap_index (K,))

        The world points are the MAP contribution; the detection triples are what the phantom
        rate is measured on.

        Takes an ALREADY-SOLVED `paths` so the caller can measure the phantom rate against the
        very same ray set that produced the detections -- and so the ray tracer, which dominates
        the runtime, runs once per frame rather than twice.
        """
        tau_all = np.asarray(paths.tau.numpy())[0]              # (n_tx, n_paths)
        phi_all = np.asarray(paths.phi_r.numpy())[0]
        val_all = np.asarray(paths.valid.numpy())[0].astype(bool)
        re, im = paths.a
        a_all = (np.asarray(re.numpy())[0, 0, :, 0]
                 + 1j * np.asarray(im.numpy())[0, 0, :, 0])     # (n_tx, n_paths)
        floor = self._touches_floor(paths)                      # (n_tx, n_paths)

        yaw = float(pose[2]) if len(pose) > 2 else 0.0
        world, d_len, d_az, d_ap = [], [], [], []
        for k, t in enumerate(self.ap_idx):
            keep = (val_all[t] & ~floor[t] & np.isfinite(tau_all[t])
                    & (tau_all[t] > 0) & (np.abs(a_all[t]) > 0))
            tau, amp = tau_all[t][keep], a_all[t][keep]
            if tau.size == 0:
                continue
            # Sionna's phi_r is a WORLD azimuth; the chain wants sensor-local
            az_local = np.angle(np.exp(1j * (phi_all[t][keep] - yaw)))
            beat = beat_matrix(tau, amp, az_local, self.cfg, rng=self.rng)
            ra = azimuth_beamform(range_fft(beat, self.cfg), self.cfg)
            rng_m, az_m = cluster_detections(cfar_2d(ra, self.cfg), ra, self.cfg)
            if rng_m.size == 0:
                continue
            ap_xy = self.ap_xy[k] if k < len(self.ap_xy) else self.ap_xy[0]
            w = bistatic_detections_to_world(rng_m, az_m, pose, ap_xy)
            if w.size:
                world.append(w)
            d_len.append(bistatic_path_len(rng_m))
            d_az.append(np.angle(np.exp(1j * (az_m + yaw))))     # world azimuth
            d_ap.append(np.full(rng_m.size, k, dtype=int))

        W = np.concatenate(world) if world else np.empty((0, 2))
        L = np.concatenate(d_len) if d_len else np.empty(0)
        A = np.concatenate(d_az) if d_az else np.empty(0)
        P = np.concatenate(d_ap) if d_ap else np.empty(0, dtype=int)
        return W, L, A, P
