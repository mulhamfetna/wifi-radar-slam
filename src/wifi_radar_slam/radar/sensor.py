"""Sionna-ray-traced 77 GHz FMCW radar sensor (monostatic + diffuse scattering).

`paths_to_rays` is pure NumPy and tests locally. `SionnaRadarSensor` lazily imports
sionna.rt/mitsuba *inside* its methods, so this module imports fine without Sionna and only
*running* the sensor needs the amd server -- the same pattern as lidar/sensor_sionna.py.
"""
from __future__ import annotations
import numpy as np
from ..geometry import RX_HEIGHT_M
from ..lidar.pointcloud import Scan
from .processing import radar_scan

C = 299792458.0

# Paths shorter than this round-trip delay are the co-located TX/RX self-return, not a
# reflector. 2 * 0.1 m / c.
_MIN_TAU_S = 2.0 * 0.1 / C


def paths_to_rays(tau, a, phi_r, yaw: float):
    """Sionna path arrays -> (taus, amps, sensor-local azimuths) for the radar chain.

    Args:
        tau:   absolute propagation delays (s), one per path. **Absolute**: these come from
               `paths.tau`, NOT from cfr()/cir()/taps(), whose `normalize_delays` defaults
               to True and would zero the first-path delay -- destroying the very quantity
               a radar measures. Reading paths.tau makes that pitfall structurally
               impossible rather than merely avoided.
        a:     complex path amplitudes.
        phi_r: azimuth of arrival at the receiver (rad), in the WORLD frame.
        yaw:   vehicle heading (rad).

    Returns:
        (taus, amps, azimuths) with azimuths in the SENSOR-LOCAL frame (+x forward), wrapped
        to [-pi, pi]. Non-finite paths, zero-amplitude paths, and the co-located TX/RX
        self-return (tau ~ 0) are dropped.

    NOTE -- UNVERIFIED UNTIL THE SERVER RUNS experiments/validate_radar_sensor.py:
    Sionna is known to change its angle convention when TX and RX are co-located, as they
    are in a monostatic radar (NVlabs/sionna-rt#5). The `phi_r - yaw` mapping below is the
    documented convention and is our working HYPOTHESIS; that script verifies it empirically
    against a wall at a known bearing. If the bearing comes back mirrored, fix it HERE --
    never compensate downstream, or the ablation's geometry axis becomes uninterpretable.
    """
    tau = np.asarray(tau, dtype=float).ravel()
    a = np.asarray(a, dtype=complex).ravel()
    phi_r = np.asarray(phi_r, dtype=float).ravel()
    ok = (np.isfinite(tau) & np.isfinite(phi_r) & np.isfinite(a)
          & (np.abs(a) > 0) & (tau > _MIN_TAU_S))
    tau, a, phi_r = tau[ok], a[ok], phi_r[ok]
    az = np.arctan2(np.sin(phi_r - yaw), np.cos(phi_r - yaw))    # world -> local, wrapped
    return tau, a, az


def retune_scene(scene, frequency_hz: float) -> list[str]:
    """Retune a Sionna scene to `frequency_hz`, refusing to extrapolate materials out of band.

    Returns the names of the unused materials that were frozen.

    Sionna's ITU material models are only defined over published frequency bands (ITU-R
    P.2040): concrete and metal to 100 GHz, but marble only to 60 GHz and brick only to
    40 GHz. Setting `scene.frequency` eagerly re-evaluates **every material registered in
    the scene**, used or not -- so moving the street-canyon scene to 77 GHz for cell D
    raises on `marble`, a material no object in it actually uses.

    The safe resolution is not to relax the check. It is to notice that a material no ray
    ever hits has no physics to get wrong: we freeze those (drop the update callback, so
    their parameters simply stay put) and let the frequency be set. But a material that IS
    used and IS out of band must fail LOUDLY -- silently extrapolating ITU parameters past
    their validity band would quietly fabricate the permittivity of every surface the radar
    sees, which is precisely the kind of invisible error that ruins a paper. So we do not
    touch used materials, and `scene.frequency` raises on them exactly as it should.
    """
    used = {o.radio_material.name for o in scene.objects.values()}
    frozen = []
    for name, mat in scene.radio_materials.items():
        if name in used:
            continue                       # never freeze a material a ray can hit
        if mat.frequency_update_callback is not None:
            mat.frequency_update_callback = None
            frozen.append(name)
    scene.frequency = frequency_hz         # raises if a USED material is out of band
    return frozen


class SionnaRadarSensor:
    """Monostatic 77 GHz FMCW radar: a TX co-located with the vehicle RX, diffuse material
    backscatter, and the FULL detection chain (beat -> range FFT -> beamform -> CFAR).

    The chain is not an implementation choice, it is the experiment. Reading interaction
    vertices straight out of the ray tracer -- which is what LiDAR model B does -- would give
    radar zero ghosts by construction and rig RQ1, the paper's headline question. Every ghost
    and false alarm this sensor produces must earn its way through finite bandwidth, a finite
    aperture and a calibrated CFAR threshold.
    """

    def __init__(self, built, cfg, rng, scattering: float = 0.7, max_depth: int = 3):
        import sionna.rt as rt                  # lazy: server only
        self.built, self.cfg, self.rng = built, cfg, rng
        self.max_depth = max_depth
        self.scene = built.scene
        # The ablation's carrier axis. Retuning to 77 GHz is not a one-liner -- see
        # retune_scene: the scene ships with ITU materials (marble, brick) whose published
        # validity bands stop below 77 GHz, and Sionna re-evaluates all of them.
        self.frozen_materials = retune_scene(self.scene, cfg.carrier_hz)
        if "radar_tx" not in self.scene.transmitters:
            self.scene.add(rt.Transmitter("radar_tx",
                                          position=[0.0, 0.0, RX_HEIGHT_M]))
        # Diffuse backscatter is REQUIRED, not an enhancement. With specular-only materials
        # a monostatic radar is very nearly blind -- a mirror-like wall reflects away from
        # the sensor, not back to it. On this exact scene paper 2 measured 1 return with
        # specular only, versus 8,417 with diffuse enabled.
        for m in self.scene.radio_materials.values():
            try:
                m.scattering_coefficient = scattering
            except Exception:
                pass
        self.solver = rt.PathSolver()
        self.tidx = list(self.scene.transmitters.keys()).index("radar_tx")
        self.rx = self.scene.receivers["veh"]

    def _solve(self, pose):
        """Run the ray tracer at `pose` and return the raw Sionna Paths object."""
        import os
        import mitsuba as mi
        px, py = float(pose[0]), float(pose[1])
        # co-locating TX and RX is what makes the sensor monostatic
        self.scene.transmitters["radar_tx"].position = mi.Point3f(px, py, RX_HEIGHT_M)
        self.rx.position = mi.Point3f(px, py, RX_HEIGHT_M)
        ns = int(os.environ.get("WRS_NUM_SAMPLES", "1000000"))
        return self.solver(self.scene, max_depth=self.max_depth, samples_per_src=ns,
                           diffuse_reflection=True,
                           seed=int(self.rng.integers(1, 2 ** 31 - 1)))

    def _extract(self, paths):
        """Pull (tau, complex amplitude, world azimuth) out of a Sionna Paths object.

        The exact array layouts are confirmed empirically by
        experiments/validate_radar_sensor.py -- they cannot be checked without Sionna.
        """
        tau = np.asarray(paths.tau.numpy())
        a = np.asarray(paths.a.numpy())
        phi = np.asarray(paths.phi_r.numpy())
        # paths.a is returned as a leading (real, imag) pair; everything else is squeezed
        # down to a per-path vector for this single TX / single RX pair.
        if a.ndim > 1 and a.shape[0] == 2:
            a = a[0] + 1j * a[1]
        return tau.ravel(), np.asarray(a).ravel(), phi.ravel()

    def __call__(self, pose) -> Scan:
        yaw = float(pose[2]) if len(pose) > 2 else 0.0
        tau, a, phi = self._extract(self._solve(pose))
        taus, amps, az = paths_to_rays(tau, a, phi, yaw)
        return radar_scan(taus, amps, az, self.cfg, rng=self.rng)


def radar_sensor(built, cfg, rng) -> "SionnaRadarSensor":
    """make_sensor factory: matches the seam lidar/runner.run_lidar already expects, so the
    radar drops into the SAME scan-to-map ICP back-end with no change to it."""
    return SionnaRadarSensor(built, cfg, rng)
