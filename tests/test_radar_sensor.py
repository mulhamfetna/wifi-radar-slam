import numpy as np
import pytest
from wifi_radar_slam.radar.sensor import paths_to_rays

C = 299792458.0


def test_paths_to_rays_passes_delays_and_amplitudes_through_untouched():
    # tau must stay ABSOLUTE. Sionna's cfr()/cir()/taps() would zero the first-path delay
    # (normalize_delays defaults to True) and destroy range; we read paths.tau instead, so
    # the delays arriving here are already absolute and must not be rescaled.
    tau = np.array([1e-7, 2e-7])
    a = np.array([1 + 0j, 0.5 + 0.5j])
    taus, amps, az = paths_to_rays(tau, a, np.array([0.0, 0.0]), yaw=0.0)
    assert np.allclose(taus, tau)
    assert np.allclose(amps, a)


def test_paths_to_rays_rotates_world_azimuth_into_the_sensor_frame():
    # Sionna reports arrival azimuth in the WORLD frame; a Scan is sensor-local (+x
    # forward). The vehicle's yaw must therefore be subtracted.
    _, _, az = paths_to_rays(np.array([1e-7]), np.array([1 + 0j]),
                             np.array([np.deg2rad(90.0)]), yaw=np.deg2rad(30.0))
    assert np.rad2deg(az[0]) == pytest.approx(60.0)


def test_paths_to_rays_wraps_azimuth_to_minus_pi_pi():
    _, _, az = paths_to_rays(np.array([1e-7]), np.array([1 + 0j]),
                             np.array([np.deg2rad(170.0)]), yaw=np.deg2rad(-170.0))
    # 170 - (-170) = 340 deg, which must wrap to -20 deg, not stay at 340
    assert np.rad2deg(az[0]) == pytest.approx(-20.0, abs=1e-6)
    assert -np.pi <= az[0] <= np.pi


def test_paths_to_rays_drops_invalid_and_zero_amplitude_paths():
    tau = np.array([1e-7, np.nan, 3e-7])
    a = np.array([1 + 0j, 1 + 0j, 0 + 0j])          # third has zero amplitude
    phi = np.array([0.0, 0.0, 0.0])
    taus, amps, az = paths_to_rays(tau, a, phi, yaw=0.0)
    assert len(taus) == 1                            # nan dropped, zero-amp dropped
    assert taus[0] == pytest.approx(1e-7)


def test_paths_to_rays_drops_the_self_return_at_zero_delay():
    # TX and RX are CO-LOCATED in a monostatic radar, so Sionna emits a direct TX->RX path
    # at tau ~ 0. It is not a reflector -- it is the radar hearing itself -- and if it
    # survived it would put a bogus point on top of the vehicle in every single frame.
    tau = np.array([0.0, 2 * 20.0 / C])
    a = np.array([10 + 0j, 1 + 0j])
    taus, _, _ = paths_to_rays(tau, a, np.array([0.0, 0.0]), yaw=0.0)
    assert len(taus) == 1
    assert taus[0] == pytest.approx(2 * 20.0 / C)


def test_paths_to_rays_on_an_empty_path_set():
    taus, amps, az = paths_to_rays(np.empty(0), np.empty(0, dtype=complex),
                                   np.empty(0), yaw=0.0)
    assert len(taus) == 0 and len(amps) == 0 and len(az) == 0


def test_sensor_module_imports_without_sionna():
    # The point of the lazy-import pattern: this module must import on a machine that has
    # never seen Sionna (this laptop). Only *running* the sensor needs it.
    import wifi_radar_slam.radar.sensor as s
    assert hasattr(s, "SionnaRadarSensor") and hasattr(s, "radar_sensor")
