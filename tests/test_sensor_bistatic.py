import numpy as np
import pytest
from wifi_radar_slam.radar.sensor_bistatic import (bistatic_path_len,
                                                   bistatic_detections_to_world)


def test_the_chain_reports_HALF_the_bistatic_path_length():
    # THE trap. RadarConfig.range_bins() assumes the MONOSTATIC convention (tau = 2R/c), so
    # fed a bistatic delay it reports half the true path length. Getting this wrong halves
    # every WiFi range and would look like a stunning physics result.
    assert bistatic_path_len(25.0) == pytest.approx(50.0)
    assert np.allclose(bistatic_path_len(np.array([10.0, 30.0])), [20.0, 60.0])


def test_the_ellipse_solve_recovers_a_reflector_from_a_path_length_and_a_bearing():
    # Vehicle at the origin facing +x; AP at (0, 10). A reflector at (20, 0) gives
    #   |AP->R| = sqrt(20^2 + 10^2) = 22.36 ;  |R->veh| = 20  ->  path length 42.36 m
    # and the vehicle sees it at bearing 0 deg. The solve must return (20, 0).
    pose = np.array([0.0, 0.0, 0.0])
    ap = np.array([0.0, 10.0])
    L = np.hypot(20.0, 10.0) + 20.0
    w = bistatic_detections_to_world(np.array([L / 2.0]), np.array([0.0]), pose, ap)
    assert w.shape == (1, 2)
    assert np.allclose(w[0], [20.0, 0.0], atol=1e-6)


def test_the_ellipse_solve_uses_the_WORLD_bearing_so_yaw_is_applied():
    # Same geometry, but the vehicle is rotated 90 deg: the reflector now lies along the
    # vehicle's LOCAL -90 deg. The world answer must be unchanged.
    pose = np.array([0.0, 0.0, np.pi / 2])
    ap = np.array([0.0, 10.0])
    L = np.hypot(20.0, 10.0) + 20.0
    w = bistatic_detections_to_world(np.array([L / 2.0]), np.array([-np.pi / 2]), pose, ap)
    assert np.allclose(w[0], [20.0, 0.0], atol=1e-6)


def test_the_line_of_sight_path_is_rejected_not_mapped():
    # A path length equal to |AP - vehicle| is the DIRECT path: there is no reflector on it.
    # Mapping it would plant a phantom on top of the AP in every single frame.
    pose = np.array([0.0, 0.0, 0.0])
    ap = np.array([30.0, 0.0])
    w = bistatic_detections_to_world(np.array([30.0 / 2.0]), np.array([0.0]), pose, ap)
    assert w.shape == (0, 2)


def test_no_detections_gives_an_empty_array():
    w = bistatic_detections_to_world(np.empty(0), np.empty(0),
                                     np.array([0.0, 0.0, 0.0]), np.array([0.0, 10.0]))
    assert w.shape == (0, 2)


def test_module_imports_without_sionna():
    import wifi_radar_slam.radar.sensor_bistatic as s
    assert hasattr(s, "SionnaBistaticSensor")
