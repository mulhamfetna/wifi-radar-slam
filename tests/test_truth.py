import numpy as np
import pytest
from wifi_radar_slam.radar.truth import true_paths_for_tx

C = 299792458.0


class FakeTensor:
    def __init__(self, a):
        self._a = np.asarray(a)

    def numpy(self):
        return self._a


class FakePaths:
    """Mimics Sionna RT 2.0.1's real layouts, which were MEASURED on the server:
       tau / phi_r / valid    : (n_rx, n_tx, n_paths)
       objects / interactions : (depth, n_rx, n_tx, n_paths)
    """

    def __init__(self, tau, phi, valid, objects, inter):
        self.tau = FakeTensor(tau[None, ...])
        self.phi_r = FakeTensor(phi[None, ...])
        self.valid = FakeTensor(valid[None, ...])
        self.objects = FakeTensor(objects[:, None, ...])
        self.interactions = FakeTensor(inter[:, None, ...])


def make(tau, phi, valid, obj0, ninter):
    tau = np.array(tau, dtype=float).reshape(1, -1)          # (n_tx=1, n_paths)
    phi = np.array(phi, dtype=float).reshape(1, -1)
    valid = np.array(valid, dtype=bool).reshape(1, -1)
    n = tau.shape[1]
    objects = np.zeros((2, 1, n), dtype=int)
    inter = np.zeros((2, 1, n), dtype=int)
    if n:
        objects[0, 0] = np.array(obj0, dtype=int)
        for i, k in enumerate(ninter):
            inter[:k, 0, i] = 1
    return FakePaths(tau, phi, valid, objects, inter)


def test_monostatic_range_is_the_ROUND_TRIP_half_of_tau_c():
    p = make([2 * 30.0 / C], [0.0], [True], [5], [1])
    t = true_paths_for_tx(p, 0, yaw=0.0, floor_ids=set(), monostatic=True)
    assert t["range_m"][0] == pytest.approx(30.0)


def test_bistatic_range_is_the_FULL_path_length():
    # The bistatic delay IS the path length AP->reflector->vehicle. Halving it here would make
    # every WiFi detection look like a phantom, which would rig RQ1 in radar's favour.
    p = make([60.0 / C], [0.0], [True], [5], [1])
    t = true_paths_for_tx(p, 0, yaw=0.0, floor_ids=set(), monostatic=False)
    assert t["range_m"][0] == pytest.approx(60.0)


def test_azimuth_is_returned_in_the_WORLD_frame():
    # Sionna's phi_r is already a world azimuth, so `yaw` must NOT be subtracted from it.
    p = make([2 * 30.0 / C], [np.deg2rad(40.0)], [True], [5], [1])
    t = true_paths_for_tx(p, 0, yaw=np.deg2rad(10.0), floor_ids=set(), monostatic=True)
    assert np.rad2deg(t["azimuth_world_rad"][0]) == pytest.approx(40.0)


def test_invalid_paths_are_dropped():
    p = make([2 * 30.0 / C, 2 * 40.0 / C], [0.0, 0.0], [True, False], [5, 5], [1, 1])
    t = true_paths_for_tx(p, 0, yaw=0.0, floor_ids=set(), monostatic=True)
    assert t["n"] == 1


def test_ground_bounce_paths_are_dropped():
    # 61% of a monostatic radar's rays hit the road. The ground-truth map holds building
    # footprints only, so a road return has nothing to be scored against -- and paper 2's LiDAR
    # dropped them the same way, which is what keeps the maps comparable across papers.
    p = make([2 * 30.0 / C, 2 * 40.0 / C], [0.0, 0.0], [True, True], [7, 5], [1, 1])
    t = true_paths_for_tx(p, 0, yaw=0.0, floor_ids={7}, monostatic=True)
    assert t["n"] == 1
    assert t["range_m"][0] == pytest.approx(40.0)


def test_empty_path_set():
    p = make([], [], [], [], [])
    t = true_paths_for_tx(p, 0, yaw=0.0, floor_ids=set(), monostatic=True)
    assert t["n"] == 0
    assert t["range_m"].shape == (0,)
