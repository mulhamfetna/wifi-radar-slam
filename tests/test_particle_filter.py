import numpy as np
from wifi_radar_slam.slam.particle_filter import run_slam


def test_recovers_straight_trajectory():
    n = 20
    dt = 0.05
    velocity = np.tile([5.0, 0.0], (n, 1))
    # one reflector at (10, 3); build detections: range+AoA from the true path
    aps = [np.array([0.0, 20.0, 6.0])]
    refl = np.array([10.0, 3.0])
    dets = []
    for f in range(n):
        px, py = f * 5.0 * dt, 0.0
        d = refl - np.array([px, py])
        rng = np.linalg.norm(d)
        aoa = np.arctan2(d[1], d[0])
        dets.append(np.array([[rng, aoa, 0.0]]))
    rng_gen = np.random.default_rng(0)
    traj, mp = run_slam(dets, aps, velocity, dt, rng_gen, n_particles=300)
    assert traj.shape == (n, 3)
    # end position near (0.05*5*19, 0)
    assert np.isclose(traj[-1, 0], 4.75, atol=0.5)
    assert np.isclose(traj[-1, 1], 0.0, atol=0.5)
    # a mapped point near the true reflector
    assert np.min(np.linalg.norm(mp - refl, axis=1)) < 1.0
