import numpy as np
from wifi_radar_slam.slam.particle_filter import run_slam


def test_recovers_straight_trajectory():
    n = 20
    dt = 0.05
    velocity = np.tile([5.0, 0.0], (n, 1))
    # one reflector at (10, 3); build BISTATIC detections (AP->refl->vehicle) + AoA
    aps = [np.array([0.0, 20.0, 6.0])]
    ap_xy = aps[0][:2]
    refl = np.array([10.0, 3.0])
    dets = []
    for f in range(n):
        pos = np.array([f * 5.0 * dt, 0.0])
        d = refl - pos
        path = np.linalg.norm(ap_xy - refl) + np.linalg.norm(d)   # bistatic path length
        aoa = np.arctan2(d[1], d[0])
        dets.append(np.array([[path, aoa, 0.0]]))
    rng_gen = np.random.default_rng(0)
    traj, mp = run_slam(dets, aps, velocity, dt, rng_gen, n_particles=300,
                        init_pose=np.array([0.0, 0.0, 0.0]))
    assert traj.shape == (n, 3)
    # end position near (0.05*5*19, 0)
    assert np.isclose(traj[-1, 0], 4.75, atol=0.5)
    assert np.isclose(traj[-1, 1], 0.0, atol=0.5)
    # a mapped point near the true reflector
    assert np.min(np.linalg.norm(mp - refl, axis=1)) < 1.0
