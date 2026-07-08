import numpy as np
from wifi_radar_slam.slam.particle_filter import run_slam, _triangulate_bistatic, _cluster


def test_cluster_min_support_rejects_phantoms():
    # a dense cluster of 6 near (10, 3) + two isolated phantoms far away
    dense = np.array([[10.0, 3.0]] * 6) + np.random.default_rng(0).normal(0, 0.05, (6, 2))
    phantoms = np.array([[80.0, -40.0], [-30.0, 60.0]])
    pts = np.vstack([dense, phantoms])
    kept = _cluster(pts, radius=0.5, min_support=3)
    assert len(kept) == 1                                   # only the dense cluster survives
    assert np.linalg.norm(kept[0] - [10.0, 3.0]) < 0.2
    # default min_support=1 keeps everything (back-compatible)
    assert len(_cluster(pts, radius=0.5, min_support=1)) == 3


def test_triangulate_grazing_geometry():
    # Reflector nearly in line with the AP direction -> small denom but a
    # well-determined solve. The identity path/aoa are built from `refl`, so the
    # solver must recover it (the old |denom|<1 guard wrongly rejected this).
    pose = np.array([0.0, 0.0])
    ap = np.array([0.0, 40.0])
    refl = np.array([1.0, 20.0])
    d = refl - pose
    path = np.linalg.norm(ap - refl) + np.linalg.norm(d)
    aoa = np.arctan2(d[1], d[0])
    # confirm this really is the grazing (small-denom) regime
    u = np.array([np.cos(aoa), np.sin(aoa)])
    assert abs(2.0 * (path - (ap - pose) @ u)) < 1.0
    est = _triangulate_bistatic(pose, ap, path, aoa)
    assert est is not None
    assert np.allclose(est, refl, atol=1e-6)


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
