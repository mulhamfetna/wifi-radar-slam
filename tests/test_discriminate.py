import numpy as np
from wifi_radar_slam.discriminate import path_features, C, FEATURE_NAMES


def test_path_features_labels_and_excess():
    # 3 paths, ap0 at (0,20), vehicle (frame0) at (0,0)
    poses = np.array([[0.0, 0.0, 0.0]])
    aps = np.array([[0.0, 20.0, 6.0]])
    # columns: frame, ap, delay_s, phi_r, theta_r, n_bounce, first_type, obj, is_floor
    delay_direct = 20.0 / C           # 20 m direct path length
    paths = np.array([
        [0, 0, delay_direct,        np.pi/2, np.pi/2, 1, 1, 5, 0],   # single, non-floor -> useful
        [0, 0, 40.0 / C,            0.3,     np.pi/2, 2, 1, 5, 0],   # multi-bounce -> not useful
        [0, 0, 30.0 / C,            0.1,     np.pi/2, 1, 1, 7, 1],   # single but floor -> not useful
    ])
    X, y, names = path_features(paths, poses, aps)
    assert names == FEATURE_NAMES and X.shape == (3, 5)
    assert y.tolist() == [1, 0, 0]
    # range column = delay*c
    assert np.isclose(X[0, 0], 20.0) and np.isclose(X[1, 0], 40.0)
    # excess = range - dist_ap (dist_ap = 20 m); path0 excess ~ 0
    assert np.isclose(X[0, 1], 0.0, atol=1e-6)
    assert np.isclose(X[1, 1], 20.0, atol=1e-6)
