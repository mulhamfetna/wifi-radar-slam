import numpy as np
from wifi_radar_slam.sensing.oracle import single_scatter_mask


def _stack(paths_types):
    """paths_types: list of per-path [d0, d1, d2] codes -> (depth, n_tx=1, n_paths)."""
    arr = np.array(paths_types).T          # (depth, n_paths)
    return arr[:, None, :]                  # (depth, 1, n_paths)


def test_single_scatter_mask_selects_one_interaction_any_type():
    inter = _stack([
        [1, 0, 0],   # p0: single specular              -> keep
        [1, 1, 0],   # p1: two speculars                -> drop
        [4, 0, 0],   # p2: single refraction            -> keep (one turn point)
        [0, 0, 0],   # p3: LOS (zero bounce)            -> drop
        [1, 4, 0],   # p4: specular + refraction        -> drop
        [2, 0, 0],   # p5: single diffuse               -> keep
        [4, 1, 1],   # p6: three interactions           -> drop
    ])
    valid = np.ones((1, 7), dtype=bool)
    mask = single_scatter_mask(inter, valid)
    assert mask.shape == (1, 7)
    assert mask[0].tolist() == [True, False, True, False, False, True, False]


def test_single_scatter_mask_respects_valid():
    inter = _stack([[4, 0, 0], [4, 0, 0]])
    valid = np.array([[True, False]])
    mask = single_scatter_mask(inter, valid)
    assert mask[0].tolist() == [True, False]
