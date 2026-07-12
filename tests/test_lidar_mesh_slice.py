import numpy as np
from wifi_radar_slam.lidar.mesh_slice import _bbox_to_segments


def test_box_cut_by_scan_plane_yields_four_edges():
    segs = _bbox_to_segments(np.array([0.0, 0.0, -1.0]),
                             np.array([2.0, 4.0, 3.0]), z_height=1.0)
    assert segs.shape == (4, 2, 2)
    # the four rectangle corners are all present across the segment endpoints
    corners = {tuple(p) for s in segs for p in s}
    assert corners == {(0.0, 0.0), (2.0, 0.0), (2.0, 4.0), (0.0, 4.0)}


def test_box_not_spanning_scan_height_is_skipped():
    segs = _bbox_to_segments(np.array([0.0, 0.0, -1.0]),
                             np.array([2.0, 4.0, 0.5]), z_height=1.0)   # top at 0.5 < 1.0
    assert segs.shape == (0, 2, 2)
