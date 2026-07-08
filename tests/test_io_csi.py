import numpy as np
from wifi_radar_slam.io_csi import csi_frames_to_pipeline


def test_frames_to_pipeline_shape_and_modal_filter():
    rng = np.random.default_rng(0)
    # 4 frames of (n_sub=30, n_rx=3, n_tx=1) + 1 odd frame with a different shape
    good = [rng.normal(size=(30, 3, 1)) + 1j * rng.normal(size=(30, 3, 1)) for _ in range(4)]
    odd = rng.normal(size=(30, 2, 1)) + 1j * rng.normal(size=(30, 2, 1))
    out = csi_frames_to_pipeline(good + [odd])
    # -> (n_frames, n_ap, n_rx_antennas, n_subcarriers); odd frame dropped
    assert out.shape == (4, 1, 3, 30)
    assert np.iscomplexobj(out)
    # a 2-D frame (n_sub, n_rx) gets a tx axis added
    out2 = csi_frames_to_pipeline([rng.normal(size=(30, 3)) + 0j])
    assert out2.shape == (1, 1, 3, 30)


def test_frames_to_pipeline_empty():
    out = csi_frames_to_pipeline([])
    assert out.shape[0] == 0
