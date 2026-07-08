import numpy as np
from wifi_radar_slam.dataset import CsiDataset, PATH_COLUMNS


def test_dataset_roundtrip(tmp_path):
    rng = np.random.default_rng(0)
    ds = CsiDataset(
        csi=(rng.normal(size=(5, 2, 4, 8)) + 1j * rng.normal(size=(5, 2, 4, 8))).astype(np.complex64),
        poses=rng.normal(size=(5, 3)),
        ap_positions=rng.normal(size=(2, 3)),
        gt_map=rng.normal(size=(10, 2)),
        paths=rng.normal(size=(20, len(PATH_COLUMNS))),
        meta={"scene": "test", "carrier_hz": 5.2e9},
    )
    p = tmp_path / "d.npz"
    ds.save(str(p))
    back = CsiDataset.load(str(p))
    assert back.csi.shape == ds.csi.shape and np.iscomplexobj(back.csi)
    assert np.allclose(back.poses, ds.poses)
    assert back.meta["scene"] == "test"
    pf = back.path_frame()
    assert set(pf) == set(PATH_COLUMNS)
    assert pf["delay_s"].shape == (20,)
