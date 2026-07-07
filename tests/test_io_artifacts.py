import numpy as np
from wifi_radar_slam import io_artifacts as io


def test_roundtrip_array(tmp_path, monkeypatch):
    monkeypatch.setattr(io, "RESULTS_ROOT", tmp_path)
    a = np.arange(6).reshape(2, 3).astype(float)
    io.save_array("r1", "channel", "csi", a)
    assert io.exists("r1", "channel", "csi")
    b = io.load_array("r1", "channel", "csi")
    assert np.allclose(a, b)


def test_roundtrip_json(tmp_path, monkeypatch):
    monkeypatch.setattr(io, "RESULTS_ROOT", tmp_path)
    io.save_json("r1", "eval", "metrics", {"ate": 0.3})
    assert io.load_json("r1", "eval", "metrics")["ate"] == 0.3
