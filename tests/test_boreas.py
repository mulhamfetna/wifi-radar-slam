import numpy as np
import pytest
from wifi_radar_slam.radar import boreas
from wifi_radar_slam.lidar.pointcloud import Scan


N_AZ, N_RG = 400, 3360
META = boreas.BOREAS_N_METADATA_COLS      # 11


def synthetic_image(target_bin=100, base_ts=1_606_417_097_528_152):
    """A Boreas-format polar image with one target at `target_bin` in every azimuth.

    Byte layout verified against a REAL file (2026-07-12):
      cols 0..7  int64  per-azimuth timestamp (us)
      cols 8..9  uint16 rotation encoder
      col  10    uint8  valid flag (255)
      cols 11..  uint8  power, 3360 range bins
    """
    img = np.zeros((N_AZ, META + N_RG), dtype=np.uint8)
    # encoder steps by 14 per azimuth (0.9 deg); 400 * 0.9 = 360 deg exactly
    enc = (12 + 14 * np.arange(N_AZ)).astype(np.uint16)
    img[:, 8:10] = enc.view(np.uint8).reshape(N_AZ, 2)
    ts = (base_ts + (np.arange(N_AZ) * 625)).astype(np.int64)     # ~250 ms / 400
    img[:, 0:8] = ts.view(np.uint8).reshape(N_AZ, 8)
    img[:, 10] = 255
    img[:, META + target_bin] = 200
    return img, enc, ts


def test_decode_shapes():
    img, _, _ = synthetic_image()
    power, az, ts, valid = boreas.decode_polar(img)
    assert power.shape == (N_AZ, N_RG)
    assert az.shape == (N_AZ,) and ts.shape == (N_AZ,) and valid.shape == (N_AZ,)
    assert valid.all()


def test_azimuth_decode_spans_exactly_one_revolution():
    # THE decode that must be right: azimuth = encoder * pi / 2800. Verified against a real
    # file -- the encoder steps by 14, which is 0.9 deg, and 400 x 0.9 = 360 deg exactly.
    img, enc, _ = synthetic_image()
    _, az, _, _ = boreas.decode_polar(img)
    assert np.allclose(az, enc.astype(float) * np.pi / 2800.0)
    assert np.rad2deg(az[1] - az[0]) == pytest.approx(0.9, abs=1e-6)
    assert np.rad2deg(az[-1] - az[0]) == pytest.approx(359.1, abs=1e-3)


def test_timestamp_decode_is_per_azimuth_and_monotone():
    # Each azimuth carries its OWN timestamp; a full rotation spans ~250 ms. That is what
    # makes motion compensation both possible and necessary.
    img, _, ts = synthetic_image()
    _, _, got, _ = boreas.decode_polar(img)
    assert np.array_equal(got, ts)
    assert np.all(np.diff(got) > 0)
    span_ms = (got[-1] - got[0]) / 1000.0
    assert 200.0 < span_ms < 300.0


def test_power_excludes_the_metadata_columns():
    # An off-by-11 here would silently shift EVERY range by 0.66 m and plant a bright
    # fictional target at zero range in every azimuth.
    img, _, _ = synthetic_image(target_bin=100)
    power, _, _, _ = boreas.decode_polar(img)
    assert power[:, 100].max() == 200
    assert power[:, :100].max() == 0


def test_range_bins():
    r = boreas.range_bins(N_RG)
    assert r.shape == (N_RG,)
    assert r[0] == pytest.approx(0.5 * boreas.BOREAS_RANGE_RES_M)
    assert np.diff(r)[0] == pytest.approx(boreas.BOREAS_RANGE_RES_M)
    assert r[-1] == pytest.approx(200.3, abs=0.1)      # 3360 bins x 0.0596 m


def test_decode_rejects_a_non_uint8_image():
    with pytest.raises(ValueError):
        boreas.decode_polar(np.zeros((400, 3371), dtype=np.float32))


def test_load_radar_scan_from_a_written_png(tmp_path):
    from PIL import Image
    img, _, _ = synthetic_image(target_bin=1000)       # 1000 * 0.0596 = 59.6 m
    p = tmp_path / "1606417097528152.png"
    Image.fromarray(img).save(p)
    scan = boreas.load_radar_scan(str(p), k=1, min_range_m=2.0, max_range_m=100.0)
    assert isinstance(scan, Scan)
    assert len(scan) == N_AZ                            # one return per azimuth
    r = np.linalg.norm(scan.points, axis=1)
    assert np.allclose(r, 1000.5 * boreas.BOREAS_RANGE_RES_M, atol=1e-6)


def test_load_gt_poses():
    csv = (
        "GPSTime,easting,northing,altitude,vel_east,vel_north,vel_up,"
        "roll,pitch,heading,angvel_z,angvel_y,angvel_x\n"
        "1606417097528152,0.0,0.0,0.5,0,0,0,3.12,-0.01,0.2,0,0,0\n"
        "1606417097778155,1.0,2.0,0.5,0,0,0,3.12,-0.01,0.3,0,0,0\n"
    )
    ts, poses = boreas.load_gt_poses(csv)
    assert ts.tolist() == [1606417097528152, 1606417097778155]
    assert poses.shape == (2, 3)
    assert np.allclose(poses[1, :2], [1.0, 2.0])
    assert poses[1, 2] == pytest.approx(0.3)            # heading -> yaw, radians


def test_gt_timestamps_match_the_scan_filenames():
    # The join key. GPSTime is EXACTLY the PNG filename -- no interpolation, and no chance of
    # an off-by-one frame shift.
    csv = ("GPSTime,easting,northing,altitude,vel_east,vel_north,vel_up,"
           "roll,pitch,heading,angvel_z,angvel_y,angvel_x\n"
           "1606417097528152,0.0,0.0,0.5,0,0,0,0,0,0.0,0,0,0\n")
    ts, _ = boreas.load_gt_poses(csv)
    assert str(ts[0]) == "1606417097528152"


def test_load_gt_poses_rejects_a_missing_column():
    with pytest.raises(ValueError, match="missing column"):
        boreas.load_gt_poses("GPSTime,easting,northing\n1,2,3\n")


# --- motion compensation --------------------------------------------------------

def test_motion_compensation_shifts_points_by_the_ego_motion_during_the_sweep():
    # A Navtech scan takes ~249 ms. At 10 m/s the car moves 2.5 m mid-sweep -- 40x the 0.06 m
    # range resolution. A point measured at the START of the sweep was taken from a position
    # 2.5 m behind where the car is at the reference time, so in the reference frame that point
    # must move BACKWARD along the direction of travel.
    pts = np.array([[10.0, 0.0], [10.0, 0.0]])
    t_az = np.array([0, 250_000], dtype=np.int64)      # first azimuth, last azimuth (us)
    out = boreas.motion_compensate(pts, t_az, 250_000, velocity_xy=(10.0, 0.0))
    assert np.allclose(out[1], [10.0, 0.0])            # last azimuth IS the reference -> unmoved
    assert np.allclose(out[0], [7.5, 0.0])             # 0.25 s earlier -> back by 10*0.25 = 2.5 m


def test_motion_compensation_is_a_no_op_at_zero_velocity():
    pts = np.array([[10.0, 0.0], [0.0, 5.0]])
    t_az = np.array([0, 250_000], dtype=np.int64)
    out = boreas.motion_compensate(pts, t_az, 250_000, velocity_xy=(0.0, 0.0))
    assert np.allclose(out, pts)


def test_motion_compensation_rejects_a_length_mismatch():
    with pytest.raises(ValueError):
        boreas.motion_compensate(np.zeros((3, 2)), np.zeros(2, dtype=np.int64), 0, (1.0, 0.0))


def test_load_radar_scan_with_velocity_undistorts(tmp_path):
    from PIL import Image
    img, _, _ = synthetic_image(target_bin=1000)
    p = tmp_path / "scan.png"
    Image.fromarray(img).save(p)
    still = boreas.load_radar_scan(str(p), k=1, max_range_m=100.0)
    moving = boreas.load_radar_scan(str(p), k=1, max_range_m=100.0, velocity_xy=(10.0, 0.0))
    assert len(still) == len(moving)
    assert not np.allclose(still.points, moving.points)          # it really moves them
    # bounded by v * sweep_duration = 10 m/s * 0.25 s = 2.5 m
    assert np.abs(still.points - moving.points).max() <= 2.6
