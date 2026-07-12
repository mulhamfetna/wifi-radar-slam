import numpy as np
import pytest
from wifi_radar_slam.eval.mapping import map_under_gt_poses
from wifi_radar_slam.lidar.pointcloud import Scan


def test_a_single_scan_at_the_origin_maps_to_itself():
    m = map_under_gt_poses([Scan(np.array([[10.0, 0.0]]))], np.array([[0.0, 0.0, 0.0]]),
                           voxel=0.5)
    assert np.allclose(m, [[10.0, 0.0]])


def test_the_pose_is_applied_rotation_then_translation():
    scan = Scan(np.array([[10.0, 0.0]]))                    # 10 m ahead, sensor-local
    pose = np.array([[5.0, 5.0, np.pi / 2]])                # at (5,5), facing +y
    m = map_under_gt_poses([scan], pose, voxel=0.1)
    assert np.allclose(m, [[5.0, 15.0]], atol=1e-9)         # 10 m along +y from (5,5)


def test_scans_from_several_poses_accumulate_into_one_world_map():
    scans = [Scan(np.array([[10.0, 0.0]])), Scan(np.array([[10.0, 0.0]]))]
    poses = np.array([[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]])
    m = map_under_gt_poses(scans, poses, voxel=0.5)
    got = {tuple(np.round(p, 3)) for p in m}
    assert got == {(10.0, 0.0), (60.0, 0.0)}


def test_voxel_downsampling_collapses_duplicates():
    scans = [Scan(np.array([[10.0, 0.0], [10.1, 0.05]]))]   # same 1 m cell
    m = map_under_gt_poses(scans, np.array([[0.0, 0.0, 0.0]]), voxel=1.0)
    assert m.shape[0] == 1


def test_empty_scans_give_an_empty_map():
    m = map_under_gt_poses([Scan.empty(), Scan.empty()],
                           np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]), voxel=0.5)
    assert m.shape == (0, 2)


def test_mismatched_lengths_are_rejected():
    with pytest.raises(ValueError):
        map_under_gt_poses([Scan.empty()], np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]))
