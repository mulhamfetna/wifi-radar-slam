import numpy as np
from wifi_radar_slam.geometry import (
    straight_trajectory, velocity_from_poses, mirror_image, targets_to_pointmap,
    footprint_points,
)


def test_footprint_points_on_perimeter():
    pts = footprint_points([0.0, 0.0, 0.0], [4.0, 2.0, 3.0], spacing=1.0)
    assert pts.shape[1] == 2
    assert len(pts) > 0
    # every point lies on the rectangle boundary (on an x-edge or a y-edge)
    on_x_edge = np.isclose(pts[:, 0], 0.0) | np.isclose(pts[:, 0], 4.0)
    on_y_edge = np.isclose(pts[:, 1], 0.0) | np.isclose(pts[:, 1], 2.0)
    assert np.all(on_x_edge | on_y_edge)
    # all four corners are present
    for c in ([0, 0], [4, 0], [0, 2], [4, 2]):
        assert np.any(np.all(np.isclose(pts, c), axis=1))
    # nothing in the interior
    interior = (pts[:, 0] > 0.01) & (pts[:, 0] < 3.99) & (pts[:, 1] > 0.01) & (pts[:, 1] < 1.99)
    assert not np.any(interior)


def test_straight_trajectory_shape_and_endpoints():
    poses = straight_trajectory(length_m=60.0, speed_mps=5.0, timestep_s=0.05)
    assert poses.shape == (240, 3)
    assert np.isclose(poses[0, 0], 0.0)
    assert np.isclose(poses[-1, 0], 60.0 - 60.0 / 240)  # last sample just before end
    assert np.allclose(poses[:, 1], 0.0)               # straight along x
    assert np.allclose(poses[:, 2], 0.0)               # yaw 0


def test_velocity_constant():
    poses = straight_trajectory(60.0, 5.0, 0.05)
    vel = velocity_from_poses(poses, 0.05)
    assert vel.shape == (240, 2)
    assert np.allclose(vel[1:, 0], 5.0, atol=1e-6)     # 5 m/s in x


def test_mirror_image_across_y_wall():
    # wall in the plane y=6, normal +y; AP at y=20 mirrors to y=-8
    va = mirror_image(np.array([10.0, 20.0, 6.0]),
                      wall_point=np.array([0.0, 6.0, 0.0]),
                      wall_normal=np.array([0.0, 1.0, 0.0]))
    assert np.allclose(va, [10.0, -8.0, 6.0])


def test_pointmap_covers_boxes():
    targets = [{"kind": "pole", "center": [0.0, 0.0, 1.5], "size": [0.2, 0.2, 3.0]}]
    pts = targets_to_pointmap(targets, spacing=0.5)
    assert pts.shape[1] == 3
    assert pts.shape[0] > 0
    assert np.all(np.abs(pts[:, 0]) <= 0.2)            # within box x-extent
