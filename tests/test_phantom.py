import numpy as np
import pytest
from wifi_radar_slam.eval.phantom import (match_detections, phantom_stats,
                                          phantom_stats_frames)


def test_a_detection_on_top_of_a_true_path_matches_it():
    m = match_detections(np.array([20.0]), np.array([0.1]),
                         np.array([20.0, 50.0]), np.array([0.1, 1.0]))
    assert m.tolist() == [0]


def test_a_detection_far_from_every_true_path_is_a_PHANTOM():
    # THE headline measurement. 3 m range scale, 10 deg azimuth scale, cost cap 3.0 ->
    # anything beyond ~9 m / ~30 deg of every real path corresponds to no propagation path.
    m = match_detections(np.array([20.0]), np.array([0.0]),
                         np.array([80.0]), np.array([0.0]))
    assert m.tolist() == [-1]


def test_the_match_is_the_NEAREST_true_path_in_normalised_space():
    m = match_detections(np.array([21.0]), np.array([0.0]),
                         np.array([25.0, 20.0]), np.array([0.0, 0.0]))
    assert m.tolist() == [1]                       # 20 m is nearer than 25 m


def test_range_and_azimuth_are_traded_off_by_their_own_scales():
    # 3 m of range error costs the same as 10 deg of azimuth error, by construction.
    far_in_range = match_detections(np.array([23.0]), np.array([0.0]),
                                    np.array([20.0]), np.array([0.0]))
    far_in_angle = match_detections(np.array([20.0]), np.array([np.deg2rad(10.0)]),
                                    np.array([20.0]), np.array([0.0]))
    assert far_in_range.tolist() == [0] and far_in_angle.tolist() == [0]
    # but 4x either one is beyond the cost cap
    assert match_detections(np.array([32.0]), np.array([0.0]),
                            np.array([20.0]), np.array([0.0])).tolist() == [-1]


def test_azimuth_difference_wraps():
    m = match_detections(np.array([20.0]), np.array([np.deg2rad(179.0)]),
                         np.array([20.0]), np.array([np.deg2rad(-179.0)]))
    assert m.tolist() == [0]                       # 2 deg apart, not 358


def test_phantom_stats_counts_and_rate():
    det_r = np.array([20.0, 21.0, 90.0])           # two real, one phantom
    det_a = np.array([0.0, 0.0, 0.0])
    tr_r = np.array([20.0, 21.0])
    tr_a = np.array([0.0, 0.0])
    s = phantom_stats(det_r, det_a, tr_r, tr_a)
    assert s["n_detections"] == 3
    assert s["n_phantoms"] == 1
    assert s["phantom_rate"] == pytest.approx(1 / 3)


def test_range_bias_is_the_median_SIGNED_error_over_MATCHED_detections():
    # Paper 2 reported a 6.45 m median range BIAS -- a systematic offset, not scatter -- so
    # the sign must survive. Phantoms are excluded: they have no true path to be biased
    # against, and including them would measure nothing.
    det_r = np.array([22.0, 23.0, 200.0])          # +2, +3 on real paths; one phantom
    det_a = np.zeros(3)
    tr_r = np.array([20.0, 20.0])
    tr_a = np.zeros(2)
    s = phantom_stats(det_r, det_a, tr_r, tr_a)
    assert s["n_phantoms"] == 1
    assert s["range_bias_m"] == pytest.approx(2.5)          # median of [+2, +3]


def test_resolution_scaled_tolerance_is_stricter_for_a_fine_sensor():
    # A 0.5 m detection error is well inside a 3 m fixed tolerance, but OUTSIDE a
    # resolution-scaled tolerance for a sensor that resolves to 0.0375 m (cell D). Reporting
    # only the fixed tolerance would let a high-resolution sensor look phantom-free BY
    # CONSTRUCTION -- exactly the rigged comparison this paper exists to avoid.
    det_r, det_a = np.array([20.5]), np.array([0.0])
    tr_r, tr_a = np.array([20.0]), np.array([0.0])
    loose = phantom_stats(det_r, det_a, tr_r, tr_a, range_scale_m=3.0)
    tight = phantom_stats(det_r, det_a, tr_r, tr_a, range_scale_m=3 * 0.0375)
    assert loose["phantom_rate"] == 0.0
    assert tight["phantom_rate"] == 1.0


def test_no_true_paths_means_every_detection_is_a_phantom():
    s = phantom_stats(np.array([10.0, 20.0]), np.zeros(2), np.empty(0), np.empty(0))
    assert s["phantom_rate"] == 1.0
    assert np.isnan(s["range_bias_m"])


def test_no_detections_gives_nan_rate_not_zero():
    # An empty detection set has NO phantom rate. Reporting 0 % would read as "perfect",
    # when in fact the sensor is blind.
    s = phantom_stats(np.empty(0), np.empty(0), np.array([10.0]), np.array([0.0]))
    assert s["n_detections"] == 0
    assert np.isnan(s["phantom_rate"])


# --- per-frame aggregation ------------------------------------------------------

def test_frames_are_matched_INDEPENDENTLY_not_pooled():
    # THE bug this guards, and it would have produced a flattering, false headline.
    # A detection in frame 0 must be explained by a true path that existed IN FRAME 0. If
    # frames are pooled, a detection can be "explained" by a path from a completely different
    # vehicle position -- which massively UNDERCOUNTS phantoms. Paper 2 matched per frame; so
    # must we, or the ~89% comparison is meaningless.
    #
    # Here: frame 0's detection at 20 m matches nothing in frame 0 (whose only path is at
    # 90 m), but WOULD match frame 1's path at 20 m if the frames were pooled.
    det_r = [np.array([20.0]), np.array([90.0])]
    det_a = [np.array([0.0]), np.array([0.0])]
    tru_r = [np.array([90.0]), np.array([20.0])]
    tru_a = [np.array([0.0]), np.array([0.0])]
    s = phantom_stats_frames(det_r, det_a, tru_r, tru_a)
    assert s["n_detections"] == 2
    assert s["n_phantoms"] == 2                    # BOTH are phantoms, frame by frame
    assert s["phantom_rate"] == 1.0
    # and if we (wrongly) pooled them, neither would be a phantom -- proving the test bites
    pooled = phantom_stats(np.concatenate(det_r), np.concatenate(det_a),
                           np.concatenate(tru_r), np.concatenate(tru_a))
    assert pooled["n_phantoms"] == 0


def test_frames_aggregate_counts_and_pool_the_bias():
    det_r = [np.array([22.0]), np.array([23.0, 200.0])]
    det_a = [np.array([0.0]), np.array([0.0, 0.0])]
    tru_r = [np.array([20.0]), np.array([20.0])]
    tru_a = [np.array([0.0]), np.array([0.0])]
    s = phantom_stats_frames(det_r, det_a, tru_r, tru_a)
    assert s["n_detections"] == 3
    assert s["n_phantoms"] == 1
    assert s["range_bias_m"] == pytest.approx(2.5)         # median of [+2, +3] across frames


def test_frames_with_no_detections_contribute_nothing():
    s = phantom_stats_frames([np.empty(0), np.array([20.0])],
                             [np.empty(0), np.array([0.0])],
                             [np.array([20.0]), np.array([20.0])],
                             [np.array([0.0]), np.array([0.0])])
    assert s["n_detections"] == 1
    assert s["n_phantoms"] == 0


def test_frames_with_no_detections_at_all_give_nan():
    s = phantom_stats_frames([np.empty(0)], [np.empty(0)],
                             [np.array([20.0])], [np.array([0.0])])
    assert s["n_detections"] == 0
    assert np.isnan(s["phantom_rate"])
