import pytest
from wifi_radar_slam.radar.cells import CELLS


def test_the_four_cells_exist():
    assert set(CELLS) == {"A", "B", "C", "D"}


def test_A_to_B_changes_ONLY_the_geometry():
    # The whole ablation rests on one-variable-at-a-time. If A and B ever differ in carrier or
    # bandwidth, the A->B step stops measuring geometry and the paper's decomposition is
    # meaningless.
    a, b = CELLS["A"], CELLS["B"]
    assert a.config.carrier_hz == b.config.carrier_hz
    assert a.config.bandwidth_hz == b.config.bandwidth_hz
    assert a.monostatic is False and b.monostatic is True


def test_B_to_C_changes_ONLY_the_carrier():
    b, c = CELLS["B"], CELLS["C"]
    assert b.config.bandwidth_hz == c.config.bandwidth_hz
    assert b.monostatic is True and c.monostatic is True
    assert b.config.carrier_hz == 5.2e9 and c.config.carrier_hz == 77e9


def test_C_to_D_changes_ONLY_the_bandwidth():
    c, d = CELLS["C"], CELLS["D"]
    assert c.config.carrier_hz == d.config.carrier_hz == 77e9
    assert c.monostatic is True and d.monostatic is True
    assert c.config.bandwidth_hz == 160e6 and d.config.bandwidth_hz == 4e9


def test_every_cell_uses_the_SAME_detection_chain():
    # "CFAR" everywhere. A cell that quietly switched front-end would confound the algorithm
    # with the physics the ablation is trying to isolate.
    assert {c.front_end for c in CELLS.values()} == {"cfar"}


def test_bandwidth_sets_range_resolution_and_carrier_does_not():
    # The physical claim the C->D step tests: resolution is c/2B, independent of carrier.
    assert CELLS["B"].config.range_resolution_m == pytest.approx(
        CELLS["C"].config.range_resolution_m)
    assert CELLS["D"].config.range_resolution_m < 0.1
