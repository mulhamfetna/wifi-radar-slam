import pytest
from wifi_radar_slam.cost import (load_cost_data, wifi_package_cost, lidar_envelope,
                                  cost_ratio)

COST_YAML = "data/cost_data.yaml"


def test_cost_data_has_wifi_and_lidar_tiers():
    d = load_cost_data(COST_YAML)
    assert d["currency"] == "USD"
    # WiFi receivers include the headline (Pi+nexmon) and low-end (ESP32) options
    rx = d["wifi"]["receivers"]
    assert {"pi_nexmon", "esp32"} <= set(rx)
    assert rx["esp32"]["low"] < rx["pi_nexmon"]["high"]
    # antennas and an AP unit (for the deployed sensitivity) exist
    assert d["wifi"]["antennas"]["low"] > 0
    assert d["wifi"]["ap_unit"]["low"] > 0
    # LiDAR envelope spans legacy spinning down to a budget 2D floor, and MUST include
    # the ouster_os1 tier -- the sensor whose params models A/B actually simulate.
    tiers = {t["key"]: t for t in d["lidar_tiers"]}
    assert {"legacy_spinning", "ouster_os1", "mid_solid_state",
            "cheap_solid_state", "budget_2d"} <= set(tiers)
    assert tiers["legacy_spinning"]["low"] > tiers["ouster_os1"]["high"]
    assert tiers["ouster_os1"]["low"] > tiers["mid_solid_state"]["high"]
    # the budget 2D scanner is explicitly NOT an automotive peer (honesty guard)
    assert tiers["budget_2d"]["automotive_grade"] is False
    assert tiers["cheap_solid_state"]["automotive_grade"] is True


def test_every_price_entry_is_sourced_and_dated():
    d = load_cost_data(COST_YAML)
    entries = list(d["wifi"]["receivers"].values()) + [
        d["wifi"]["antennas"], d["wifi"]["ap_unit"]] + d["lidar_tiers"]
    for e in entries:
        assert e["low"] <= e["high"]
        assert e["source"] and isinstance(e["source"], str)   # text citation, no fabricated URLs
        assert e["date"]


def test_wifi_package_cost_headline_and_deployed():
    d = load_cost_data(COST_YAML)
    lo, hi = wifi_package_cost(d)                       # default rx=pi_nexmon + antennas
    exp_lo = d["wifi"]["receivers"]["pi_nexmon"]["low"] + d["wifi"]["antennas"]["low"]
    exp_hi = d["wifi"]["receivers"]["pi_nexmon"]["high"] + d["wifi"]["antennas"]["high"]
    assert (lo, hi) == (exp_lo, exp_hi)
    # low-end receiver is cheaper
    assert wifi_package_cost(d, rx="esp32")[0] < lo
    # deployed-AP sensitivity adds n_aps * ap_unit
    ap = d["wifi"]["ap_unit"]
    lo3, hi3 = wifi_package_cost(d, n_aps=3)
    assert lo3 == exp_lo + 3 * ap["low"]
    assert hi3 == exp_hi + 3 * ap["high"]


def test_unknown_receiver_raises():
    d = load_cost_data(COST_YAML)
    with pytest.raises(KeyError):
        wifi_package_cost(d, rx="not_a_receiver")


def test_lidar_envelope_ordered_and_flagged():
    d = load_cost_data(COST_YAML)
    tiers = lidar_envelope(d)
    assert len(tiers) == 5
    keys = [t["key"] for t in tiers]
    assert "budget_2d" in keys and "ouster_os1" in keys
    assert all(isinstance(t["low"], float) for t in tiers)


def test_cost_ratio_is_a_conservative_range():
    # WiFi $50-100 vs LiDAR $500-600:
    #   conservative low  = cheapest LiDAR / priciest WiFi = 500/100 = 5x
    #   optimistic  high  = priciest  LiDAR / cheapest WiFi = 600/50  = 12x
    lo, hi = cost_ratio((50.0, 100.0), (500.0, 600.0))
    assert lo == pytest.approx(5.0)
    assert hi == pytest.approx(12.0)
