from wifi_radar_slam.cost import load_cost_data

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
