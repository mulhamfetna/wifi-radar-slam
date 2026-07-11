"""Paper-2 cost model (RQ5): WiFi sensing package vs the automotive LiDAR envelope.

Joins sourced prices (data/cost_data.yaml) with the RQ3 accuracy numbers and emits:
  1. absolute cost table (WiFi package vs each LiDAR tier)
  2. cost-ratio envelope (x cheaper), headline + deployed-AP sensitivity
  3. cost-normalized performance ($.ATE localization value, $/IoU mapping value)
Runs locally in seconds; no Sionna, no server.

    python experiments/run_cost_model.py
"""
import json

from wifi_radar_slam.cost import (load_cost_data, wifi_package_cost, lidar_envelope,
                                  cost_ratio, cost_normalized)

COST_YAML = "data/cost_data.yaml"
WIFI = "data/wifi_results.json"
LIDAR_A = "data/lidar_geo_results.json"
LIDAR_B = "data/lidar_sionna_results.json"


def _fmt(rng, unit="", nd=0):
    lo, hi = rng
    if lo == float("inf"):
        return "inf (no mapping value at any price)"
    if nd:
        return f"{lo:,.{nd}f}-{hi:,.{nd}f}{unit}"
    return f"{lo:,.0f}-{hi:,.0f}{unit}"


def main() -> None:
    data = load_cost_data(COST_YAML)
    wifi = json.load(open(WIFI))
    lidar = {"A (geometric)": json.load(open(LIDAR_A)),
             "B (Sionna diffuse)": json.load(open(LIDAR_B))}

    pkg = wifi_package_cost(data)                       # headline: pi_nexmon + antennas
    pkg_esp = wifi_package_cost(data, rx="esp32")
    pkg_dep3 = wifi_package_cost(data, n_aps=3)         # deployed-AP sensitivity
    tiers = lidar_envelope(data)

    print("## Cost (RQ5)\n")
    print(f"WiFi package (headline, ambient-free: Pi4+nexmon + antennas): "
          f"**${_fmt(pkg)}**")
    print(f"WiFi package (low-end ESP32 + antennas): ${_fmt(pkg_esp)}")
    print(f"WiFi package (+3 deployed APs, sensitivity): ${_fmt(pkg_dep3)}\n")

    print("| LiDAR tier | Automotive? | Price (USD) | WiFi is X cheaper (headline) "
          "| X cheaper (+3 APs) |")
    print("|---|---|---:|---:|---:|")
    for t in tiers:
        r = cost_ratio(pkg, (t["low"], t["high"]))
        rd = cost_ratio(pkg_dep3, (t["low"], t["high"]))
        auto = "yes" if t["automotive_grade"] else "**no (2D floor)**"
        print(f"| {t['name']} | {auto} | ${_fmt((t['low'], t['high']))} "
              f"| {_fmt(r, 'x', 1)} | {_fmt(rd, 'x', 1)} |")

    print("\n### Cost-normalized performance (price read WITH accuracy)\n")
    print("$.m = price x ATE (localization value, lower better). "
          "$/IoU = price per IoU point (mapping value, lower better).\n")
    print("| Scene | Sensor | Price (USD) | $.m (localization) | $/IoU (mapping) |")
    print("|---|---|---:|---:|---:|")
    # CONSISTENCY: models A/B were measured at OUSTER_OS1 params, so their measured
    # performance must be priced at the OS1 tier -- NOT at a cheap solid-state tier we
    # never simulated. Pricing high-end performance at budget-LiDAR cost would be
    # apples-to-oranges (and would flatter LiDAR).
    os1 = next(t for t in tiers if t["key"] == "ouster_os1")
    rows = []
    for scene in ("controlled_wall", "street_canyon"):
        w = wifi[scene]["realistic"]
        rows.append((scene, "WiFi (realistic CSI)", pkg, w.get("ate"), w.get("iou", 0.0)))
        for name, res in lidar.items():
            m = res[scene]
            rows.append((scene, f"LiDAR {name} @ OS1 price", (os1["low"], os1["high"]),
                         m["ate"], m["iou"]))
    for scene, sensor, price, ate, iou in rows:
        acc = cost_normalized(price, ate, "accuracy")
        cov = cost_normalized(price, iou, "coverage")
        print(f"| {scene} | {sensor} | ${_fmt(price)} | {_fmt(acc, '', 2)} "
              f"| {_fmt(cov, '', 0)} |")

    out = {
        "wifi_package_usd": {"headline": pkg, "esp32": pkg_esp, "deployed_3ap": pkg_dep3},
        "lidar_tiers": [{"key": t["key"], "low": t["low"], "high": t["high"],
                         "automotive_grade": t["automotive_grade"],
                         "ratio_vs_wifi_headline": cost_ratio(pkg, (t["low"], t["high"]))}
                        for t in tiers],
    }
    with open("data/cost_results.json", "w") as f:
        json.dump(out, f, indent=2)
    print("\nsaved -> data/cost_results.json")


if __name__ == "__main__":
    main()
