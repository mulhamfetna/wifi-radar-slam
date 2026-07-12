import json
import subprocess
import sys
from pathlib import Path


def test_figure_data_comes_from_the_committed_jsons():
    from experiments.make_paper2_figures import figure_data
    d = figure_data()
    # RQ3 numbers must equal what is in the result files (no hand-typed values)
    lidar_a = json.load(open("data/lidar_geo_results.json"))
    assert d["rq3"]["controlled_wall"]["LiDAR-A"]["ate"] == \
        lidar_a["controlled_wall"]["ate"]
    # the mapping-ceiling figure must use the isolation result
    iso = json.load(open("data/mapping_floor_isolation.json"))
    assert d["ceiling"]["controlled_wall"]["no_plausible_match_pct"] == \
        iso["controlled_wall"]["no_plausible_match_pct"]


def test_script_renders_all_five_figures():
    out = Path("docs/assets")
    subprocess.run([sys.executable, "experiments/make_paper2_figures.py"], check=True)
    for i in range(2, 7):
        f = out / f"paper2_fig{i}.pdf"
        assert f.exists() and f.stat().st_size > 1000, f"missing/empty {f}"
