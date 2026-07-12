"""Fetch N radar scans + the GT poses from the Boreas benchmark. Server-only (needs network).

Boreas is served over ANONYMOUS public HTTPS -- no registration, no credentials, and no `aws`
CLI (which the server does not have). That is precisely why we anchor on Boreas rather than
Oxford Radar RobotCar, whose download requires a registration that cannot be automated.

    nice -n 19 ionice -c3 .venv/bin/python experiments/fetch_boreas.py

Sequence boreas-2020-11-26-13-58 holds 12,426 scans (2.75 GB) at 4 Hz. We take the first
N_SCANS, which at 4 Hz is several km -- comfortably enough for KITTI's 800 m sub-sequences.
"""
from __future__ import annotations
import concurrent.futures as cf
import logging
import os
import re
import time
import urllib.parse
import urllib.request

BASE = "https://boreas.s3.amazonaws.com"
SEQ = "boreas-2020-11-26-13-58"
OUT = f"data/boreas/{SEQ}"
N_SCANS = 2500                     # ~625 s at 4 Hz; ~1.1 GB
WORKERS = 16

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("boreas")


def list_radar_keys(limit: int) -> list[str]:
    """List the first `limit` radar PNG keys, paginating the S3 REST listing."""
    keys: list[str] = []
    token = None
    while len(keys) < limit:
        url = f"{BASE}/?list-type=2&prefix={SEQ}/radar/&max-keys=1000"
        if token:
            url += "&continuation-token=" + urllib.parse.quote(token, safe="")
        with urllib.request.urlopen(url, timeout=60) as r:
            body = r.read().decode()
        keys += re.findall(r"<Key>([^<]+\.png)</Key>", body)
        m = re.search(r"<NextContinuationToken>([^<]+)</NextContinuationToken>", body)
        if not m:
            break
        token = m.group(1)
    return sorted(keys)[:limit]


def fetch(key: str) -> int:
    dest = os.path.join("data", "boreas", key)
    if os.path.exists(dest) and os.path.getsize(dest) > 0:
        return 0                                    # resumable: never re-download
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    with urllib.request.urlopen(f"{BASE}/{key}", timeout=120) as r:
        data = r.read()
    with open(dest, "wb") as f:
        f.write(data)
    return len(data)


def main() -> None:
    os.makedirs(f"{OUT}/applanix", exist_ok=True)

    log.info("fetching GT poses ...")
    with urllib.request.urlopen(f"{BASE}/{SEQ}/applanix/radar_poses.csv", timeout=120) as r:
        open(f"{OUT}/applanix/radar_poses.csv", "wb").write(r.read())
    log.info("  -> %s/applanix/radar_poses.csv", OUT)

    log.info("listing radar keys ...")
    keys = list_radar_keys(N_SCANS)
    log.info("  -> %d scans", len(keys))
    if not keys:
        raise SystemExit("no radar keys listed -- the bucket layout has changed; re-derive it "
                         "rather than guessing")

    t0 = time.time()
    total = 0
    with cf.ThreadPoolExecutor(max_workers=WORKERS) as pool:
        for i, n in enumerate(pool.map(fetch, keys), 1):
            total += n
            if i % 100 == 0 or i == len(keys):
                el = time.time() - t0
                eta = el / i * (len(keys) - i)
                log.info("  %4d/%d  %.2f GB  elapsed %.0fs  ETA %.0fs",
                         i, len(keys), total / 1e9, el, eta)
    log.info("done: %d scans, %.2f GB in %.0f s", len(keys), total / 1e9, time.time() - t0)


if __name__ == "__main__":
    main()
