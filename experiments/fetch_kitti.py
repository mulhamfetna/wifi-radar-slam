"""Fetch one KITTI odometry sequence (default 04) from KITTI's public S3 using
HTTP range requests, so we download ~0.5 GB instead of the 84 GB full velodyne zip.

Server-only (needs network + `pip install remotezip`):
    .venv/bin/pip install remotezip
    nice -n 19 ionice -c3 python experiments/fetch_kitti.py
"""
import os
from remotezip import RemoteZip

BASE = "https://s3.eu-central-1.amazonaws.com/avg-kitti/"
SEQ = "04"
OUT = "data/kitti"


def main() -> None:
    vdir = f"{OUT}/sequences/{SEQ}/velodyne"
    os.makedirs(vdir, exist_ok=True)
    os.makedirs(f"{OUT}/poses", exist_ok=True)
    with RemoteZip(BASE + "data_odometry_poses.zip") as z:
        open(f"{OUT}/poses/{SEQ}.txt", "wb").write(z.read(f"dataset/poses/{SEQ}.txt"))
    with RemoteZip(BASE + "data_odometry_calib.zip") as z:
        open(f"{OUT}/sequences/{SEQ}/calib.txt", "wb").write(
            z.read(f"dataset/sequences/{SEQ}/calib.txt"))
    with RemoteZip(BASE + "data_odometry_velodyne.zip") as z:
        vel = sorted(n for n in z.namelist()
                     if f"/sequences/{SEQ}/velodyne/" in n and n.endswith(".bin"))
        print(f"downloading {len(vel)} velodyne frames for seq {SEQ}")
        for i, n in enumerate(vel):
            open(f"{vdir}/{os.path.basename(n)}", "wb").write(z.read(n))
            if i % 50 == 0:
                print("  ", i, "/", len(vel))
    print("done ->", vdir)


if __name__ == "__main__":
    main()
