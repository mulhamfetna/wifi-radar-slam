#!/bin/sh
# Fetch a few real commodity-WiFi CSI sample captures shipped with CSIKit
# (github.com/Gi-z/CSIKit, redistributed under CSIKit's licence) for the
# real-CSI proof-of-concept (experiments/run_real_csi.py). Not committed here.
set -e
DEST="$(dirname "$0")/../data/real"
mkdir -p "$DEST"
BASE="https://raw.githubusercontent.com/Gi-z/CSIKit/master/CSIKit/data"
for f in intel/log.all_csi.6.7.6.dat intel/example.dat nexmon/example_4358.pcap; do
    echo "fetching $f"
    curl -sSL -o "$DEST/$(basename "$f")" "$BASE/$f"
done
echo "done -> $DEST"
ls -la "$DEST"
