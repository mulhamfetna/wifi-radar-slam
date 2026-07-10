"""LiDAR baseline substrate (paper 2): config, scans, ICP SLAM, runner.

Additive to the shared wifi_radar_slam pipeline; imports no Sionna/Mitsuba so it
runs in the default (fast) test suite. Sensor models A/B/C are added on later
branches against the `make_sensor` seam consumed by `runner.run_lidar`.
"""
