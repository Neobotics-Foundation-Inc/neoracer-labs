#!/usr/bin/env python3
"""
Snapshot one LIDAR frame, annotate it, and save it to data/.

Captures the current scan, overlays the wall beams, forward probe, lookahead
radius, and commanded path, stamps the time and PD coefficients in the corner,
and writes data/YYYYMMDD_HHMMSS.png. Run before operating to confirm the LIDAR,
the geometry, and the controller all agree.

Usage:
    python3 scripts/capture_lidar.py            # live scan from the robot
    python3 scripts/capture_lidar.py --demo     # synthetic scan, no ROS needed
    python3 scripts/capture_lidar.py --timeout 8
"""

import argparse
import os
import sys
from datetime import datetime

import matplotlib

matplotlib.use("Agg")  # headless save; no display required
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lidar_viz
import control  # lidar_viz put the project dir on sys.path


def main():
    parser = argparse.ArgumentParser(description="Capture and annotate one LIDAR frame.")
    parser.add_argument("--demo", action="store_true", help="use a synthetic scan (no ROS)")
    parser.add_argument("--timeout", type=float, default=5.0, help="seconds to wait for a live scan")
    args, _ = parser.parse_known_args()

    cfg, speed = lidar_viz.load_cfg()
    controller = control.WallFollowController(speed=speed, config=cfg)

    rc = None
    if args.demo:
        scan = lidar_viz.synthetic_corridor()
    else:
        rc, scan = lidar_viz.open_racecar(timeout=args.timeout)
        if scan.size == 0:
            print(f"ERROR: no LIDAR data within {args.timeout}s. Is the driver publishing /scan?")

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    fig, ax = plt.subplots(figsize=(8, 8))
    lidar_viz.render(ax, scan, cfg, speed, controller, ts)

    os.makedirs(lidar_viz.DATA_DIR, exist_ok=True)
    out_path = os.path.join(lidar_viz.DATA_DIR, f"{ts}.png")
    fig.savefig(out_path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"saved {out_path}")

    if rc is not None:
        try:
            rc._RacecarReal__running = False
        except AttributeError:
            pass


if __name__ == "__main__":
    main()
