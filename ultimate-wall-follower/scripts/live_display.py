#!/usr/bin/env python3
"""
Stream the annotated LIDAR frame to the screen.

Same overlay as capture_lidar.py (wall beams, forward probe, lookahead radius,
commanded path, timestamp, PD coefficients), refreshed live. Intended for a
machine SSH'ed in with X11 forwarding (ssh -Y), where the window renders over
the forwarded display.

Usage:
    ssh -Y user@robot
    python3 scripts/live_display.py            # live scan from the robot
    python3 scripts/live_display.py --demo      # synthetic moving scan, no ROS
    python3 scripts/live_display.py --selftest  # headless render check, no display

Stop with the window close button or Ctrl-C.
"""

import argparse
import math
import os
import sys

import matplotlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def _demo_scan(frame):
    # Sweep the car side to side so every overlay is visible in motion.
    car_x = 35.0 * math.sin(frame / 25.0)
    import lidar_viz

    return lidar_viz.synthetic_corridor(car_x=car_x)


def run_selftest():
    """Render a few synthetic frames headlessly; exit non-zero on any error."""
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    import lidar_viz  # puts the project dir on sys.path
    import control

    cfg, speed = lidar_viz.load_cfg()
    controller = control.WallFollowController(speed=speed, config=cfg)
    fig, ax = plt.subplots(figsize=(8, 8))
    for frame in range(5):
        result = lidar_viz.render(ax, _demo_scan(frame * 10), cfg, speed, controller, "selftest")
        assert -1.0 <= result.angle <= 1.0 and -1.0 <= result.speed <= 1.0
    plt.close(fig)
    print("selftest ok: 5 frames rendered, outputs in range")


def run_live(demo, timeout, interval_ms):
    from datetime import datetime

    matplotlib.use("TkAgg")  # interactive backend; forwards over ssh -Y
    import matplotlib.animation as animation
    import matplotlib.pyplot as plt

    import lidar_viz  # puts the project dir on sys.path
    import control

    cfg, speed = lidar_viz.load_cfg()
    controller = control.WallFollowController(speed=speed, config=cfg)

    rc = None
    if not demo:
        rc, scan = lidar_viz.open_racecar(timeout=timeout)
        if scan.size == 0:
            print(f"WARNING: no LIDAR yet after {timeout}s; will keep polling live.")

    fig, ax = plt.subplots(figsize=(8, 8))
    state = {"frame": 0}

    def update(_):
        state["frame"] += 1
        if demo:
            scan = _demo_scan(state["frame"])
        else:
            samples = rc.lidar.get_samples()
            scan = samples if samples is not None else []
        lidar_viz.render(ax, scan, cfg, speed, controller, datetime.now().strftime("%Y%m%d_%H%M%S"))

    ani = animation.FuncAnimation(fig, update, interval=interval_ms)
    try:
        plt.show()
    finally:
        if rc is not None:
            try:
                rc._RacecarReal__running = False
            except AttributeError:
                pass
    return ani


def main():
    parser = argparse.ArgumentParser(description="Stream annotated LIDAR frames.")
    parser.add_argument("--demo", action="store_true", help="synthetic moving scan (no ROS)")
    parser.add_argument("--selftest", action="store_true", help="headless render check, no display")
    parser.add_argument("--timeout", type=float, default=5.0, help="seconds to wait for first live scan")
    parser.add_argument("--interval", type=int, default=100, help="refresh period in ms")
    args, _ = parser.parse_known_args()

    if args.selftest:
        run_selftest()
        return
    run_live(args.demo, args.timeout, args.interval)


if __name__ == "__main__":
    main()
