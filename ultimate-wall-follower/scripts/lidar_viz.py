"""
Shared LIDAR visualization for the validation scripts.

Renders one scan in the car frame (forward = up, right = +x) with the exact
features the controller uses: the four wall beams, the forward clearance probe,
the lookahead radius, and the commanded path, plus a timestamp and the PD
coefficients. capture_lidar.py saves a single frame; live_display.py streams it.

The scan, Config, and geometry come straight from control.py and
config/wall_follower.yaml, so the overlay matches what wall_follower.py acts on.
No matplotlib backend is selected here; the entry scripts choose one (Agg to
save, an interactive backend to stream).
"""

import math
import os
import sys

import numpy as np
from matplotlib.patches import Circle, Polygon

_HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(_HERE)
LIBRARY_DIR = os.path.normpath(os.path.join(PROJECT_DIR, "..", "..", "library"))
DATA_DIR = os.path.join(PROJECT_DIR, "data")
CONFIG_PATH = os.path.join(PROJECT_DIR, "config", "wall_follower.yaml")

sys.path.insert(0, PROJECT_DIR)  # import control (pure, no ROS)
import control

# Steering kinematics for the commanded-path sketch only. The real wheelbase and
# max steering angle are not exposed by the driver, so these are approximate and
# affect the drawn arc's curvature, nothing the controller does.
MAX_STEER_RAD = math.radians(20.0)
WHEELBASE_CM = 24.0


def load_cfg():
    """Return (Config, speed) from the shipped YAML, or defaults if unreadable."""
    try:
        return control.load_config(CONFIG_PATH)
    except (FileNotFoundError, ImportError):
        return control.Config(), control.DEFAULT_SPEED


def synthetic_corridor(car_x=-25.0, half_width=100.0, front=350.0, n=1080):
    """A demo scan (cm, clockwise, 0 = front) for a corridor with a front wall.

    car_x < 0 places the car nearer the left wall, so the controller should
    command a right turn. Used by --demo and the offline self-test.
    """
    angles = np.arange(n) * 2.0 * np.pi / n
    sin_a, cos_a = np.sin(angles), np.cos(angles)
    scan = np.zeros(n, dtype=float)
    for i in range(n):
        hits = []
        for wall_x in (half_width, -half_width):  # right (+x) and left (-x) walls
            if abs(sin_a[i]) > 1e-6:
                t = (wall_x - car_x) / sin_a[i]
                if t > 0:
                    hits.append(t)
        if cos_a[i] > 1e-6:  # front wall at y = front
            hits.append(front / cos_a[i])
        scan[i] = min(hits) if hits else 0.0
    return scan


def _ray_xy(distance, angle_deg):
    a = math.radians(angle_deg)
    return distance * math.sin(a), distance * math.cos(a)


def _path_xy(angle, length, n=48):
    """Constant-curvature path from the steering command (positive = right)."""
    steer = angle * MAX_STEER_RAD
    kappa = math.tan(steer) / WHEELBASE_CM
    s = np.linspace(0.0, length, n)
    if abs(kappa) < 1e-6:
        return np.zeros_like(s), s
    return (1.0 - np.cos(kappa * s)) / kappa, np.sin(kappa * s) / kappa


def render(ax, scan, cfg, speed, controller, ts_str):
    """Draw one annotated frame on ax. Returns the controller Result."""
    scan = np.asarray(scan, dtype=float)
    result = controller.compute(scan)
    lookahead = cfg.lookahead_base_cm + cfg.lookahead_gain_cm * speed
    view = max(200.0, lookahead * 1.5)

    ax.clear()
    ax.set_aspect("equal")
    ax.set_xlim(-view, view)
    ax.set_ylim(-0.5 * view, 1.5 * view)
    ax.set_xlabel("x right (cm)")
    ax.set_ylabel("y forward (cm)")
    ax.set_title("LIDAR validation - car frame (forward up, right +x)")
    ax.grid(True, alpha=0.2)

    if scan.size == 0 or not np.any(scan > 0):
        ax.text(0.5, 0.5, "NO LIDAR DATA", transform=ax.transAxes, ha="center",
                va="center", color="red", fontsize=20)
        return result

    # Scan points.
    n = scan.size
    angles = np.arange(n) * 2.0 * np.pi / n
    valid = scan > 0
    xs = scan[valid] * np.sin(angles[valid])
    ys = scan[valid] * np.cos(angles[valid])
    ax.scatter(xs, ys, s=2, c="0.4", label="scan")

    # Forward clearance probe (narrow front cone).
    half_fw = cfg.front_window_deg / 2.0
    pr = view * 1.4
    e_left = _ray_xy(pr, -half_fw)
    e_right = _ray_xy(pr, half_fw)
    ax.add_patch(Polygon([(0, 0), e_right, e_left], closed=True, color="tab:cyan",
                         alpha=0.15, label="forward probe"))

    # Wall beams (abeam + forward-tilted, per side) and the inferred wall slices.
    theta = cfg.beam_theta_deg
    beams = [
        (cfg.abeam_right_deg, "tab:orange", "R abeam"),
        (cfg.abeam_right_deg - theta, "tab:orange", "R fwd"),
        (cfg.abeam_left_deg, "tab:green", "L abeam"),
        (cfg.abeam_left_deg + theta, "tab:green", "L fwd"),
    ]
    ends = {}
    for deg, color, label in beams:
        d = control.average_distance(scan, deg, cfg.beam_window_deg)
        if d > 0:
            x, y = _ray_xy(d, deg)
            ax.plot([0, x], [0, y], color=color, lw=1.2, label=label)
            ax.plot(x, y, "o", color=color, ms=4)
            ends[deg] = (x, y)
    for a_deg, f_deg, color in (
        (cfg.abeam_right_deg, cfg.abeam_right_deg - theta, "tab:orange"),
        (cfg.abeam_left_deg, cfg.abeam_left_deg + theta, "tab:green"),
    ):
        if a_deg in ends and f_deg in ends:
            (x1, y1), (x2, y2) = ends[a_deg], ends[f_deg]
            dx, dy = x1 - x2, y1 - y2
            ax.plot([x2 - dx, x1 + dx], [y2 - dy, y1 + dy], color=color, lw=2.0, alpha=0.6)

    # Lookahead radius and point.
    ax.add_patch(Circle((0, 0), lookahead, fill=False, ls="--", ec="tab:blue",
                        label=f"lookahead {lookahead:.0f}cm"))
    ax.plot(0, lookahead, "x", color="tab:blue", ms=8)

    # Commanded path.
    px, py = _path_xy(result.angle, lookahead * 1.5)
    ax.plot(px, py, color="magenta", lw=2.0, ls="--", label="desired path")

    # Car marker.
    ax.plot(0, 0, marker="^", color="black", ms=12)

    # Timestamp + PD coefficients + live state, top-left.
    info = (
        f"{ts_str}\n"
        f"kp={cfg.kp:g}  kd={cfg.kd:g}\n"
        f"speed={speed:g}  L={lookahead:.0f}cm\n"
        f"mode={result.mode}  angle={result.angle:+.2f}\n"
        f"v_cmd={result.speed:+.2f}  fwd={result.forward_clear_cm:.0f}cm\n"
        f"blend={result.blend:.2f}  err={result.error_cm:+.0f}cm"
    )
    ax.text(0.02, 0.98, info, transform=ax.transAxes, va="top", ha="left",
            family="monospace", fontsize=8, color="white",
            bbox=dict(boxstyle="round", fc="black", alpha=0.6))
    ax.legend(loc="upper right", fontsize=7, framealpha=0.6)
    return result


def open_racecar(timeout=5.0):
    """Create the real racecar, spin it async (no driving), and wait for a scan.

    Returns (racecar, scan). scan is empty if none arrived within timeout. Imports
    racecar_core here so --demo works on machines without ROS.
    """
    import time

    sys.path.insert(0, LIBRARY_DIR)
    import racecar_core

    rc = racecar_core.create_racecar(isSimulation=False)
    rc.set_start_update(lambda: rc.drive.stop(), lambda: None)
    rc.go_async()

    deadline = time.time() + timeout
    scan = np.array([])
    while time.time() < deadline:
        samples = rc.lidar.get_samples()
        if samples is not None and len(samples) > 0 and np.any(np.asarray(samples) > 0):
            scan = np.asarray(samples, dtype=float)
            break
        time.sleep(0.05)
    return rc, scan
