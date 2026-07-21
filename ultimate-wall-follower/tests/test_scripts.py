"""Offline tests for the validation scripts' shared visualization.

Renders synthetic scans headlessly (Agg backend); no ROS, no display. The live
scan path (open_racecar) needs hardware and is not exercised here.
"""

import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_DIR, "scripts"))

import lidar_viz  # noqa: E402
import control  # noqa: E402


def _controller():
    cfg, speed = lidar_viz.load_cfg()
    return control.WallFollowController(speed=speed, config=cfg), cfg, speed


def test_synthetic_corridor_shape_and_values():
    scan = lidar_viz.synthetic_corridor()
    assert scan.shape == (1080,)
    assert (scan >= 0).all() and (scan > 0).any()


def test_load_cfg_matches_shipped():
    cfg, speed = lidar_viz.load_cfg()
    assert 0.0 <= speed <= 1.0
    assert cfg.kp > 0


def test_render_draws_and_matches_controller_sign():
    ctrl, cfg, speed = _controller()
    fig, ax = plt.subplots()
    scan = lidar_viz.synthetic_corridor(car_x=-25.0)  # nearer the left wall
    result = lidar_viz.render(ax, scan, cfg, speed, ctrl, "20260606_000000")
    assert result.mode in ("wall", "gap")
    assert result.angle > 0  # offset left -> steer right
    assert len(ax.lines) > 0  # beams / path drawn
    assert ax.get_title()
    plt.close(fig)


def test_render_handles_empty_scan():
    ctrl, cfg, speed = _controller()
    fig, ax = plt.subplots()
    result = lidar_viz.render(ax, [], cfg, speed, ctrl, "20260606_000000")
    assert result.mode == "no_data"
    plt.close(fig)


def test_path_curves_right_for_positive_angle():
    x_right, _ = lidar_viz._path_xy(0.5, 100.0)
    x_left, _ = lidar_viz._path_xy(-0.5, 100.0)
    x_straight, y_straight = lidar_viz._path_xy(0.0, 100.0)
    assert x_right[-1] > 0  # positive angle curves toward +x (right)
    assert x_left[-1] < 0
    assert abs(x_straight[-1]) < 1e-6 and y_straight[-1] > 0
