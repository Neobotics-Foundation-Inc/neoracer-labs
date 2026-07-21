"""Unit tests for control.py.

Scans are synthesized by ray-casting against explicit wall geometry, so the
LIDAR-to-steering sign conventions are verified against known scenes rather than
assumed. Distances are in cm; the car sits at the origin heading +x, with +y to
the left. Scan index i maps to clockwise angle i*360/n deg (0 = front).
"""

import math
import os

import numpy as np
import pytest

import control


N = 1080  # real RACECAR Neo scan length


def _ray_segment(origin, direction, a, b):
    """Positive distance from origin along direction to segment a->b, or None."""
    ox, oy = origin
    dx, dy = direction
    ax, ay = a
    bx, by = b
    ex, ey = bx - ax, by - ay
    det = dx * (-ey) - (-ex) * dy
    if abs(det) < 1e-12:
        return None
    rx, ry = ax - ox, ay - oy
    t = (rx * (-ey) - (-ex) * ry) / det
    u = (dx * ry - dy * rx) / det
    if t > 1e-9 and -1e-9 <= u <= 1.0 + 1e-9:
        return t
    return None


def build_scan(walls, pose=(0.0, 0.0, 0.0), n=N):
    """Ray-cast a scan. walls: list of (a, b) segments. pose: (x, y, yaw_deg),
    yaw positive = counter-clockwise (nose toward +y/left)."""
    x, y, yaw_deg = pose
    psi = math.radians(yaw_deg)
    cos_p, sin_p = math.cos(psi), math.sin(psi)
    scan = np.zeros(n, dtype=float)
    for i in range(n):
        phi = math.radians(i * 360.0 / n)
        cx, cy = math.cos(phi), -math.sin(phi)
        dx = cos_p * cx - sin_p * cy
        dy = sin_p * cx + cos_p * cy
        best = math.inf
        for a, b in walls:
            t = _ray_segment((x, y), (dx, dy), a, b)
            if t is not None and t < best:
                best = t
        scan[i] = 0.0 if math.isinf(best) else best
    return scan


def corridor(half_width_left=100.0, half_width_right=100.0, length=2000.0):
    """Two parallel walls along x at +left and -right."""
    return [
        ((-length, half_width_left), (length, half_width_left)),
        ((-length, -half_width_right), (length, -half_width_right)),
    ]


def mirror_scan(scan):
    """Reflect a scan left-right: clockwise angle a maps to 360 - a, so a point
    on the right (90 deg) moves to the left (270 deg)."""
    n = len(scan)
    return scan[[(n - i) % n for i in range(n)]]


# --- pure helpers -----------------------------------------------------------


def test_clamp_and_remap():
    assert control.clamp(5, 0, 1) == 1
    assert control.clamp(-5, 0, 1) == 0
    assert control.clamp(0.5, 0, 1) == 0.5
    assert control.remap(50, 0, 100, 0, 1) == 0.5
    assert control.remap(0, 0, 0, 0.3, 1) == 0.3  # degenerate input range


def test_signed_offset():
    assert control._signed_offset_deg(0) == 0
    assert control._signed_offset_deg(90) == 90  # right
    assert control._signed_offset_deg(270) == -90  # left
    assert control._signed_offset_deg(350) == -10


def test_average_distance_ignores_no_data():
    scan = np.full(N, 200.0)
    front = control._window_indices(N, -2, 2)
    scan[front] = 0.0  # no-data straight ahead
    assert control.average_distance(scan, 0.0, 4.0) == 0.0
    assert control.average_distance(scan, 90.0, 4.0) == 200.0


def test_closest_point_window():
    scan = np.full(N, 300.0)
    spike = control._window_indices(N, 88, 92)
    scan[spike] = 50.0
    ang, dist = control.closest_point(scan, 45, 135)
    assert dist == 50.0
    assert abs(control._signed_offset_deg(ang) - 90) < 3.0


def test_closest_point_empty_window():
    scan = np.zeros(N)
    ang, dist = control.closest_point(scan, -30, 30)
    assert ang is None and dist == 0.0


# --- wall geometry ----------------------------------------------------------


def test_wall_metrics_parallel_centered():
    scan = build_scan(corridor(100, 100))
    right = control.wall_metrics(scan, 90, 40, 2.0)
    left = control.wall_metrics(scan, 270, 320, 2.0)
    assert right is not None and left is not None
    assert abs(right[0] - 100) < 3.0
    assert abs(left[0] - 100) < 3.0
    assert abs(right[1]) < math.radians(1.5)
    assert abs(left[1]) < math.radians(1.5)


def test_wall_metrics_yaw_signs():
    # Nose turned left (psi=+10): away from right wall, toward left wall.
    scan = build_scan(corridor(100, 100), pose=(0, 0, 10))
    right = control.wall_metrics(scan, 90, 40, 2.0)
    left = control.wall_metrics(scan, 270, 320, 2.0)
    assert right[1] > math.radians(6)  # right alpha ~ +psi
    assert left[1] < -math.radians(6)  # left alpha ~ -psi
    assert abs(right[0] - 100) < 4.0  # perpendicular distance recovered
    assert abs(left[0] - 100) < 4.0


def test_wall_metrics_missing_wall():
    # Only a left wall present; right beams see nothing.
    walls = [((-2000, 100), (2000, 100))]
    scan = build_scan(walls)
    assert control.wall_metrics(scan, 90, 40, 6.0) is None


# --- centering controller ---------------------------------------------------


def _controller(speed=0.5):
    return control.WallFollowController(speed=speed)


def test_centered_straight_drives_straight():
    scan = build_scan(corridor(100, 100))
    res = _controller().compute(scan)
    assert res.mode == "wall"
    assert abs(res.angle) < 0.1
    assert res.speed > 0.0


def test_offset_toward_left_steers_right():
    scan = build_scan(corridor(100, 100), pose=(0, 30, 0))  # closer to left wall
    res = _controller().compute(scan)
    assert res.error_cm > 0
    assert res.angle > 0  # positive = right


def test_offset_toward_right_steers_left():
    scan = build_scan(corridor(100, 100), pose=(0, -30, 0))
    res = _controller().compute(scan)
    assert res.error_cm < 0
    assert res.angle < 0


def test_yaw_nose_left_corrects_right():
    # Centered position but nose yawed left: lookahead should command right.
    scan = build_scan(corridor(100, 100), pose=(0, 0, 12))
    res = _controller().compute(scan)
    assert res.angle > 0


# --- follow-the-gap ---------------------------------------------------------


def _front_left_blocked_scan():
    """Open everywhere except a near wall spanning front and left-front."""
    scan = np.full(N, 300.0)
    blocked = control._window_indices(N, -60, 30)  # offsets -60..+30 deg
    scan[blocked] = 40.0
    return scan


def test_gap_steer_aims_at_open_side():
    scan = _front_left_blocked_scan()
    ctrl = _controller()
    angle, bearing = ctrl._gap_steer(scan)
    assert bearing > 0  # widest gap is on the right
    assert angle > 0


def test_gap_mode_engages_when_boxed_in():
    scan = _front_left_blocked_scan()
    res = _controller().compute(scan)
    assert res.mode == "gap"
    assert res.blend > 0.9
    assert res.angle > 0


# --- adaptive speed and safety ----------------------------------------------


def test_speed_drops_with_forward_obstacle():
    open_scan = build_scan(corridor(100, 100))
    near = build_scan(corridor(100, 100))
    front = control._window_indices(N, -30, 30)
    near[front] = 60.0  # obstacle close ahead
    fast = _controller().compute(open_scan)
    slow = _controller().compute(near)
    assert slow.speed < fast.speed
    assert slow.speed > 0.0


def test_empty_scan_stops():
    res = _controller().compute(np.array([]))
    assert res.speed == 0.0 and res.angle == 0.0 and res.mode == "no_data"


def test_all_zero_scan_stops():
    res = _controller().compute(np.zeros(N))
    assert res.speed == 0.0 and res.mode == "no_data"


def test_speed_scales_with_speed_knob():
    scan = build_scan(corridor(100, 100))
    slow = control.WallFollowController(speed=0.3).compute(scan)
    fast = control.WallFollowController(speed=0.8).compute(scan)
    assert fast.speed > slow.speed


# --- turn indicator ---------------------------------------------------------


def test_turn_symbol_directions():
    assert control.turn_symbol(-0.5) == "<"  # left
    assert control.turn_symbol(0.5) == ">"  # right
    assert control.turn_symbol(0.0) == ""  # straight


def test_turn_symbol_deadband():
    assert control.turn_symbol(0.04, deadband=0.06) == ""
    assert control.turn_symbol(-0.04, deadband=0.06) == ""
    assert control.turn_symbol(0.06, deadband=0.06) == ">"
    assert control.turn_symbol(-0.06, deadband=0.06) == "<"


def test_turn_symbol_matches_steering_sign():
    # A car closer to the left wall steers right and should show '>'.
    scan = build_scan(corridor(100, 100), pose=(0, 30, 0))
    res = _controller().compute(scan)
    assert res.angle > 0
    assert control.turn_symbol(res.angle, control.Config().turn_deadband) == ">"


# --- configuration ----------------------------------------------------------


def test_config_from_nested_mapping_overrides_and_ignores_unknown():
    cfg = control.Config.from_mapping(
        {
            "centering_pd": {"kp": 0.01, "kd": 0.2},
            "adaptive_speed": {"speed_min_frac": 0.4},
            "bogus_section": {"not_a_field": 99},
        }
    )
    assert cfg.kp == 0.01
    assert cfg.kd == 0.2
    assert cfg.speed_min_frac == 0.4
    assert cfg.beam_theta_deg == control.Config().beam_theta_deg  # default kept
    assert not hasattr(cfg, "not_a_field")


def test_config_from_empty_mapping_is_defaults():
    assert control.Config.from_mapping({}) == control.Config()
    assert control.Config.from_mapping(None) == control.Config()


def test_load_config_yaml_round_trip(tmp_path):
    yaml = pytest.importorskip("yaml")
    path = tmp_path / "params.yaml"
    path.write_text(
        yaml.safe_dump({"speed": 0.65, "centering_pd": {"kp": 0.02}, "blend": {"blend_lo_cm": 70.0}})
    )
    cfg, speed = control.load_config(str(path))
    assert speed == 0.65
    assert cfg.kp == 0.02
    assert cfg.blend_lo_cm == 70.0


def test_config_kp_changes_steering():
    # Small offset (unsaturated), kd off, so steering is proportional to kp.
    scan = build_scan(corridor(100, 100), pose=(0, 6, 0))

    def angle_for(kp):
        cfg = control.Config(kp=kp, kd=0.0)
        return control.WallFollowController(speed=0.5, config=cfg).compute(scan).angle

    low, high = angle_for(0.005), angle_for(0.02)
    assert high > low > 0
    assert high == pytest.approx(low * 4, rel=0.05)  # 4x kp -> 4x angle


def test_config_speed_floor_changes_speed():
    boxed = build_scan(corridor(100, 100))
    front = control._window_indices(N, -30, 30)
    boxed[front] = 60.0  # frontal obstacle pins the clearance factor low

    def speed_for(floor):
        cfg = control.Config(speed_min_frac=floor)
        return control.WallFollowController(speed=0.5, config=cfg).compute(boxed).speed

    assert speed_for(0.6) > speed_for(0.25)


def test_config_steer_ema_damps_first_frame():
    scan = build_scan(corridor(100, 100), pose=(0, 20, 0))

    def first_angle(ema):
        cfg = control.Config(kd=0.0, steer_ema=ema)
        return control.WallFollowController(speed=0.5, config=cfg).compute(scan).angle

    assert abs(first_angle(0.9)) < abs(first_angle(0.0))  # heavier filter, smaller step


# --- robustness -------------------------------------------------------------


def test_first_frame_has_no_derivative_spike():
    # Same scan every frame means constant error; with the derivative skipped on
    # the first frame, the EMA should rise into the command, never overshoot it.
    scan = build_scan(corridor(100, 100), pose=(0, 25, 0))
    ctrl = _controller()
    a1 = ctrl.compute(scan).angle
    a2 = ctrl.compute(scan).angle
    a3 = ctrl.compute(scan).angle
    assert a1 <= a2 <= a3  # monotone settle, no entry spike
    assert abs(a1) < 1.0  # not slammed to the clamp on engage


def test_mirror_symmetry_negates_angle():
    scan = build_scan(corridor(100, 100), pose=(0, 22, 0))  # asymmetric scene
    a = _controller().compute(scan).angle
    a_mirror = _controller().compute(mirror_scan(scan)).angle
    assert a > 0
    assert a_mirror == pytest.approx(-a, abs=0.02)


def test_output_bounds_under_random_scans():
    rng = np.random.default_rng(0)
    ctrl = _controller()
    for _ in range(40):
        scan = rng.uniform(0.0, 600.0, N)
        scan[rng.random(N) < 0.2] = 0.0  # sprinkle no-data
        res = ctrl.compute(scan)
        assert -1.0 <= res.angle <= 1.0
        assert -1.0 <= res.speed <= 1.0


def test_blend_rises_monotonically_as_front_closes():
    blends = []
    for d in (400, 300, 200, 140, 90):
        walls = corridor(150, 150) + [((d, -150), (d, 150))]  # frontal wall at x=d
        res = _controller().compute(build_scan(walls))
        blends.append(res.blend)
    assert blends == sorted(blends)  # non-decreasing as the wall gets closer
    assert blends[0] < blends[-1]


def test_shipped_config_loads():
    """The committed config/wall_follower.yaml parses into a usable Config."""
    pytest.importorskip("yaml")
    path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "config", "wall_follower.yaml")
    cfg, speed = control.load_config(path)
    assert 0.0 <= speed <= 1.0
    assert cfg.kp > 0 and cfg.gap_arc_deg > 0
    # A controller built from the shipped config produces a sane command.
    res = control.WallFollowController(speed=speed, config=cfg).compute(build_scan(corridor(100, 100)))
    assert -1.0 <= res.angle <= 1.0 and res.speed > 0.0
