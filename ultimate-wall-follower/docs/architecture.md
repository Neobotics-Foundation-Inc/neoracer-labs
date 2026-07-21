# Architecture

## Components

| Module | Responsibility |
|---|---|
| `wall_follower.py` | Entry point. Reads the LIDAR scan, enforces the deadman, calls the controller once per frame, publishes drive commands, prints throttled debug. Holds the `SPEED` knob. |
| `control.py` | All decision logic as pure functions and the `WallFollowController` class. No ROS or OpenCV imports, so it is unit-testable in isolation. |
| `config/wall_follower.yaml` | Tunable parameters (PD gains, lookahead, gap, blend, speed scaling, filter). Loaded into a `Config` on START; missing keys fall back to defaults. |
| `tests/` | pytest suite. Synthesizes scans by ray-casting against explicit wall geometry to verify sign conventions and behavior. |

## Data pipeline

```
                rc.lidar.get_samples()  (1080 ranges, cm, CW, 0 deg = front)
                          |
                          v
        +-------------------------------------+
        | WallFollowController.compute(scan)  |
        |                                     |
        |  forward clearance (narrow front)   |
        |            |                        |
        |   +--------+--------+               |
        |   v                 v               |
        | wall centering    follow-the-gap    |
        | (2 beams/wall,    (safety bubble +   |
        |  lookahead, PD)    widest free gap)  |
        |   \                 /               |
        |    \   blend by    /                |
        |     \  clearance  /                 |
        |      v           v                  |
        |        steering angle               |
        |            |                        |
        |     low-pass (EMA) filter           |
        |            |                        |
        |   adaptive speed (clearance x turn) |
        +-------------------------------------+
                          |
                          v
            rc.drive.set_speed_angle(speed, angle)
```

## Control stages

1. Forward clearance. Mean valid range in a narrow window straight ahead. A
   narrow window is used so the parallel side walls of a corridor are not read
   as a frontal obstacle. No return ahead is treated as open space.
2. Wall centering. For each wall, an abeam beam (90 / 270 deg) and a beam tilted
   `beam_theta_deg` toward the front give the wall angle `alpha` and the
   perpendicular distance `D` via `alpha = atan2(a*cos(theta) - b, a*sin(theta))`,
   `D = b*cos(alpha)`. The lookahead-projected distance is `D + L*sin(alpha)`,
   with `L = lookahead_base + lookahead_gain*SPEED`. The centering error is
   `D_right_proj - D_left_proj`; a PD on it produces the wall steering. Positive
   error (closer to the left wall, or nose yawed left) commands a right turn.
3. Follow-the-gap. Within a front arc, samples are free if open or beyond
   `gap_min_dist_cm`. A bubble of `gap_bubble_deg` around the nearest return is
   masked. Steering aims at the center of the longest free run; bearing right of
   center commands a right turn.
4. Blend. Weight `w` ramps 0 to 1 as forward clearance drops from `hi` to `lo`
   (both scale with `SPEED`). `angle = (1-w)*wall + w*gap`. A missing wall forces
   `w = 1`. The result passes through an EMA low-pass to absorb LIDAR jitter at
   60 Hz.
5. Adaptive speed. `speed = SPEED * clearance_factor * steering_factor`, where
   `clearance_factor` falls with forward clearance toward `speed_min_frac` and
   `steering_factor` falls with `|angle|` toward `turn_min_frac`. The car brakes
   into corners and runs full out on straights.

## State

`WallFollowController` keeps only the previous centering error (PD derivative)
and the previous steering command (EMA). No pose, no map. `reset()` clears both;
`wall_follower.py` resets whenever the deadman is released.

## Configuration

`config/wall_follower.yaml` holds every tunable value plus the top-level
`speed`. `control.load_config(path)` parses it (PyYAML, imported lazily) and
`Config.from_mapping` flattens the nested sections, sets fields by name, and
ignores unknown keys; absent keys keep their defaults. `wall_follower.py`
resolves the path against its own location and reloads on START, so editing the
YAML and re-pressing START applies new values without restarting the process. If
the file is missing or PyYAML is absent, the follower logs a warning and runs on
`Config()` defaults.

## Sign conventions

- Scan index `i` -> clockwise angle `i*360/n` deg, 0 = front. Right side ~ 90,
  rear ~ 180, left ~ 270.
- Drive `angle`: positive = right (the backend negates internally).
- Verified end to end by the ray-cast tests in `tests/test_control.py`.
