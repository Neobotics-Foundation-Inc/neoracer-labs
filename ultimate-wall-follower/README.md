# Ultimate Wall Follower

Adaptive high-speed wall following for the RACECAR Neo. Steers from the live 2D
LIDAR scan only: no SLAM, no map, no per-track tuning. One knob (`SPEED`) sets
the pace; lookahead, braking, and cornering adapt to the scan.

## Table of contents

- [Quick start](#quick-start)
- [Controls](#controls)
- [Parameters](#parameters)
- [Layout](#layout)
- [Behavior](#behavior)
- [Validation tools](#validation-tools)
- [Tests](#tests)
- [Driver contract](#driver-contract)
- [Changelog](#changelog)

## Quick start

```
cd ~/jupyter_ws/neobotics/labs/ultimate_wall_follower
python3 wall_follower.py
```

`wall_follower.py` is the entry point; running it starts everything. Press the
START button to engage. Hold the RIGHT BUMPER (deadman) while it drives; release
it or press BACK to stop.

## Controls

| Input | Effect |
|---|---|
| START | Engage the follower |
| RIGHT BUMPER (hold) | Deadman; the car drives only while held |
| BACK | Stop |

## Parameters

All parameters live in [config/wall_follower.yaml](config/wall_follower.yaml).
Edit values there and press START to reload; no restart is needed. `speed` is
the primary knob:

```yaml
speed: 0.5   # forward pace in [0, 1]; raise to go faster
```

The other groups (PD gains, lookahead, follow-the-gap, blend, adaptive speed,
output filter) have sane defaults; a missing key falls back to the default in
[control.py](control.py) `Config`. Lookahead distance and the braking thresholds
scale with `speed`, so raising the knob automatically looks farther ahead and
brakes earlier.

## Layout

```
ultimate_wall_follower/
  wall_follower.py          entry point: I/O, deadman, config reload, rc.go()
  control.py                pure control + geometry math (no ROS/OpenCV imports)
  config/wall_follower.yaml  tunable parameters; reloads on START
  scripts/                  validation tools (capture_lidar.py, live_display.py)
  data/                     captured frames (timestamped PNGs, gitignored)
  tests/                    pytest suite over synthetic ray-cast scans
  docs/                     architecture.md, changelog.md, test-log.md
  README.md
```

The split keeps all decision logic in `control.py` as plain numpy functions, so
it is fully unit-testable without the robot.

## Behavior

Two behaviors share each LIDAR scan and are blended by forward clearance:

- Wall centering. Two beams per wall recover the wall angle and perpendicular
  distance; the distance is projected to a lookahead point ahead of the car, so
  a corner registers before the car reaches it. A PD acts on the projected
  left-minus-right difference.
- Follow-the-gap. A safety bubble masks the nearest obstacle, then steering aims
  at the center of the widest free angular gap ahead.

Blend weight rises as forward clearance falls: open walls -> centering; tight or
broken geometry (corners, gaps, a missing wall) -> gap following. Speed scales
down with low forward clearance and with steering magnitude, so the car brakes
into corners and runs full out on straights. See [docs/architecture.md](docs/architecture.md).

The dot-matrix panel shows the current turn direction: `<` when steering left,
`>` when steering right, blank when within a small deadband of straight. The
panel is text-only on the current backend, so this uses `show_text`; the symbol
is published only when it changes, not every frame.

## Validation tools

Two scripts in `scripts/` confirm the LIDAR, the geometry, and the controller
agree before you let the car drive. Both render in the car frame (forward up,
right +x) with the wall beams, forward probe, lookahead radius, commanded path,
and a corner box holding the timestamp and PD coefficients.

- [scripts/capture_lidar.py](scripts/capture_lidar.py): snapshot one frame to
  `data/YYYYMMDD_HHMMSS.png`.
- [scripts/live_display.py](scripts/live_display.py): stream the same frame to a
  window, intended for a host connected with `ssh -Y` (X11 forwarding).

```
python3 scripts/capture_lidar.py            # live scan from the robot
python3 scripts/capture_lidar.py --demo     # synthetic scan, no ROS needed
python3 scripts/live_display.py             # stream (needs a display; use ssh -Y)
python3 scripts/live_display.py --demo      # synthetic moving scan, no ROS
python3 scripts/live_display.py --selftest  # headless render check
```

The live scan path spins the racecar with `go_async` and never commands motion
(it stops the drive on entry), so it is safe to run on a powered car.

## Tests

```
cd ~/jupyter_ws/neobotics/labs/ultimate_wall_follower
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/ -q
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` avoids a third-party plugin incompatible with
the system pytest 6.2.5. Tests synthesize scans by ray-casting against known
wall geometry, so the LIDAR-to-steering sign conventions are verified, not
assumed.

## Driver contract

The follower depends on the RACECAR Neo backend (built separately):

- `/scan`: distances in cm, 1080 samples, 0 deg straight ahead increasing
  clockwise, `0.0` = no return.
- `/drive`: `set_speed_angle(speed, angle)` in `[-1, 1]`; positive angle = right.
- `/led_matrix/command`: `show_text(str)` for the dot-matrix panel (text-only;
  `set_matrix` is a placeholder on this backend).

If the driver changes sample count, units, sign, or the no-data sentinel, update
the feature extraction in `control.py`. Note: on the current local library
build, `set_max_speed` is a no-op (fixed upstream), so top speed is capped at the
driver default until the library is updated; `speed` still scales pace below that
cap.

## Changelog

Latest entries; full history in [docs/changelog.md](docs/changelog.md).

### [0.2.0] - 2026-06-06

- Added validation scripts: `capture_lidar.py` (save an annotated frame) and
  `live_display.py` (stream it over `ssh -Y`), both overlaying the wall beams,
  forward probe, lookahead, commanded path, timestamp, and PD coefficients.

### [0.1.1] - 2026-06-06

- Fixed a first-frame steering spike on deadman re-engage (derivative skipped on
  the first frame after a reset).
- Added regression tests for config application, mirror symmetry, output bounds,
  and blend behavior.

### [0.1.0] - 2026-06-06

- Adaptive hybrid follower: lookahead wall centering blended with follow-the-gap,
  adaptive speed, deadman entry point.
- YAML parameter file in `config/`, reloaded on START.
- Dot-matrix turn indicator (`<` left, `>` right).
- Unit test suite over synthetic ray-cast scans.
