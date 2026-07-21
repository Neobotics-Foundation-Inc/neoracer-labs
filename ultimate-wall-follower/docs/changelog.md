# Changelog

All notable changes to this project. Format: Keep a Changelog
(keepachangelog.com). Versioning: Semantic Versioning (semver.org).

## [Unreleased]

## [0.2.0] - 2026-06-06

### Added
- Validation scripts in `scripts/`, sharing `lidar_viz.py`: `capture_lidar.py`
  saves one annotated frame to `data/YYYYMMDD_HHMMSS.png`; `live_display.py`
  streams the same view (for `ssh -Y` hosts), with `--demo` and `--selftest`
  modes that need no ROS. Both overlay the wall beams, forward probe, lookahead
  radius, and commanded path, and stamp the time and PD coefficients.
- `data/` for captured frames (PNGs gitignored).
- Tests for the shared visualization (synthetic scan, render sign, empty scan,
  path curvature).

## [0.1.1] - 2026-06-06

### Added
- Tests: config values change behavior (kp, speed floor, EMA damping), first
  frame has no derivative spike, left-right mirror negates steering, output
  stays bounded under random scans, blend rises as the front wall closes.

### Fixed
- Skip the PD derivative on the first frame after a reset (or after re-acquiring
  the walls). Previously prev_error jumped 0 -> error over a tiny dt, spiking the
  steering toward the clamp on every deadman re-engage.

## [0.1.0] - 2026-06-06

### Added
- Adaptive hybrid wall follower (`control.py`, `wall_follower.py`): lookahead
  two-beam wall centering blended with follow-the-gap by forward clearance,
  with an EMA low-pass on the steering output.
- Adaptive speed scaled by forward clearance and steering magnitude; lookahead
  and brake thresholds scale with the single `speed` knob.
- YAML parameter file `config/wall_follower.yaml`, loaded via
  `control.load_config` and reloaded on START.
- Dot-matrix turn indicator: `<` when steering left, `>` when steering right,
  blank within a steering deadband; published on change via `show_text`.
- Deadman entry point: drives only while the right bumper is held.
- pytest suite over synthetic ray-cast scans covering geometry signs, centering,
  follow-the-gap, adaptive speed, safety stops, the turn symbol, and config
  loading.
- Scaffold: `README.md`, `docs/architecture.md`, `docs/test-log.md`,
  `.gitignore`, `.gitattributes`.
