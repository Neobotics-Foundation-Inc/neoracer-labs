# Test log

## Unit tests (offline)

Run:

```
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/ -q
```

### 2026-06-06 - initial suite

- Setup: numpy 1.26.2, pytest 6.2.5, Python 3.10.12. Scans synthesized by
  ray-casting against explicit wall geometry (corridors, yawed poses, blocked
  fronts) at the real 1080-sample resolution.
- Coverage: clamp/remap, signed-offset mapping, no-data handling in
  average/closest helpers, wall geometry (parallel, yawed sign, missing wall),
  centering sign (centered, offset left/right, yaw correction), follow-the-gap
  (open-side bearing, gap mode engagement), adaptive speed (drops with frontal
  obstacle, scales with SPEED), and safety stops (empty and all-zero scans).
- Result: 22 passed (includes config loading: nested-mapping override,
  unknown-key tolerance, empty/None defaults, YAML round-trip, and the shipped
  config/wall_follower.yaml parsing into a working controller).
- Notes: forward clearance switched from nearest-in-cone to a narrow forward
  mean after the cone caught corridor side walls and falsely tripped gap mode on
  a straight. Closest-point angle tolerance set to 3 deg for 1080-sample
  quantization.

## On-robot tests

Pending. Procedure for the first powered runs:

1. Bench, wheels off ground. Confirm `set_speed_angle` sign by hand-rotating the
   car and watching steering. Confirm the deadman gates driving.
2. Low SPEED (~0.3) in a known corridor. Verify centering and that corners slow
   the car and trip gap mode. Log observations below.
3. Raise SPEED in small steps, one run per step. Record top stable SPEED and any
   failure mode (oscillation, corner clipping, gap thrash).

| Date | SPEED | Environment | Observation | Outcome |
|---|---|---|---|---|
| | | | | |
