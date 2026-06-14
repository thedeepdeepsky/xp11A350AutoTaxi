# Steer 1.00 + Fast Route Capture Fix

This build keeps low-speed steering authority at 1.00 and adds an AP-style fast route-capture mode.

## Why steer did not reach 1.00 before

The old controller capped PID output at about 65 deg and then divided it by `steer_full_deflection_deg=85.0`, with a steering exponent above 1.0. That meant even with `max_steer_ratio=1.00`, the actual command often stayed below full steering.

## What changed

- `max_track_capture_angle_deg=88.0`
- `max_steer_ratio=1.00`
- `low_speed_max_steer_ratio=1.00`
- fast-capture mode uses `fast_capture_steer_full_deflection_deg=45.0`, so large off-track/initial errors can drive steering up to 1.00.
- off-track recovery uses stronger cross-track bias only while far from the planned route.
- when the aircraft crosses back over the route centerline, the controller clears derivative/integral memory and fades out the aggressive capture mode to reduce S-turn oscillation.

## Tuning

If recovery is still too slow, raise:

```ini
fast_capture_gain_deg_per_m=0.70
fast_capture_max_bias_deg=50.0
fast_capture_steer_smoothing_per_sec=3.00
```

If it snaps back too hard or starts oscillating again, lower:

```ini
fast_capture_gain_deg_per_m=0.45
fast_capture_kp_boost=1.15
fast_capture_steer_smoothing_per_sec=2.00
```
