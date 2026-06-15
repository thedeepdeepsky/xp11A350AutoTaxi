# XTE Corridor / Anti-Sway Fix

This build changes the straight-line/path tracking logic to reduce sway during yaw/XTE correction.

## Problem addressed

The previous controller could still command a large yaw/steer correction when XTE was only around 3-5 m, then keep that command until after crossing the taxiway centerline. The aircraft would then steer back the other way, creating repeated S-turns.

## New behavior

1. First capture into a configurable XTE corridor, default +/-5 m.
2. Once inside that corridor, aggressive fast-capture gains are faded out.
3. Inside the corridor, effective XTE is softened and PID Kp is reduced.
4. KD / damping is increased so the aircraft rolls out earlier.
5. If the nose is already moving toward the centerline, the controller starts pre-zero rollout before XTE crosses 0.
6. In fine-track mode the steer command and steer rate are damped, unless tight-turn / early-turn mode is active.

## New INI parameters

```ini
xte_corridor_control=true
xte_corridor_m=5.0
xte_fine_tune_m=1.2
xte_corridor_outer_gain=0.55
xte_corridor_inner_gain=0.42
xte_corridor_max_intercept_deg=13.0
xte_corridor_outer_max_intercept_deg=22.0
xte_corridor_lead_sec=4.0
xte_corridor_prezero_return_deg=7.0
xte_corridor_kp_scale=0.52
xte_corridor_kd_boost=1.45
xte_corridor_capture_fade=0.18
sway_damping_band_m=6.0
sway_damping_command_scale=0.55
sway_damping_steer_rate_per_sec=2.0
```

## Tuning hints

If XTE still grows beyond 5 m before capture:

```ini
xte_corridor_outer_gain=0.65
xte_corridor_outer_max_intercept_deg=26.0
track_course_gain=1.30
```

If sway is still too strong inside +/-5 m:

```ini
xte_corridor_kp_scale=0.42
sway_damping_command_scale=0.45
sway_damping_steer_rate_per_sec=1.6
```

If it becomes too lazy near the centerline:

```ini
xte_corridor_kp_scale=0.65
xte_corridor_inner_gain=0.55
sway_damping_command_scale=0.70
```
