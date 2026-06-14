# Tight Turn Radius Fix

This build reduces the excessive/wide turn radius seen during A350 taxi turns.

Changes:

- Added `tight_turn_mode` in `A350AutoTaxi.ini`.
- In sharp upcoming turns, the controller now aims just after the corner apex instead of far down the next taxiway leg.
- Reduced default dynamic lookahead from the previous very long values.
- Reduced taxi speed and sharp-turn speed during corners.
- Allows full steer authority during tight turns while keeping PID damping.
- Blends stronger differential braking in tight turns.

Key tuning knobs:

```ini
tight_turn_lookahead_after_apex_m=42.0
tight_turn_speed_kts=3.2
tight_turn_steer_full_deflection_deg=32.0
tight_turn_steer_smoothing_per_sec=3.20
tight_turn_differential_brake_max=0.12
```

For an even smaller radius, reduce `tight_turn_lookahead_after_apex_m` to 30-35 and `tight_turn_speed_kts` to 2.8.
If the aircraft cuts too hard or snakes, raise `tight_turn_lookahead_after_apex_m` to 50-60 and lower `tight_turn_kp_boost`.
