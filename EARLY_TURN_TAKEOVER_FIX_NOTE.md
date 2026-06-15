# Early Turn Takeover Fix

This build fixes the case where the aircraft waits too long before applying full tiller near a large taxiway turn.

## What changed

1. Added `early_turn_takeover` mode.
   - Arms the tight/one-pass turn controller before the normal tight-turn window.
   - Fades out the `track_course_control` straight-leg logic earlier so it no longer holds the aircraft straight into the intersection.
   - Blends the heading target toward the outbound taxiway leg before the nose reaches the taxi node.

2. Added an early snap floor.
   - If a 70-100 degree corner is approaching and target steering is still too small, the code now forces a turn-side target steer before the normal snap code runs.
   - The snap floor grows with `earlyTurnBlend`, so it is gentle at the start of the window and becomes aggressive close to the corner.

3. Retuned `A350AutoTaxi.ini`.
   - `tight_turn_trigger_distance_m` increased from 115 m to 185 m.
   - `track_course_tight_turn_fade` increased to 1.00.
   - `early_turn_takeover_distance_m=185` and `early_turn_full_distance_m=115`.
   - `early_turn_snap_min_target_ratio=0.82`, `early_turn_snap_to_ratio=0.98`.
   - Tight turn full-deflection mapping tightened to 14 degrees for quicker full steer.

## Tuning hints

If full-steer still starts late:

```ini
early_turn_takeover_distance_m=210.0
early_turn_full_distance_m=135.0
early_turn_snap_min_target_ratio=0.90
```

If the aircraft cuts the inside of the corner too much:

```ini
early_turn_takeover_distance_m=165.0
early_turn_full_distance_m=95.0
early_turn_snap_min_target_ratio=0.70
```

If route-track still appears to limit the turn:

```ini
track_course_tight_turn_fade=1.00
early_turn_track_course_fade=1.00
early_turn_heading_blend=1.00
```
