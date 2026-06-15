# Full-steer 50 m gate / standards-based taxi turn fix

## Why this change
The previous build forced full tiller about 190 m before a large turn. That made the aircraft cut into the turn too early and caused sign/route-track conflicts. Airport taxiway design material is based on aircraft geometry, cockpit-over-centreline turning, fillet radius and track-in, not a long full-tiller command hundreds of metres before the node.

## Behaviour now
- Route and lookahead anticipation can still start early.
- Before the near-apex window, large-turn steering is capped by `tight_turn_pre_full_steer_cap_ratio`.
- Full steer is now only allowed at about 55 m before the upcoming turn node.
- At the 55 m gate, `forceFullTurnActive` may still set steer immediately to ±1.00.

## Main ini changes
```ini
# Full tiller near corner, not hundreds of metres early
tight_turn_force_full_steer_distance_m=55.0
tight_turn_pre_full_steer_cap_ratio=0.45
tight_turn_pre_full_snap_min_ratio=0.12

# Early anticipation is now moderate only
early_turn_takeover_distance_m=135.0
early_turn_full_distance_m=55.0
early_turn_min_blend=0.18
early_turn_heading_blend=0.60
early_turn_track_course_fade=0.55
early_turn_snap_to_ratio=0.45
early_turn_snap_min_target_ratio=0.12

turn_anticipation_distance_m=135.0
tight_turn_trigger_distance_m=115.0
tight_turn_snap_to_ratio=0.45
tight_turn_snap_min_target_ratio=0.12
```

## Source changes
- Added `tightTurnPreFullSteerCapRatio` and `tightTurnPreFullSnapMinRatio` config fields.
- Added a `largeTurnPreFullWindow` cap so route-track/tight-turn logic cannot command full steer before the near-apex gate.
- Modified early-turn and tight-turn snap logic so hard one-pass snap becomes full only inside `tightTurnForceFullSteerDistanceM`.
