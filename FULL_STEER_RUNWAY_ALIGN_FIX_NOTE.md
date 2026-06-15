# Full-steer large turns + runway destination alignment fix

This revision is based on `A350AutoTaxi_GSX_UI_XTE_CORRIDOR_SWAY_FIX`.

## Changes

1. **Normal taxi speed changed to 25 kt**
   - `taxi_speed_kts=25.0`
   - Dynamic throttle gain/rate raised so the target can be reached more aggressively.

2. **Large-turn full-steer takeover earlier and immediate**
   - Added `tight_turn_force_full_steer=true`.
   - When an upcoming 65°+ corner is within `tight_turn_force_full_steer_distance_m`, the controller:
     - disables gradual steering build-up,
     - forces `steer` directly to `+/-1.00`,
     - sets the smoothed steering state immediately, and
     - holds full steer until runway/taxiway outbound heading rollout begins.
   - Route-track and XTE corridor logic are faded out during this hard-turn phase.

3. **Runway destination line-up**
   - If Destination is a runway, the controller now uses apt.dat runway endpoints to compute the selected runway heading and centreline.
   - Near runway destination it blends from taxi-route tracking into runway line-up:
     - heading alignment: aircraft heading captures selected runway heading;
     - centreline alignment: lateral runway centreline error commands an intercept angle;
     - arrival is not declared until heading and centreline tolerances are met.

## Main new ini keys

```ini
tight_turn_force_full_steer=true
tight_turn_force_full_steer_angle_deg=65.0
tight_turn_force_full_steer_distance_m=190.0
tight_turn_force_full_steer_ratio=1.00
tight_turn_force_full_release_heading_deg=24.0

runway_alignment_mode=true
runway_align_capture_distance_m=420.0
runway_align_full_distance_m=260.0
runway_align_center_gain_deg_per_m=0.85
runway_align_max_intercept_deg=34.0
runway_align_speed_kts=8.0
runway_align_center_tolerance_m=4.0
runway_align_heading_tolerance_deg=6.0
```

## Tuning

For even earlier full steer:

```ini
tight_turn_force_full_steer_distance_m=220.0
early_turn_takeover_distance_m=270.0
```

If it cuts inside too much:

```ini
tight_turn_force_full_steer_distance_m=160.0
tight_turn_force_full_release_heading_deg=30.0
```

If runway line-up is too aggressive:

```ini
runway_align_center_gain_deg_per_m=0.60
runway_align_max_intercept_deg=24.0
```
