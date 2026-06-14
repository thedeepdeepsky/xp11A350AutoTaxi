# FAST_STEER_SLATS_FIX

This patch targets two user-observed issues:

1. **Steer builds too slowly in tight turns**
   - Added `steer_fast_response` rate boost.
   - Added `tight_turn_snap_steer` so the tiller command can jump part-way into a turn instead of spending a full second ramping from zero.
   - Tight turn defaults changed to smaller radius: `tight_turn_speed_kts=2.7`, `tight_turn_steer_full_deflection_deg=24.0`, `tight_turn_steer_smoothing_per_sec=8.50`, differential brake max `0.16`.

2. **Optional ECAM F/CTL SLATS/FLAPS assist**
   - Added `fctl_secondary_assist` and default datarefs:
     - `sim/cockpit2/controls/flap_ratio`
     - `sim/flightmodel/controls/flaprqst`
     - `sim/flightmodel/controls/slatrat`
   - These default datarefs may be ignored by custom A350 aircraft; if so, map them with DataRefTool.
   - This does not replace nosewheel steering; it only requests secondary F/CTL surface movement during tight turns.

New important INI keys:

```ini
steer_fast_response=true
steer_fast_response_rate_per_sec=9.50
tight_turn_snap_steer=true
tight_turn_snap_to_ratio=0.62
fctl_secondary_assist=true
fctl_secondary_flap_ratio=0.16
```
