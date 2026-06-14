# A350 AutoTaxi Control Fix

This version fixes the case where UI shows ACTIVE but the aircraft does not roll.

Key changes:
- Breakaway throttle is no longer capped by old `max_throttle=0.11` ini files.
- Default max throttle/creep/breakaway thrust raised for heavy A350 taxi start.
- Differential braking is disabled at near-zero speed and during stuck breakaway.
- Parking brake is released every active control frame, using both cockpit2 and legacy datarefs when available.
- Writes both `throttle_ratio_all` and per-engine `throttle_ratio[]`.
- Writes both `flightmodel/controls/*_brake_add` and `cockpit2/controls/*_brake_ratio` when available.
- Adds slow `sim/engines/throttle_up` command assist when complex aircraft overwrite throttle datarefs.

Important installation note: replace the old `A350AutoTaxi.ini` in the X-Plane plugin folder, not only `win.xpl`.
