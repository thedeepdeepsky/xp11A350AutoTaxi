# One-pass 90-degree tight turn fix

This build changes the previous tight-turn snap from 0.62 to an aggressive one-pass mode.

Current fast snap behavior:
- Normal tight turn snap target: `tight_turn_snap_to_ratio=0.88`
- Hard 70-100 degree corner snap floor: `tight_turn_hard_snap_min_target_ratio=0.70`
- Hard 70-100 degree corner snap cap: `tight_turn_hard_snap_to_ratio=0.88`

So in a 90-degree turn, when the target steer has a clear sign, the command will jump immediately to at least about 0.70 and up to 0.88, then continue toward 1.00 via the fast rate limiter.

One-pass anti-overshoot logic was added in AutoTaxi.cpp: when the aircraft heading is within `tight_turn_rollout_heading_deg` of the outbound taxiway leg, the PID output is capped so the tiller starts unwinding before the nose overshoots the outbound heading.

The route segment advance lower clamp was relaxed from 0.70 to 0.55, so `route_segment_advance_along=0.68` is now honored and the controller can commit to the outbound leg earlier.
