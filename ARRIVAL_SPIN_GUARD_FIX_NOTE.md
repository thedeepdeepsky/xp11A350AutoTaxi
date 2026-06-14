# Arrival Spin Guard Fix

This build addresses the circling/spinning issue that can appear at runway hold-short or destination nodes such as DE18R when apt.dat contains a tiny final hook or a very sharp final node pair.

Changes:

1. Terminal route pruning during planning
   - If the final leg is extremely short, or a sharp final hook is detected, the last micro-node is removed from the route.
   - The aircraft stops at the previous practical node instead of trying to capture an impossible tiny turn.

2. Terminal arrival bubble
   - Once inside `terminal_arrival_radius_m`, AutoTaxi stops and holds.
   - If the aircraft passes the final node but is still close, the overshoot guard stops it instead of letting it circle.

3. Final-area tight-turn suppression
   - In the final terminal area, one-pass snap/tight-turn logic is disabled.
   - This prevents the aggressive 90-degree turn code from commanding a full-circle near the last node.

4. Final-area speed reduction
   - Near destination, target speed fades toward `terminal_speed_kts`.

New ini parameters:

```ini
terminal_arrival_guard=true
terminal_arrival_radius_m=34.0
terminal_overshoot_radius_m=55.0
terminal_overshoot_seconds=0.8
terminal_no_tight_turn_distance_m=85.0
terminal_slowdown_distance_m=130.0
terminal_speed_kts=2.0
terminal_min_final_leg_m=28.0
terminal_sharp_turn_deg=95.0
terminal_sharp_turn_short_leg_m=55.0
terminal_xte_stop_m=16.0
```

If it still circles at a particular airport, increase:

```ini
terminal_arrival_radius_m=42.0
terminal_no_tight_turn_distance_m=110.0
terminal_min_final_leg_m=35.0
```

If it stops too early, reduce:

```ini
terminal_arrival_radius_m=24.0
terminal_min_final_leg_m=18.0
```
