# Early turn / anti-oscillation route capture fix

This build changes the taxi controller from a simple route-preview PID into an earlier-turning, damped path-capture controller.

## What changed

1. **Earlier corner anticipation**
   - When a turn is within `turn_anticipation_distance_m`, the controller extends its lookahead into the next leg.
   - This starts steering before the aircraft nose reaches the taxi node, reducing the late S-turn after intersections.

2. **Less back-and-forth oscillation**
   - Lower P gain, no integral by default, stronger filtered D damping.
   - Added a small cross-track bias so the aircraft smoothly converges back to the original planned route instead of chasing the next node.

3. **Earlier slowdown before turns**
   - When approaching a strong corner, target speed blends down before reaching the node.

4. **Steering cap can still reach 1.00**
   - `max_steer_ratio=1.00`
   - `low_speed_max_steer_ratio=1.00`
   - high-speed steering remains scheduled to avoid large oscillation at taxi speed.

5. **Differential braking kept stronger**
   - `differential_brake_max=0.10`
   - threshold reduced to `28 deg` so it assists large low-speed corrections earlier.

## Main tuning parameters

```ini
lookahead_distance_m=115.0
lookahead_min_distance_m=70.0
lookahead_max_distance_m=190.0
lookahead_speed_gain_m_per_kt=5.0

pid_heading_kp=0.62
pid_heading_ki=0.000
pid_heading_kd=0.26
pid_derivative_filter_sec=0.55
pid_cross_track_gain_deg_per_m=0.06
pid_cross_track_max_deg=8.0
max_track_capture_angle_deg=65.0

turn_anticipation=true
turn_anticipation_distance_m=150.0
turn_anticipation_min_angle_deg=15.0
turn_anticipation_extra_lookahead_m=85.0
turn_anticipation_slowdown_kts=4.5
route_segment_advance_along=0.82

max_steer_ratio=1.00
low_speed_max_steer_ratio=1.00
high_speed_max_steer_ratio=0.55
differential_brake_threshold_deg=28.0
differential_brake_max=0.10
```

## If it still swings

Reduce aggressiveness first:

```ini
pid_heading_kp=0.50
pid_cross_track_gain_deg_per_m=0.04
lookahead_distance_m=130.0
```

## If it turns too late

Increase anticipation:

```ini
turn_anticipation_distance_m=180.0
turn_anticipation_extra_lookahead_m=110.0
route_segment_advance_along=0.78
```
