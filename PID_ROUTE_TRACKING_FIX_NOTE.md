# PID route tracking fix

This build changes AutoTaxi steering from node-chasing to AP-like route capture.

Main changes:

1. The aircraft no longer steers directly toward the nearest/next taxi node. It projects the aircraft onto the planned taxi polyline and follows a forward look-ahead point on that same route. This prevents +/- steering oscillation when the aircraft starts near a node.
2. Route segment selection is monotonic. The controller can advance along the route, but it will not jump backward to a previous node just because it is geometrically close.
3. A PID heading/path controller was added. The PID output is converted to a capped/smoothed nosewheel steering command.
4. Differential braking is raised to 0.10 by default but only engages while rolling and only for large PID corrections.
5. UI now displays XTE (cross-track error) and PID output angle.

New INI tuning keys:

- pid_route_tracking=true
- pid_heading_kp=0.95
- pid_heading_ki=0.010
- pid_heading_kd=0.12
- pid_integral_limit_deg_sec=120.0
- max_track_capture_angle_deg=42.0
- route_segment_advance_along=0.94
- route_segment_scan_ahead=4
- lookahead_min_distance_m=45.0
- lookahead_max_distance_m=120.0
- lookahead_speed_gain_m_per_kt=3.0
- differential_brake_max=0.10

If steering is still too nervous, reduce `pid_heading_kp` to 0.75 or raise `lookahead_distance_m` to 95.
If it returns to the route too slowly, raise `pid_heading_kp` to 1.10 or lower `lookahead_distance_m` to 65.
