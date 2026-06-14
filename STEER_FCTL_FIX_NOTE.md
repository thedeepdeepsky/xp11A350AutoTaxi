# Steer + F/CTL coupling fix

This revision addresses two taxi-control issues observed in the A350:

1. Steering command was too aggressive and frequently saturated at `-1.00` / `+1.00`.
2. During taxi turns, the ECAM F/CTL page did not show enough rudder/roll surface response.

## Changes

- Added speed-scheduled steering caps:
  - `max_steer_ratio=0.38`
  - `low_speed_max_steer_ratio=0.42`
  - `high_speed_max_steer_ratio=0.20`
- Increased `steer_full_deflection_deg` to `85.0` to avoid saturation.
- Reduced steering rate via `steer_smoothing_per_sec=1.2`.
- Reduced differential braking; it was too strong for taxi turns.
- Added optional F/CTL coupling:
  - `fctl_turn_coupling=true`
  - `fctl_rudder_dataref=sim/cockpit2/controls/yoke_heading_ratio`
  - `fctl_roll_dataref=sim/cockpit2/controls/yoke_roll_ratio`
  - legacy joystick equivalents are also written when writable.

The F/CTL coupling is deliberately small. It should make rudder/roll indications move during turns without using full flight-control deflection.

If your specific A350 uses custom FlightFactor datarefs for its ECAM F/CTL page, use DataRefTool and replace the four `fctl_*_dataref` values in `A350AutoTaxi.ini`.
