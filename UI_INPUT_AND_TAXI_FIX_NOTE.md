# UI input / opacity / no-movement fix

This build fixes three field-test issues reported from X-Plane 11.55:

1. Buttons were visually visible but sometimes did not react.
   - Mouse hit testing is now tolerant to both global boxel coordinates and local window coordinates.
   - Hit boxes are padded by 6 px for high-DPI / scaled displays.

2. Panel was too transparent.
   - Main panel alpha increased from 0.90 to 0.97.
   - Dropdown/buttons are more opaque.
   - Bottom buttons are larger and easier to click.

3. Aircraft could arm AutoTaxi but stay still.
   - Default max_throttle raised from 0.11 to 0.24.
   - Added creep_throttle and breakaway_throttle.
   - If the aircraft is active but almost stationary for stuck_boost_seconds, throttle is temporarily boosted.
   - Optional command_assist issues the X-Plane brake-release command when available.
   - UI now displays last steering/throttle/brake commands and stuck timer while active.

If the route plans and the status becomes ACTIVE but the aircraft still does not move, use DataRefTool to verify the aircraft-specific writable datarefs in A350AutoTaxi.ini. Some FlightFactor/custom aircraft may intercept the default throttle/brake/tiller datarefs.
