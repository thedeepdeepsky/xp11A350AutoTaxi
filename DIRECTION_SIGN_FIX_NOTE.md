# Direction Sign Fix

This build fixes the turn-direction regression introduced in the full-steer / runway-align build.

Changes:

1. Runway centreline alignment sign corrected.
   - Positive XTE means aircraft is left of the selected runway centreline.
   - The plugin convention is positive heading/steer = turn right.
   - Therefore positive runway XTE must add a positive centreline intercept, not a negative one.

2. Hard/full-turn steering no longer guesses direction from a separate turn-sign path.
   - Forced full steer now uses the same controller sign already used by PID/targetSteer.
   - Fallback order: targetSteer -> pidOutputDeg -> headingErr -> lookahead/course -> outbound heading.
   - This prevents route-track, runway-align, and tight-lookahead from commanding opposite signs.

3. Differential brake assist side is now tied to the actual steering command sign.
   - During hard/full turns, the brake assist will not fight the forced steering command.

If the aircraft still turns opposite in every situation, the aircraft-specific steering dataref itself is inverted and should be changed in A350AutoTaxi.ini or handled with an aircraft-specific dataref mapping.
