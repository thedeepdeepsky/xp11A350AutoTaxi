# Dynamic throttle + destination dropdown fix

Changes in this build:

1. Added a dynamic taxi-speed governor. Throttle is no longer a mostly fixed step; it is continuously scheduled from target speed, ground speed, capture severity, tight-turn blend and terminal slowdown. UI now shows target speed and throttle mode.
2. Added throttle rate limiting and separate increase/decrease rates, so thrust can rise when speed decays and drop quickly during overspeed or final arrival.
3. Destination selector hit zone was enlarged. The selector now has an invisible wider click area and `<` / `>` buttons, so destination can still be changed even if the combo list is not opening on a particular X-Plane 11/HiDPI setup.
4. The dropdown list is processed before the bottom route buttons and shows a visible "No destinations in this filter" row if a filter has no entries.
5. Mouse wheel over the destination selector opens/scrolls the destination list.
