# Track Course Pre-Zero Steering Fix

This build changes the route-following logic for small and medium cross-track errors.

## Problem fixed
The previous controller mostly followed a far look-ahead point. On straight taxiway tracking this could produce only a very small heading correction until XTE had already grown to several metres. During a correction, it also tended to keep the same steering direction until XTE crossed 0, then returned the tiller late, causing S-turns.

## New logic
Added a direct track-course controller:

- follow current taxiway segment course, not only the distant look-ahead target
- add Stanley/L1-style XTE intercept angle immediately when XTE appears
- inside the pre-zero band, soften/reverse effective XTE before displayed XTE crosses 0
- blend this with existing look-ahead/tight-turn logic so 90-degree turns still use the corner code

## New INI parameters

```ini
track_course_control=true
track_course_blend=0.92
track_course_lookahead_m=16.0
track_course_speed_gain_m_per_kt=0.55
track_course_gain=1.55
track_course_max_intercept_deg=38.0
track_course_prezero_band_m=11.0
track_course_prezero_lead_sec=5.6
track_course_prezero_power=2.30
track_course_prezero_min_heading_deg=0.10
track_course_prezero_counter_m=7.0
track_course_tight_turn_fade=0.85
```

## Tuning
If it still waits too long before steering back:

```ini
track_course_gain=1.8
track_course_prezero_counter_m=9.0
track_course_prezero_band_m=13.0
```

If it begins to snake:

```ini
track_course_gain=1.25
track_course_prezero_counter_m=5.0
track_course_blend=0.75
```
