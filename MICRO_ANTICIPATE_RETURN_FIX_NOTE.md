# Micro anticipate return-steer fix

This version adds anticipatory return steering for small taxiway corrections.

Problem fixed:
- During fine centerline corrections the measured XTE could change only very slowly.
- The controller waited for XTE to cross zero before unwinding/reversing steer.
- Result: small S-turns even on minor route corrections.

New behavior:
- Computes a geometry-derived XTE rate from aircraft heading relative to the planned taxi leg.
- Predicts future XTE even when the measured XTE delta is tiny.
- Starts unwinding/counter-steering before the displayed XTE reaches zero.
- Uses a smaller steering deadband while micro anticipate is active.

New INI keys:
```ini
micro_anticipate_return=true
micro_anticipate_band_m=8.0
micro_anticipate_lead_sec=4.2
micro_anticipate_geom_rate_weight=0.90
micro_anticipate_min_speed_kts=1.0
micro_anticipate_min_heading_deg=0.5
micro_anticipate_max_lead_m=10.0
micro_anticipate_bias_boost=1.35
micro_anticipate_deadband_deg=0.20
```

Tuning:
- Earlier return steer: increase `micro_anticipate_lead_sec` to 4.8 or 5.2.
- Stronger small correction: increase `micro_anticipate_bias_boost` to 1.50.
- Less twitchy: lower `micro_anticipate_lead_sec` to 3.2 and raise `micro_anticipate_min_heading_deg` to 0.9.
