# Micro Pre-zero Direct Steer Fix

This build fixes the case where small course corrections only started to unwind the tiller after XTE crossed zero.

Key change:
- Keep the previous predictive XTE and geometric heading-rate lead.
- Add a direct pre-zero steer return layer after PID target generation.
- When the aircraft is still on one side of the planned taxi route but the tiller/PID is still steering toward the route centerline, the controller blends the requested steer toward the opposite side before XTE reaches zero.

New config entries:

```ini
micro_anticipate_direct_steer=true
micro_anticipate_direct_band_m=7.0
micro_anticipate_counter_steer_ratio=0.32
micro_anticipate_counter_min_ratio=0.10
micro_anticipate_toward_steer_threshold=0.010
micro_anticipate_direct_blend=1.15
micro_anticipate_pid_return_deg=15.0
```

Tuning:
- Earlier/stronger return: increase `micro_anticipate_direct_band_m`, `micro_anticipate_counter_steer_ratio`, or `micro_anticipate_pid_return_deg`.
- Less twitch: decrease `micro_anticipate_counter_steer_ratio` or `micro_anticipate_direct_blend`.
