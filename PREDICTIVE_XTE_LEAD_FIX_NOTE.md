# Predictive XTE lead fix

This build adds cross-track-error prediction to the route capture controller.

Problem observed: the aircraft waited until `xte` crossed 0 before changing steer, causing an S-turn.

Fix:
- Estimate signed XTE rate from current and previous XTE.
- Low-pass filter the rate.
- Use `xte + xte_rate * lead_time` for steering bias.
- If the predicted XTE will cross zero soon, steer changes before displayed XTE reaches zero.

New ini keys:

```ini
predictive_xte_lead=true
predictive_xte_lead_time_sec=3.2
predictive_xte_max_lead_m=22.0
predictive_xte_rate_filter_sec=0.55
predictive_xte_min_active_m=0.8
```

Tuning:
- Earlier counter-steer: increase `predictive_xte_lead_time_sec` to 3.8 or 4.2.
- Less early weave: reduce `predictive_xte_lead_time_sec` to 2.4-2.8 or increase `predictive_xte_rate_filter_sec` to 0.75.
