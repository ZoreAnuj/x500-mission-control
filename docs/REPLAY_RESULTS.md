# Episode Replay — Field Test Results

Real-hardware results from `scripts/field_replay.py` replaying recorded
`drone_hoop_30hz_v5` episodes on the Holybro X500 (ArduCopter, GPS, no RC).
Raw per-tick logs are in [`results/`](../results/).

---

## Episode 276 — 2026-07-06 (outdoors, GPS lock)

**Setup:** `--episode 276 --hz 10 --alt 2`, stride 3 → **217 setpoints @ 10 Hz (21.6 s)**, recorded yaw0 = −90°.
Vehicle state: 13 sats, HDOP 0.95, EKF `POS_HORIZ_ABS`, `MOT_THST_HOVER=0.6`.
Outcome: **full episode completed** (no guard/descent/ENTER abort), landed clean.
Log: [`results/replay_ep276_field.csv`](../results/replay_ep276_field.csv).

### Altitude (up, relative to replay start)
| | peak | final |
|---|---|---|
| Dataset (expected) | +1.68 m | — |
| Commanded | +1.67 m | +1.67 m |
| **Flown** | **+1.97 m** | +1.97 m |

Commanded matches the dataset. Flown **overshot by ~0.30 m** (mild over-climb; `MOT_THST_HOVER=0.6` slightly above true hover) and **never sagged** (min −0.00 m).

### Horizontal displacement (from replay start)
| | North | East | max radius |
|---|---|---|---|
| Commanded | +1.57 m | +4.38 m | 4.65 m |
| **Flown** | **+1.57 m** | **+4.05 m** | 4.34 m |

North exact; East lagged 0.33 m (normal control lag at speed). Path ran mostly East because yaw0 = −90°. Well inside the 12 m radius guard.

### Tracking error (flown vs commanded setpoint)
| axis | mean | max | guard |
|---|---|---|---|
| horizontal | **0.21 m** | 0.70 m | 2.0 m |
| vertical | 0.06 m | 0.43 m | — |

### Attitude / yaw
- roll −2…+11°, pitch −5…+6° (modest; commanded speed ≤1.5 m/s)
- yaw tracked the full −90…+90° sweep (`yaw_cmd` −90…91°, flown −90…89°)

### Verdict
The replay **faithfully reproduced the dataset trajectory** — sub-metre tracking throughout (0.21 m mean horizontal), full episode flown, clean modest attitudes, no altitude sag. The only deviation is a harmless ~0.3 m steady altitude overshoot from `MOT_THST_HOVER` being a touch high; `MOT_HOVER_LEARN=2` will refine it over more flights.
