# Dataset Replay → ArduCopter (SITL first) — Design

**Goal.** Before any real flight, quantify how well ArduCopter's GUIDED controller tracks the *policy-style* command stream by replaying recorded training episodes from `drone_hoop_30hz_v5` (the dataset that trained the Drone Hoop policy). Same test later runs unchanged against the real X500 over SiK.

## Why replay instead of only canned patterns

Replayed actions are the exact action distribution the vision policy emits: 4-D body-frame waypoints `[Δx, Δy, Δz, Δyaw]` over a 0.5 s lookahead at 30 Hz. If GUIDED tracks these well, the sim2real command path is validated end-to-end.

Replay is **open-loop**: actions were recorded against the sim trajectory, and the shim rotates by *live* yaw, so the flown path diverges over the episode. That divergence rate is the measurement (how faithfully the command interface reproduces the intended motion), not a defect.

## Architecture

```
LeRobot episode (parquet)          control_shim (verbatim policy math)          ArduCopter
 action[t] = [Δx,Δy,Δz,Δyaw] ──▶  clip ±0.5 m/±0.3 rad                    ──▶  GUIDED
 @ 30 Hz                          rotate body→NED by LIVE yaw                   SET_POSITION_TARGET_LOCAL_NED
                                  v = Δ/0.5 s, clamp 1.5 m/s                    (frame LOCAL_NED, mask 0xDF8:
                                  setpt += v·dt   (seed at hover)                pos + vel_ff + yaw)
                                  yaw_cmd = wrap(live_yaw + Δyaw)
                                        │
                                        └──▶ CSV log: t, action, setpt, yaw_cmd,
                                             LOCAL_POSITION_NED (pos+vel), ATTITUDE
```

- **Target 1: SITL** (prebuilt ArduCopter binary, WSL, TCP 5760). No hoop, no vision — pure command-tracking test.
- **Target 2 (later): real X500** — same script, connection string `COM13`/57600; rate may drop to ~10 Hz over SiK.

**Reference trajectory (offline, for scoring):** integrate the same clipped actions with the *recorded* yaw (`atan2(sin ψ, cos ψ)` from `observation.state[2:4]`) → the intended NED path. Compare flown vs intended.

## Components

| Component | What | Where |
|---|---|---|
| `scripts/sitl_replay.py` | connect → GUIDED → arm → takeoff (default 2 m) → seed `setpt` at hover → stream episode actions @ 30 Hz through shim → land. `--episode N --dataset PATH --hz 30 --alt 2`. Logs CSV. | x500-mission-control |
| `scripts/analyze_replay.py` | CSV + dataset → metrics: pos RMSE (XY/Z) vs reference, yaw RMSE, per-axis divergence over time, achieved stream Hz, latency; plot (matplotlib). | x500-mission-control |

Reuses the proven arm/takeoff/mode sequences from `x500_first_flight.py` / `server.py`. Single reader thread + write lock, per repo convention.

## Safety / correctness notes

- **Clip parity:** the live shim clips Δpos to ±0.5 m; dataset actions can reach ~0.75 m (1.5 m/s × 0.5 s). Replay applies the same clip as live inference (parity is the point). Report % of clipped samples per episode.
- `GUID_TIMEOUT` (3 s): a stalled stream latches the last position target → hover, not flyaway.
- SITL phase has zero physical risk; field phase inherits the existing kill + FS_GCS=LAND safety net.
- EKF origin: seed `setpt` from `LOCAL_POSITION_NED` after takeoff settles; never command before a position estimate.

## Acceptance

1. SITL: episode 0 replays end-to-end; analyzer emits metrics + plot.
2. Flown-vs-reference divergence and tracking RMSE reported for ≥3 episodes.
3. Same script connects to a serial port unchanged (arg only) for the later field run.
