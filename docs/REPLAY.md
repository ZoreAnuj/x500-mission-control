# Episode Replay Flight — `field_replay.py`

Replays a recorded **drone_hoop_30hz_v5** episode on the real X500 (or SITL) by
streaming its policy-style setpoints through ArduCopter. This is the sim2real
**controls** test: does the flight controller fly the same higher-level command
stream (`SET_POSITION_TARGET_LOCAL_NED`, position + velocity + yaw) that the
imitation-learning policy emits in simulation? No camera / no policy in the loop
yet — the recorded actions stand in for the policy.

## What it does

```
connect → no-RC failsafe → preflight gates → GUIDED → arm → takeoff
        → yaw to the episode's recorded heading
        → replay the episode as NED position+velocity+yaw setpoints
        → LAND
```

The control math — clip ±0.5 m/±0.3 rad → rotate body→NED by the **live** yaw →
`v = Δ / 0.5 s` (clamp 1.5 m/s) → `setpt += v·dt` → `yaw = live + Δyaw` — is
identical to the sim inference loop (`play_cmenc_imle_v1.py`); only the transport
(MAVLink instead of gRPC) differs. ArduCopter's GUIDED position controller plays
the role the sim's onboard cascade PID plays.

## Bundled episode

`data/drone_hoop_ep276.parquet` — **episode 276** (649 rows, 21.6 s @ 30 Hz),
the safest of the 300: gentle (8.8 % of Δx clipped), stays within **4.67 m** of
takeoff, peaks **+1.7 m** above the hover. It is loaded by default — no external
dataset needed.

**Ground track:** after yawing to compass **−90° (West)** it flies a near-straight
diagonal ≈ **4.5 m West + 1.3 m North**, climbing to ~1.7 m above the takeoff
hover. **Ensure ~6 m clear to the West/WNW.**

## Prerequisites

- **Props ON.** Close Mission Planner / the `server.py` dashboard — the serial
  port is exclusive.
- `pip install pymavlink pandas pyarrow numpy`
- Compass calibrated (field recal / CompassMot), GPS 3D lock. The script sets the
  no-RC failsafe itself (`FS_THR_ENABLE=0`, `FS_GCS_ENABLE=5` → link-loss = LAND).
- **Geofence (recommended):** `FENCE_ENABLE=1`, `FENCE_RADIUS=15`, `FENCE_ALT_MAX=6`.

## Run

```bash
# Real flight — bundled ep276, 10 Hz over the SiK radio, 2 m takeoff:
python field_replay.py --connect COM13 --baud 57600 --episode 276 --hz 10 --alt 2

# Hover-only drift baseline (no replay) — do this first:
python field_replay.py --connect COM13 --hover 60

# Props-off bench check (skips the preflight quality gates):
python field_replay.py --connect COM13 --force

# A different episode (needs the full external dataset):
python field_replay.py --dataset /path/to/drone_hoop_30hz_v5 --episode 50
```

Preflight gates (fix ≥ 3, sats ≥ 10, HDOP ≤ 1.5, EKF variances ≤ 0.5, level) must
pass or the script aborts; `--force` bypasses them for bench/SITL.

## Landing — always happens, first trigger wins

Landing uses the repo sequence (`land_and_disarm`): **`NAV_LAND` → watch the
descent while beating the GCS heartbeat** (so a slow descent isn't cut by
`FS_GCS`) → **wait for auto-disarm** (explicit disarm only as a fallback).
Triggered by whichever comes first:

1. **Episode complete**
2. **Commanded descent** after the straight/climb phase — the dataset has no
   landing tail (all 300 episodes end mid-climb), so this is a dormant safety net
3. **ENTER** in the terminal — **immediate land kill, at any time**
4. **Tracking/radius guard** — lands if the drone diverges > 2 m from the
   commanded setpoint or > 12 m from takeoff

## Safety layers (independent)

`ENTER → LAND` · tracking/radius guard → LAND · geofence RTL/LAND ·
`FS_GCS = LAND` if the script dies · `GUID_TIMEOUT` 3 s → hover if the stream stalls.

## After the flight

Every tick is logged to `replay_epNNN_field.csv` (same columns as the SITL runs):

```bash
python analyze_replay.py replay_ep276_field.csv --dataset <dataset> --episode 276 --hz 10
python watch_replay.py   replay_ep276_field.csv     # live preview (run during the flight too)
python render_replay_video.py replay_ep276_field.csv --hz 10   # mp4
```

## SITL test (no hardware)

```bash
# in WSL, prebuilt ArduCopter SITL on tcp:5760:
python3 field_replay.py --connect tcp:127.0.0.1:5760 --episode 276 --force
```

Verified end-to-end in SITL: full flight → land → auto-disarm, and **ENTER
in-flight → immediate land + disarm**.
