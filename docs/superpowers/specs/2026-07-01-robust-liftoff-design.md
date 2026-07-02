# Robust Liftoff — Design Spec

**Date:** 2026-07-01
**Target:** `scripts/x500_first_flight.py` (GPS path), modified in place
**Goal:** Make autonomous liftoff → altitude-hold → land reliable, eliminating the two observed failures: (1) drift while on GPS, (2) toppling flat on the ground during takeoff.

## Problem analysis

Two failures observed in earlier real-hardware tests:

1. **Toppled flat on the ground (dynamic rollover).** The vehicle tilts laterally to correct a perceived position error while a landing leg still touches ground → the leg pivots → it rolls. Amplified by `TARGET_ALT=0.5 m` (never clears ground effect) and a fixed 2.5 s airborne window that sends `LAND` mid-throttle-ramp (ArduCopter's GUIDED takeoff ramps over 3–4 s per NOTES.md), so the vehicle is only ever partially airborne — the exact topple condition.
2. **Drift even with GPS.** Position-estimate *quality* was poor (measured HDOP 5.0 / 6 sats). GUIDED faithfully chases a several-meter-wandering estimate; a marginal compass (NOTES.md fitness ~28) adds circular "toilet-bowl" drift. The current script's `ARMING_CHECK=0` on weak GPS disables the very checks that would catch this.

**Strategy:** gate hard on estimate quality *before* arming, take off to a safe altitude, and give takeoff/hover/land their proper time — instead of forcing past safety checks.

## Decisions (agreed)

- **Safety stance:** abort by default with a clear reason; `--force` flag bypasses gates for deliberate bench tests.
- **Altitude:** `TARGET_ALT = 2.0 m`.
- **GPS/position gate:** strict — sats, HDOP, AND EKF variances.
- **File:** modify `x500_first_flight.py` in place; keep `--nogps` path unchanged.
- **Compass:** gate on EKF `compass_variance`; on failure, print instructions to run `x500_compass_cal.py`.

## Design

### Constants (top of file)
```
TARGET_ALT   = 2.0    # m AGL
HOVER_SECS   = 5      # s hold at altitude
MIN_SATS     = 10
MAX_HDOP     = 1.5
MAX_EKF_VAR  = 0.5    # pos_horiz / velocity / compass variance ceiling
MIN_VOLTS    = 13.0   # refuse to "fly" on USB power (~0 V) or a flat pack
LEVEL_DEG    = 3.0    # max |roll|,|pitch| at arm
FORCE        = "--force" in sys.argv
```

### Phase 1 — Preflight gates (new)
Poll telemetry up to ~60 s; require ALL of the following before arming (each prints the exact reason on failure; any `PreArm:` STATUSTEXT is surfaced). `--force` downgrades every gate to a warning.

| Gate | Requirement | Source msg |
|---|---|---|
| GPS fix | `fix_type >= 3` | GPS_RAW_INT |
| Satellites | `sats >= MIN_SATS` | GPS_RAW_INT |
| HDOP | `hdop <= MAX_HDOP` | GPS_RAW_INT (`eph/100`) |
| EKF position | `pos_horiz_variance <= MAX_EKF_VAR` | EKF_STATUS_REPORT |
| EKF velocity | `velocity_variance <= MAX_EKF_VAR` | EKF_STATUS_REPORT |
| EKF compass | `compass_variance <= MAX_EKF_VAR` → else print "run x500_compass_cal.py" | EKF_STATUS_REPORT |
| Battery | `MIN_VOLTS < volt` (rejects ~0 V USB power) | SYS_STATUS |
| Level & still | `|roll|,|pitch| < LEVEL_DEG` | ATTITUDE |
| RC present | RC-receiver health bit set (else radio failsafe disarms ~1 s after arm) | SYS_STATUS sensor health |

`ARMING_CHECK` stays enabled (remove the `ARMING_CHECK=0` on weak GPS). Request EKF_STATUS_REPORT via SET_MESSAGE_INTERVAL / data-stream so variances arrive.

### Phase 2 — Arm & climb (timing fix)
Set GUIDED (confirm via HEARTBEAT) → arm (surface STATUSTEXT, check COMMAND_ACK) → `NAV_TAKEOFF` p7=`TARGET_ALT` → **climb-watch with `wait_alt()` until `relative_alt >= TARGET_ALT - margin`** (up to ~15 s), replacing the fixed 2.5 s window. Report peak/throttle. If it never reaches ~0.15 m, report NO LIFTOFF and land+disarm.

### Phase 3 — Hover, land, settle-disarm
At altitude, **hold `HOVER_SECS`** (GUIDED station-keeps; position latched — no streaming). Optionally send one `SET_POSITION_TARGET_LOCAL_NED` at current position (type_mask 0xDF8) to firmly re-assert the hold and avoid a lurch. Then `NAV_LAND` → watch descent → **wait for ArduPilot's own auto-disarm** (poll armed flag) with a timeout fallback to a normal disarm. Do not force-disarm mid-descent.

### Error handling
Any phase failure → land (if airborne) → disarm → restore any changed params → exit non-zero with reason. `--nogps` path unchanged.

## Out of scope
- Fixing GPS/compass hardware quality (the gate refuses; it does not pretend to fix drift in software).
- Web dashboard / other scripts.
- The `--nogps` fake-GPS flow (untouched).

## Addendum — transmitter-less operation (no RC)

This airframe has **no RC transmitter**, so the manual kill lives in the web UI (HOLD-TO-KILL,
already present in `server.py`). Two coupled changes make no-RC flight safe:

- **Radio failsafe OFF** (`FS_THR_ENABLE=0`) — otherwise ArduCopter disarms ~1 s after arming.
- **GCS-heartbeat failsafe = LAND** (`FS_GCS_ENABLE=5`, default `FS_GCS_TIMEOUT` 5 s) as the
  automatic replacement: if the ground link drops, the vehicle lands itself.

Because FS_GCS only activates once a GCS heartbeat has been seen and triggers when they stop,
**the controller must stream heartbeats continuously**:
- `x500_first_flight.py` sends them **inline** in every in-flight loop (arm-wait, takeoff-ACK,
  climb, hover, land) — all on the main thread, no locking. The RC pre-flight gate is removed;
  `set_failsafe_norc()` applies the two params each run (verified via read-back on live hardware).
- `server.py` runs a **`gcs_heartbeat()` daemon at 2 Hz** and calls `set_failsafe()` after the
  first heartbeat, so UI-driven flights get the same protection.

Consequence to remember: **if the script/server stops or the SiK link drops while armed, the
vehicle LANDs** (by design). `scripts/diagnostics/set_killswitch.py` (RC5_OPTION=31) is now moot.

## Success criteria
- With good GPS (sats ≥ 10, HDOP ≤ 1.5) + healthy EKF + battery + RC: climbs to ~2 m, holds ~5 s without toppling, lands, disarms.
- With poor GPS / no battery / not level: aborts before arming with a specific reason (unless `--force`).
- No blanket `ARMING_CHECK=0`; takeoff window is bounded by reaching altitude, not a sub-ramp timer.
