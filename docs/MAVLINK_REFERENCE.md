# MAVLink / ArduCopter Reference

A working reference for the MAVLink protocol as used by this project (Holybro X500 · Pixhawk 6C · **ArduCopter** firmware · pymavlink over a 57600 SiK radio). Compiled from the official docs — every section cites its sources. Written to be read alongside `NOTES.md` (field findings) and `ARCHITECTURE.md` (how the web GCS is wired).

**Primary sources:** mavlink.io (`/en/services/*`, `/en/messages/common.html`, `/en/messages/minimal.html`, `/en/messages/ardupilotmega.html`, `/en/mavgen_python/`), ardupilot.org (copter + dev wikis), the canonical `common.xml` / `ardupilotmega.xml` dialects, and `pymavlink/mavutil.py`.

> **Firmware note.** This vehicle runs **ArduCopter**. Mode numbers and several command param encodings are ArduPilot-specific — **do not reuse PX4 values**. Where `master` MAVLink has migrated magic numbers to named enums (reboot, storage, arm-force), the field firmware still honors the classic magic values; both are noted.

---

## 1. Command Protocol

Source: https://mavlink.io/en/services/command.html · https://mavlink.io/en/messages/common.html#MAV_RESULT

### COMMAND_LONG (#76) vs COMMAND_INT (#75)
Both carry a `MAV_CMD` in `command` + 7 params, both are answered by `COMMAND_ACK`.

| | COMMAND_LONG | COMMAND_INT |
|---|---|---|
| param1–4 | float | float |
| param5/6 (x,y) | float | **int32 scaled** (e.g. lat/lon ×1e7) |
| param7 (z) | float | float |
| frame | implicit/arbitrary | explicit `frame` (MAV_FRAME) |
| use for | float params 5/6 | **anything carrying lat/lon** (avoids float precision loss) |

`COMMAND_LONG.confirmation` = 0 on first send; **increment on each resend** (duplicate detection). COMMAND_INT has no confirmation field.

### COMMAND_ACK (#77) → MAV_RESULT
Fields: `command`, `result`, ext `progress` (0–100, 255=n/a), `result_param2` (denial reason), `target_system/component`.

| # | MAV_RESULT | Meaning / action |
|---|---|---|
| 0 | ACCEPTED | Valid, executing. |
| 1 | TEMPORARILY_REJECTED | Busy/wrong state — **retry later**. |
| 2 | DENIED | Refused, retry won't help — **terminal**. |
| 3 | UNSUPPORTED | Unknown command — **terminal**. |
| 4 | FAILED | Accepted but execution failed. |
| 5 | IN_PROGRESS | Long op running — **extend timeout**, expect progress ACKs. |
| 6 | CANCELLED | Long op aborted. |

Retry: on no ACK, resend (increment `confirmation`) a few times. Treat TEMPORARILY_REJECTED as retry; DENIED/UNSUPPORTED as terminal (retrying just spams the SiK link).

---

## 2. Commands used by this project (param1..param7)

Source: `common.xml` / `ardupilotmega.xml` · https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html

| Command (id) | Params |
|---|---|
| **NAV_TAKEOFF** (22) | p1=min pitch, p4=yaw (NaN=current), p5/6=lat/lon, **p7 = altitude (m, above home/EKF origin)**. ⚠️ **altitude is p7, not p1.** |
| **NAV_LAND** (21) | p1=abort alt, p2=precision-land mode, p4=yaw, p5/6=lat/lon, p7=alt. |
| **COMPONENT_ARM_DISARM** (400) | p1=1 arm / 0 disarm; **p2=force**: 0 normal, **21196 = ArduPilot magic** (force-arm bypassing pre-arm; or **in-flight disarm/kill**). |
| **DO_MOTOR_TEST** (209) | p1=motor instance (1..N), p2=throttle type, p3=throttle, p4=timeout (s), p5=motor count (mission only; 0=single), p6=test order. |
| **SET_MESSAGE_INTERVAL** (511) | p1=message id, **p2=interval in µs** (`1e6/Hz`; −1 disable, 0 default), p7=response target. |
| **DO_START_MAG_CAL** (42424, *ardupilotmega dialect*) | p1=mag bitmask (0=all), p2=retry, p3=autosave, p4=delay, p5=autoreboot. Accept=42425, Cancel=42426. |
| **PREFLIGHT_REBOOT_SHUTDOWN** (246) | classic: **p1=1 reboot autopilot** (0 none, 2 shutdown, 3 bootloader), p2=companion. master enum adds p6=20190226 FORCE. |
| **PREFLIGHT_STORAGE** (245) | classic: **p1=1 write params to EEPROM** (0 read, 2 factory, 3 sensor), p2=mission, p3=logging. Only when disarmed. |

`MOTOR_TEST_THROTTLE_TYPE`: **0=PERCENT (0–100)**, 1=PWM (1000–2000), 2=PILOT, 3=3D_PERCENT (−100..100), 4=3D_PWM, 5=3D_PILOT. `MOTOR_TEST_ORDER`: 0=DEFAULT, 1=SEQUENCE, 2=BOARD.

**Gotchas:** takeoff alt is p7; arm-force is 21196 not 1; SET_MESSAGE_INTERVAL is **microseconds**; motor-test throttle range depends on the type param; DO_START_MAG_CAL needs the ardupilotmega dialect.

---

## 3. Flying in GUIDED mode (lift → hold → land)

Source: https://ardupilot.org/copter/docs/ac2_guidedmode.html · https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html · https://mavlink.io/en/messages/common.html

### Required order (each step must succeed or takeoff is silently ignored)
1. **Set mode GUIDED**
2. **Arm** (must pass arming checks; needs a position estimate — see §5)
3. **NAV_TAKEOFF**, altitude in **param7** (m above home). Copter ramps to hover thrust via the position controller, climbs, then **auto-hovers**.

### Holding altitude/position — the key fact
- **Position targets and the post-takeoff hover are LATCHED** — the vehicle station-keeps against wind indefinitely with **no further messages required**.
- **Velocity / acceleration / attitude-rate targets are NOT latched** — they must be re-sent **≥1 Hz** or the vehicle stops/levels after **`GUID_TIMEOUT` (default 3.0 s)**.

### Re-commanding position: SET_POSITION_TARGET_LOCAL_NED (#84) / _GLOBAL_INT (#86)
`type_mask` is a bitmap where **a set bit = IGNORE that field** (clear the bits you want to command):

| Bits | Fields |
|---|---|
| 0,1,2 | position X,Y,Z |
| 3,4,5 | velocity X,Y,Z |
| 6,7,8 | accel X,Y,Z |
| 9 | FORCE_SET (accel→force) |
| 10,11 | yaw, yaw-rate |

Ready masks: **position+yaw = 0xDF8 (3576)**, velocity-only = 0xDC7 (3527).
Frames (LOCAL_NED): 1=LOCAL_NED (**Z negative-up**, rel EKF origin), 7=LOCAL_OFFSET_NED, 8=BODY_NED, 9=BODY_OFFSET_NED (rel current pos+heading). GLOBAL_INT: 0=GLOBAL(MSL), 3=GLOBAL_RELATIVE_ALT (above home), 10=TERRAIN_ALT; lat/lon = deg×1e7.

### GUIDED_NOGPS + SET_ATTITUDE_TARGET (#82)
Accepts **only** attitude (quaternion) + body rates + thrust; **cannot autonomously take off** (no position/altitude loop). type_mask (set=ignore): bit0/1/2 body rates, bit5 thrust, bit6 attitude → ArduPilot uses **0x07**. ⚠️ `thrust` defaults to a **climb-rate encoding (0.5 = hover)** unless `GUID_OPTIONS` bit 3 (=8) is set for raw thrust 0..1. Subject to `GUID_TIMEOUT` — must be streamed continuously.

### Recipe — lift → hold N seconds → land
```
set_mode(GUIDED) → ARM (400, p1=1) → NAV_TAKEOFF (22, p7=alt)
→ watch GLOBAL_POSITION_INT.relative_alt until ≈ alt
→ hold: position is latched, just wait N seconds (no streaming needed)
→ NAV_LAND (21)  [or switch to LAND mode];  auto-disarms after touchdown (DISARM_DELAY)
```

---

## 4. Telemetry messages (fields · units · scaling)

Source: https://mavlink.io/en/messages/minimal.html#HEARTBEAT · https://mavlink.io/en/messages/common.html

### HEARTBEAT (0)
`type` (MAV_TYPE: 2=QUAD, 4=HELI, 13=HEXA, 14=OCTO), `autopilot` (3=ArduPilot, 12=PX4, 8=GCS), `base_mode` (bitmask), `custom_mode` (uint32 = ArduCopter mode number), `system_status` (3=STANDBY, 4=ACTIVE, 5=CRITICAL, 6=EMERGENCY).
`base_mode` bits: 1=CUSTOM_MODE_ENABLED, 8=GUIDED, 16=STABILIZE, **128=SAFETY_ARMED**.
→ **armed** = `base_mode & 0x80`; mode valid when `base_mode & 0x01`, then map `custom_mode`.

### Unit cheat-sheet
| Field(s) | Encoding | Convert |
|---|---|---|
| lat, lon | degE7 | ÷1e7 → degrees |
| alt, relative_alt (mm) | mm | ÷1000 → m |
| vx/vy/vz, GPS vel (cm/s) | cm/s | ÷100 → m/s |
| hdg, cog, GPS yaw | cdeg | ÷100 → deg (65535=unknown) |
| eph, epv | ×100 | ÷100 → HDOP/VDOP (65535=unknown) |
| voltage_battery (SYS_STATUS) | mV (whole pack) | ÷1000 → V |
| current_battery | cA | ÷100 → A (−1=not measured) |
| battery_remaining | % | direct (**−1 = no battery monitor**) |
| roll/pitch/yaw (ATTITUDE) | **radians** | ×57.29578 → deg |
| VFR_HUD fields | SI floats | none |

- **GLOBAL_POSITION_INT (33)** — EKF-fused. `relative_alt` = above home (use for the altitude readout); `alt` = MSL.
- **GPS_RAW_INT (24)** — raw receiver. `fix_type` (GPS_FIX_TYPE: 0 NO_GPS, 1 NO_FIX, 2 2D, 3 3D, 4 DGPS, 5 RTK_FLOAT, 6 RTK_FIXED), `eph`→HDOP, `satellites_visible` (255=unknown).
- **SYS_STATUS (1)** — coarse battery + `onboard_control_sensors_present/enabled/health` bitmask (1 gyro, 2 accel, 4 mag, 8 baro, 32 GPS, 33554432 battery, 268435456 prearm). A GCS flags a sensor red when its bit is *present* but *not healthy*.
- **VFR_HUD (74)** — groundspeed m/s, heading deg, throttle %, alt m, climb m/s (already SI).
- **ATTITUDE (30)** — roll/pitch/yaw in **radians** (+ rates).
- **STATUSTEXT (253)** — `severity` (MAV_SEVERITY: 0 EMERGENCY … 2 CRITICAL, 4 WARNING … 7 DEBUG), `text[50]` (not null-terminated; long msgs chunked via `id`/`chunk_seq`). ArduPilot pre-arm failures arrive here as `"PreArm: …"`.
- **BATTERY_STATUS (147)** — richer than SYS_STATUS: multi-battery, **per-cell** `voltages[10]` (mV/cell), `current_consumed` (mAh), temperature, faults. Prefer it when present.
- **COMMAND_ACK (77)** — see §1.

---

## 5. ArduCopter modes, arming & pre-arm checks

Source: https://ardupilot.org/copter/docs/flight-modes.html · https://ardupilot.org/copter/docs/common-prearm-safety-checks.html · `ArduCopter/mode.h`

### Mode numbers (`custom_mode`)
0 STABILIZE, 1 ACRO, 2 ALT_HOLD, 3 AUTO, 4 GUIDED, 5 LOITER, 6 RTL, 7 CIRCLE, 9 LAND, 11 DRIFT, 13 SPORT, 14 FLIP, 15 AUTOTUNE, 16 POSHOLD, 17 BRAKE, 18 THROW, 20 GUIDED_NOGPS, 21 SMART_RTL, 22 FLOWHOLD, 23 FOLLOW, 24 ZIGZAG, 27 AUTO_RTL. (8/10/12 retired.)
**Need a position estimate:** AUTO, GUIDED, LOITER, RTL, CIRCLE, POSHOLD, BRAKE, DRIFT, FOLLOW, SMART_RTL, THROW, ZIGZAG. **Don't:** STABILIZE, ACRO, ALT_HOLD, LAND, GUIDED_NOGPS.

### ARMING_CHECK (bitmask, default 1 = all)
0=disable all (bench only), 1=all. Bits: 2=baro, **4=compass**, **8=GPS lock**, 16=INS, 32=params, **64=RC**, 128=board voltage, 256=battery, 2048=safety switch, 4096=GPS config. e.g. `72` = GPS(8)+RC(64) only.
`ARMING_NEED_LOC=1` additionally forces a valid location before arming (closes the "armed without a position estimate" hazard).

### How failures surface
The first failing check shows red on the GCS; while disarmed, failures also broadcast as `STATUSTEXT` (`PreArm: …`) about **every 30 s** (LED flashes yellow). **"PreArm: Need Position Estimate"** appears when a position-requiring mode is selected without an EKF solution.

### Force-arm
`COMPONENT_ARM_DISARM` p1=1, **p2=21196** forces arming past pre-arm checks (and force-disarms in flight — the kill path). Emergency use only.

---

## 6. Telemetry stream rates (SiK 57600 is bandwidth-limited)

Source: https://mavlink.io/en/services/message.html · https://ardupilot.org/dev/docs/mavlink-requesting-data.html

### Modern: MAV_CMD_SET_MESSAGE_INTERVAL (511)
p1=message id, **p2=interval µs** (`interval = 1_000_000 / Hz`; −1 disable, 0 default), p7=response target. Autopilot ACKs cmd 511. Query current via GET_MESSAGE_INTERVAL (510) → MESSAGE_INTERVAL (244). One-shot via REQUEST_MESSAGE (512).

### Legacy: REQUEST_DATA_STREAM (66) + MAV_DATA_STREAM
Group-based, deprecated but still honored by ArduPilot. Groups: 0 ALL, 1 RAW_SENSORS, 2 EXTENDED_STATUS (SYS_STATUS, GPS_RAW_INT), 3 RC_CHANNELS, 6 POSITION (GLOBAL_POSITION_INT), 10 EXTRA1 (ATTITUDE), 11 EXTRA2 (VFR_HUD), 12 EXTRA3 (BATTERY_STATUS). ArduPilot persists these as **`SRx_*` params** (x = index of the MAVLink serial port; SR1 = telemetry radio). `SERIALn_OPTIONS` bit 12 makes a port ignore GCS stream requests and honor only `SRx_*`.

### Recommended rates @ 57600 (real throughput ≈4–5 KB/s, half-duplex)
Turn RAW_SENSORS **off**. ATTITUDE 4–5 Hz, GLOBAL_POSITION_INT 2–3 Hz, GPS_RAW_INT 1–2 Hz, SYS_STATUS 1–2 Hz, VFR_HUD 2–3 Hz. Leave headroom for param/mission bulk transfers.
Message ids: ATTITUDE 30, GLOBAL_POSITION_INT 33, GPS_RAW_INT 24, SYS_STATUS 1, VFR_HUD 74.

---

## 7. MAVLink GPS injection (indoor "fake GPS")

Source: https://ardupilot.org/mavproxy/docs/modules/GPSInput.html · https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html · `GPS_INPUT` in common.xml

1. Set **`GPS1_TYPE = 14` (MAVLink)** and **reboot** (RebootRequired — backend only created at boot).
2. Stream **GPS_INPUT (#232)** at **≥4 Hz** with `fix_type ≥ 3`, valid `lat/lon/alt` (lat/lon degE7), sats ≥6, low hdop. `ignore_flags` bitmap (set=ignore field; e.g. 8 = ignore horizontal velocity). The EKF sets its **origin** to the injected position (origin can't move once set) and starts outputting a position, unlocking GUIDED/LOITER.
3. **Why it drifts:** the EKF fuses a *constant* fake position with drifting IMU integration and (if velocity is ignored) has nothing to arrest the drift. Not fixable in software — needs real GPS, optical-flow+rangefinder (`EK3_SRC1_VELXY=5`), or VIO (`EK3_SRC1_*=6 ExternalNav`).
4. After the session, restore `GPS1_TYPE=1` + reboot so the real serial GPS re-detects.

---

## 8. Compass calibration via MAVLink

Source: https://mavlink.io/en/messages/ardupilotmega.html · https://ardupilot.org/copter/docs/common-compass-calibration-in-mission-planner.html

`DO_START_MAG_CAL` (42424): p1=mask (0=all), p2=retry, p3=autosave, p5=autoreboot. Progress via **MAG_CAL_PROGRESS (191)** `completion_pct`; result via **MAG_CAL_REPORT (192)** `cal_status` (4=SUCCESS) + **`fitness`** = RMS residual in milligauss (**lower is better**; residuals-too-high → relax the Fitness setting or raise `COMPASS_OFFS_MAX`). Accept a non-autosaved cal with `DO_ACCEPT_MAG_CAL` (42425). Reboot before arming.

---

## 9. pymavlink quick reference

Source: https://mavlink.io/en/mavgen_python/ · `pymavlink/mavutil.py`

```python
from pymavlink import mavutil
m = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)   # or "com13", "udpin:...", "tcp:127.0.0.1:5760"
m.wait_heartbeat(timeout=15)          # sets m.target_system / m.target_component — REQUIRED before targeted sends
msg = m.recv_match(type=["ATTITUDE","GPS_RAW_INT"], blocking=True, timeout=1)   # list avoids discarding other types
m.mav.command_long_send(m.target_system, m.target_component, cmd, 0, p1,p2,p3,p4,p5,p6,p7)  # always 7 params
m.mav.set_mode_send(m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, m.mode_mapping()["GUIDED"])
m.mav.param_set_send(m.target_system, m.target_component, b"GPS1_TYPE", 14, mavutil.mavlink.MAV_PARAM_TYPE_INT32)  # name is BYTES
```

**Threading:** the connection is **not thread-safe** — use one reader thread + a lock around all writes, and **keep reading continuously** (buffers fill otherwise). `recv_match(type='X')` silently discards other messages it consumes while hunting — pass a list or dispatch on `msg.get_type()`. Guard every read for `None` and `get_type()=='BAD_DATA'`. Force v2 with `os.environ['MAVLINK20']='1'` before import if needed. Dialect defaults to `ardupilotmega` (needed for mag-cal commands).

**Common mistakes:** sending before `wait_heartbeat()` (IDs=0); wrong serial baud (→ BAD_DATA); param name as `str` not `bytes`; wrong count of the 7 command params; busy-looping non-blocking `recv_match`; assuming thread safety.

---

## 10. How this project's code maps to the spec (cross-check)

Verified the repo's usage against the docs above — **the command sequences are correct**:

| Project code | Spec | ✓ |
|---|---|---|
| `NAV_TAKEOFF` altitude in param7 (`server.py`, `x500_first_flight.py`) | alt is p7 | ✓ |
| kill = `ARM_DISARM` p1=0, p2=21196, spammed 6× | force-disarm magic value | ✓ |
| `DO_MOTOR_TEST` throttle type=0 (percent) | 0=PERCENT | ✓ |
| `DO_START_MAG_CAL` p1=0 (all), p3=1 (autosave) | mask/autosave slots | ✓ |
| reboot p1=1, `PREFLIGHT_STORAGE` p1=1 | classic reboot/write | ✓ |
| `SET_MESSAGE_INTERVAL` `int(1e6/hz)` at ATT 5/POS 3/HUD 2/GPS 2/SYS 1 Hz | µs interval; matches 57600 guidance | ✓ |
| armed via `MAV_MODE_FLAG_SAFETY_ARMED`, `relative_alt/1000`, `hdg/100`, `eph/100` w/ 65535 guard, ATTITUDE `math.degrees` | telemetry scaling | ✓ |
| ArduCopter `MODES` dict (0 STABILIZE … 20 GUIDED_NOGPS) | mode.h numbers | ✓ |
| GPS injection: `GPS1_TYPE=14` + reboot, stream `gps_input_send` @5 Hz, fix_type=3 | injection recipe | ✓ |
| one reader thread + `threading.Lock` on writes | pymavlink threading pattern | ✓ |

**The one substantive gap is behavioral, not protocol** — see [`NOTES.md`](../NOTES.md) and the review of `x500_first_flight.py`: its takeoff→land window (2.5 s) is shorter than ArduCopter's ~3–4 s GUIDED throttle ramp, and it has **no latched-position hold phase**, so it hops-and-lands rather than genuinely "lift → maintain altitude → land." Per §3, holding altitude in GUIDED needs no streaming — just reach the target and wait — so the fix is to climb-watch to altitude, then wait `HOVER_SECS`, then land.

---

*Compiled 2026-07-01 from official MAVLink & ArduPilot documentation. All source URLs inline above.*
