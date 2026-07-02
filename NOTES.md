# Field Notes & Findings

Hard-won lessons from bringing a Holybro X500 (Pixhawk 6C, ArduCopter) under pymavlink control over a SiK telemetry radio on Windows. Everything here was hit and solved against real hardware.

---

## Connection & platform

- **Firmware is ArduCopter** (heartbeat reports `fw=ArduPilot`). Don't reuse PX4 mode numbers — ArduCopter's `custom_mode` enum is vehicle-specific (0 STABILIZE, 4 GUIDED, 5 LOITER, 6 RTL, 9 LAND, 20 GUIDED_NOGPS …).
- **pymavlink direct over the COM port** is the reliable path. We abandoned MAVSDK/gRPC because endpoint-security software on the PC intercepted every localhost TCP listen socket, so `mavsdk_server`'s gRPC `connect()` hung forever. pymavlink needs no gRPC.
- **Find the right COM port:** the SiK ground radio is an FTDI device (`VID_0403`). Bluetooth COM ports are a common decoy.
- **The COM port is exclusive** — only one program at a time (Mission Planner, the server, or a script). "Access is denied" / `PermissionError(13)` means something else already holds it.
- **`raw_read.py` is the first diagnostic** when a link looks dead: if 0 bytes arrive, the problem is upstream of your software (radio not paired, drone off, wrong port) — not a parse bug.

## SiK radio

- Default serial-side baud is **57600**. Both radios must share NET ID + air-speed to pair; **solid green LED = linked**, blinking = searching.
- At 57600 the radio is **bandwidth-limited**. Don't request every message at high rate — use `SET_MESSAGE_INTERVAL` deliberately (we use ATTITUDE 5 Hz, position 3 Hz, GPS/SYS_STATUS 1–2 Hz). Telemetry over the radio can be slower than requested.
- Request data streams explicitly (`request_data_stream_send` or `SET_MESSAGE_INTERVAL`) or `GPS_RAW_INT` / `VFR_HUD` / `GLOBAL_POSITION_INT` may simply never arrive.

## Arming

- **GUIDED mode requires a position estimate.** Without GPS (or external nav), arming in GUIDED fails `result=4` → `PreArm: Need Position Estimate`, and `NAV_TAKEOFF` is rejected. STABILIZE arms without position but can't auto-takeoff.
- **RC transmitter must be ON** or `Radio Failsafe - Disarming` kills the motors ~1 s after arming. **Flying with NO transmitter** (our setup): set `FS_THR_ENABLE=0` to disable the radio failsafe, and use `FS_GCS_ENABLE=5` (GCS-heartbeat failsafe → LAND, default `FS_GCS_TIMEOUT` 5 s) as the replacement — the ground station **must stream GCS heartbeats** or the vehicle lands. The kill switch then lives in the web UI (HOLD-TO-KILL). `x500_first_flight.py` and `server.py` set these params and send heartbeats automatically. **Consequence:** if the controller stops or the SiK link drops while armed, the vehicle LANDs by design.
- **`BRD_SAFETY_DEFLT=0`** (no safety-button press required). Holybro ships it this way; if you flip it to 1, motor tests and arming silently break until reboot.
- **Watch `STATUSTEXT`** — `PreArm: …` messages are the only place arming-failure reasons appear. Without surfacing them, arming "silently does nothing."
- **Force-disarm / kill** = `MAV_CMD_COMPONENT_ARM_DISARM`, `param1=0`, **`param2=21196`** (the ArduPilot magic value). Works in flight, unlike a normal disarm. Spam it a few times to beat packet loss over the radio.

## GPS

- Excellent fix outdoors: `fix=3/4`, 15–22 sats, HDOP ~0.6. Indoors you get nothing.
- **`fix_type=0` (no-GPS) ≠ "searching".** Type 0 means the FC sees no GPS data at all (loose cable, or `GPS1_TYPE` set to MAVLink-fake). A real GPS searching shows `fix_type=1` and detects the module.
- **After a fake-GPS session, reboot the FC** — the real serial GPS won't re-detect until a reboot (stays at `fix=0`, then jumps to `fix=3/4`).
- **Poll GPS, don't trust one read.** A single early `GPS_RAW_INT` can report a stale `fix=0`; sample for a few seconds and take the best.

## No-GPS / indoor flight (researched + tested)

- **Fake GPS:** set `GPS1_TYPE=14` (MAVLink GPS) + reboot, then stream `GPS_INPUT` (#232) at ~5 Hz (`fix_type=3`, sats=15, HDOP=0.6, zero velocity). The EKF auto-sets origin/home and GUIDED + `NAV_TAKEOFF` work indoors. **But it drifts** — it holds against a *fixed fake* position with no real feedback, so it slides and the EKF can diverge. Cage/tether it. Not fixable in software.
- **The real fix for indoor drift:** optical flow + rangefinder (e.g. Matek 3901-L0X, `EK3_SRC1_VELXY=5`, `POSZ=2`) or a VIO camera. `GUIDED_NOGPS` (mode 20) only accepts streamed `SET_ATTITUDE_TARGET` — no autonomous takeoff.
- The static-fake-fix EKF needs **time to settle** (position variance to drop) before it will arm — expect a few retries over ~30 s.

## Compass

- GPS modes need a calibrated compass (`PreArm: Compass not calibrated`). `x500_compass_cal.py` triggers `MAV_CMD_DO_START_MAG_CAL` with live progress; rotate the vehicle through all orientations.
- Calibration "fitness" < ~16 is good; ~28 is marginal (interference near that compass) — passable but worth improving.

## Takeoff behaviour

- **GUIDED takeoff throttle ramps slowly** — ~3–4 s to reach hover thrust (~50 %). A < 3 s airborne window lands before the vehicle ever leaves the ground. Verified by capturing `VFR_HUD.throttle` (it climbed 0 → 24 → 51 % and was still rising when a short window cut it off).
- **Baro drift on the ground** after the vehicle is moved/rebooted: relative altitude can read +1 m and creep upward, then resets to ~0 at arm (home latches there). Let it settle for precise low hops.
- A prop installed wrong (CW prop on a CCW motor, or upside-down) → **asymmetric thrust → flips on takeoff.** Verify prop placement (2 CW + 2 CCW, diagonal pairs, right-side-up) after any prop change.

## Windows / tooling quirks

- **`conda run python -c "..."` swallows stdout**, and `subprocess.Popen` of a `.exe` under `conda run` fails (`WinError 2`). Run scripts as files with the full interpreter path instead.
- **TCP port 8000 can be blocked** by endpoint-security/Windows reservations (`WinError 10013`, "forbidden by access permissions") even when it's not in `netsh … excludedportrange`. The mission-control server uses **8090**; check `netsh interface ipv4 show excludedportrange protocol=tcp` and a quick test-bind if you need another.
- **Don't block the web-server startup on `wait_heartbeat`** — open the link in a background worker so the dashboard serves with or without the drone (shows LINK LOST until a heartbeat lands).

## Kill switch on the transmitter

- TX16S (EdgeTX): map a 2-position switch (e.g. **SF → CH5**) in *Model → Mixes*, then set **`RC5_OPTION=31`** (Motor Emergency Stop). Save with `MAV_CMD_PREFLIGHT_STORAGE` and reboot. `set_killswitch.py` does the param part.

## Propeller change → bring-up & tuning (2026-07-01, real hardware)

Full ordered procedure in [docs/BRINGUP_CHECKLIST.md](docs/BRINGUP_CHECKLIST.md). Hard-won points:

- **A prop change does NOT need ESC re-calibration** in theory (ESC cal only sets throttle endpoints), but if you see **uneven RPM at equal throttle** (we saw CCW motors faster than CW), the ESCs' endpoints are mismatched — run ESC cal to equalise them. On this PWM build (`MOT_PWM_TYPE=0`, not DShot), cal is via **`ESC_CALIBRATION=3`** then a **battery power-cycle, props OFF** (the FC drives motors to full throttle to learn the high endpoint). It auto-resets to 0 when done. `x500_esc_cal.py`.
- **A direction test does NOT catch an upside-down prop.** An inverted prop spins the *correct direction* but blows air **up** → that corner won't lift → topple. `x500_motor_direction.py` now checks **airflow (down/up)** per motor, not just rotation.
- **Toppling on liftoff, props verified good, motors even → it's the takeoff throttle or CG, not the props.** Root cause here: `MOT_THST_HOVER` had been pre-set to **0.25** (conservative), but this airframe's real hover is high (it had pinned at the **0.6875 max** — marginal thrust-to-weight). A feedforward that far below real hover makes a **weak, slow, tippy liftoff**. Raising `MOT_THST_HOVER` toward the real hover point (**0.5 → 0.6**) gave a firm, clean liftoff. `MOT_HOVER_LEARN=2` then refines it in the air.
- **A *consistent* one-corner tip** (same direction every time) with even motors + good props points to **CG offset** (recenter the battery) or a prop damaged in a previous topple — not overall throttle. Don't keep re-flying a topping frame; each topple risks more damage.
- **Marginal thrust-to-weight** shows up as `MOT_THST_HOVER` learning toward its 0.6875 max. Below ~1.5 T/W it's twitchy and hard to fly; the real fix is bigger/higher-pitch props or less weight, not tuning.
- **Set `MOT_THST_EXPO` by prop diameter** — 10″ → 0.65 (default, correct here), 5″ → 0.55, ≥20″ → 0.75.

## No-transmitter operation — recap

- No RC on this build. `FS_THR_ENABLE=0` (radio FS off, else it disarms ~1 s after arm) + **`FS_GCS_ENABLE=5`** (GCS-heartbeat failsafe → LAND, `FS_GCS_TIMEOUT` ~5 s) as the replacement. **The controlling program MUST stream GCS heartbeats** (FS_GCS only activates after one is seen, then triggers on loss). Every cal/flight/motor script here sends heartbeats in a background thread or inline loop; `server.py` runs a 2 Hz `gcs_heartbeat()` daemon.
- **Consequence:** if the script/server stops or the SiK link drops while armed, the vehicle LANDs itself. That is the safety net that replaces the RC kill switch. The manual stops are the **web UI HOLD-TO-KILL** and, in `x500_first_flight.py`, **pressing ENTER (STOP → LAND)**.
- `ARMING_CHECK` set to **15550** (all checks except **RC (64)** and **Battery (256)**) so it arms with no receiver and no calibrated battery monitor. `set_killswitch.py` (RC5_OPTION=31) is moot without a TX.
- **You cannot inject a command while a script holds the exclusive serial port** — not from another script, not from the web UI. The stop must come from *inside* the running program (ENTER) or by Ctrl-C (which stops heartbeats → FS_GCS LANDs in ~5 s).

## Drift guard (in `x500_first_flight.py`)

- GUIDED station-keeps against the EKF position; drift quality is dominated by **GPS HDOP + compass cal**. Bad HDOP (we once saw 5.0) or a marginal compass → metres of wander. With HDOP ~0.8 and a clean compass cal, hold is tight (sub-metre).
- The script captures a reference position at liftoff/hover-start and **auto-LANDs if it drifts** past **2.0 m (climb) / 1.5 m (hover)** or exceeds **0.8 m/s** horizontal, debounced over 2 samples so a single noisy GPS blip doesn't false-trigger.

## Linux port / permissions

- Ground SiK radio enumerates as an FTDI FT231X → **`/dev/ttyUSB0`** (`ls /dev/serial/by-id/`). The user must be in the **`dialout`** group (`sudo usermod -aG dialout $USER`, then re-login) or the node reverts to `root:dialout 660` on every re-enumeration (FC reboot / replug) and you get `PermissionError(13)`.
- **0 bytes on a raw read = the vehicle is off / battery unplugged / radios unpaired** (solid green SiK LED = linked). It's upstream of the software, not a port bug — same lesson as `raw_read.py`.

## MAVLink → UI mapping (for the web GCS)

| Message | Feeds |
|---------|-------|
| `HEARTBEAT` | armed bit (`base_mode & 0x80`), flight mode (`custom_mode`), link watchdog |
| `GLOBAL_POSITION_INT` | lat/lon, **relative_alt** (above home — use this, not MSL `alt`), heading |
| `GPS_RAW_INT` | fix type, sat count, HDOP (`eph/100`) |
| `SYS_STATUS` | battery V/%, sensor health (`battery_remaining` is −1 if no battery monitor) |
| `VFR_HUD` | ground speed, throttle %, climb |
| `ATTITUDE` | artificial horizon (roll/pitch, radians → degrees) |
| `STATUSTEXT` | PreArm / failure messages |
| `COMMAND_ACK` | command success/failure feedback |
