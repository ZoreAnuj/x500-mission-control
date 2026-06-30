# X500 Mission Control

A minimal, dependency-light **web ground control station** plus a set of **pymavlink scripts** for commanding a **Holybro X500** quadcopter (Pixhawk 6C, **ArduCopter**) over a **SiK telemetry radio** — no MAVSDK/gRPC, no QGroundControl required.

Everything here was built and **tested against real hardware** on Windows, with the flight controller reached over a 915 MHz SiK radio on a serial COM port.

![status](https://img.shields.io/badge/firmware-ArduCopter-success) ![status](https://img.shields.io/badge/link-pymavlink%20%2F%20SiK-blue)

---

## What's inside

| Part | What it does |
|------|--------------|
| **`missioncontrol/`** | Web GCS — FastAPI + pymavlink backend, single-file dark dashboard. Live telemetry, attitude horizon, safety controls. |
| **`scripts/`** | Standalone, self-contained pymavlink scripts (motor test, takeoff/land, GPS monitor, compass cal). |
| **`scripts/diagnostics/`** | Small probes for bringing up a flaky link (raw byte read, baud scan, GPS poll, kill-switch setup). |
| **`NOTES.md`** | **Read this.** Every gotcha and finding from getting this flying — saves you hours. |
| **`docs/ARCHITECTURE.md`** | How the web mission control is wired. |

### Mission Control features
- **Live telemetry @ 5 Hz** — GPS (fix / sats / HDOP / lat / lon), battery, altitude (relative), ground speed, heading, throttle
- **Live SVG attitude horizon** (roll/pitch) — zero dependencies, works fully offline
- **STATUSTEXT log** — surfaces ArduCopter `PreArm: …` failures (the #1 reason "arm does nothing")
- **Safety controls** — Set GUIDED · slide-to-arm · gated Takeoff · Land · RTL · **hold-to-kill** (force-disarm)
- **Connect / Disconnect / Refresh** — release the COM port without killing the server
- **LINK LOST** banner on heartbeat loss

---

## Hardware / prerequisites

- A **Holybro X500** (or any ArduCopter vehicle) with a Pixhawk-class flight controller
- A **SiK telemetry radio pair** (e.g. Holybro 915/433 MHz) — one on the vehicle, one on USB
- **Python 3.10+**
- The ground radio's **serial COM port** — on Windows find it in Device Manager → Ports (FTDI, `VID_0403`)

## Install

```bash
pip install -r requirements.txt
```

## Configure (one line)

Everything defaults to **`COM13` @ `57600` baud**. Change `PORT` / `BAUD` at the top of `missioncontrol/server.py` and each script in `scripts/` to match your ground radio's COM port. (Linux/macOS: use a path like `/dev/ttyUSB0`.)

---

## Quick start — Mission Control

```bash
python missioncontrol/server.py
```
Then open **http://127.0.0.1:8090** in your browser.

- The dashboard comes up **immediately**, even with the drone off — it shows **LINK LOST** until a heartbeat arrives.
- Power the vehicle → GPS, battery, attitude, and position fill in live.
- The server **holds the COM port exclusively**. Use the **Disconnect** button to free it for another tool, **Connect** to grab it back.

### Flying from the dashboard
1. Wait for **GPS 3D fix** (status bar turns green) — ArduCopter won't arm in GUIDED without it.
2. **Set GUIDED**.
3. **Slide to arm** (the slider, not a click — prevents accidental arming).
4. Enter an altitude and hit **TAKEOFF** (enabled only when *armed + GUIDED + 3D fix*).
5. **LAND** or **RTL** to come down. **HOLD TO KILL** (1.5 s) force-disarms in an emergency.

---

## Standalone scripts

Each is self-contained — `python scripts/<name>.py`. Close Mission Planner / the mission-control server first (the COM port is exclusive).

| Script | Purpose |
|--------|---------|
| `x500_motor_test.py` | Spin all 4 motors at low throttle (`DO_MOTOR_TEST`). **Props off.** |
| `x500_first_flight.py` | Arm → GUIDED → takeoff → brief hover → land → disarm. `--nogps` flag for GPS-denied. |
| `x500_fakegps_flight.py` | Inject a synthetic GPS so the EKF gets a fix **indoors**, then fly. (Drifts — see NOTES.) |
| `x500_gps_monitor.py` | Live GPS fix / sats / HDOP watcher; flags the moment it's ready to fly. |
| `x500_compass_cal.py` | Onboard magnetometer calibration with live progress. |

### Diagnostics (`scripts/diagnostics/`)
| Script | Purpose |
|--------|---------|
| `raw_read.py` | Are *any* bytes arriving on the COM port? Distinguishes "link down" from "parse issue". |
| `baud_scan.py` | Scan common bauds for a MAVLink heartbeat. |
| `gps_poll.py` | Poll GPS for up to 60 s, print fix/sats. |
| `set_killswitch.py` | Set `RC5_OPTION=31` (Motor Emergency Stop) on an RC switch. |

---

## ⚠️ Heads-up / safety (read before flying)

- **Props OFF** for bench and motor tests. Spinning motors with a loose prop will flip/crash the vehicle.
- **RC transmitter ON** with a kill switch for *any* flight — ArduCopter's **radio failsafe disarms** if the TX is off, ~1 s after arming.
- **GUIDED needs a 3D GPS fix.** Indoors it won't arm/takeoff. Use real GPS outdoors, or the fake-GPS path (which **drifts** — no real position feedback; cage/tether it).
- **GUIDED takeoff is gentle** — throttle ramps over ~3–4 s to reach hover thrust. A too-short airborne window lands before it ever lifts.
- **The kill switch force-disarms** (`MAV_CMD_COMPONENT_ARM_DISARM`, `param2=21196`). This works **in flight**, i.e. it will drop the vehicle. Emergency use only.
- **One program owns the COM port.** Mission Control and the scripts are mutually exclusive — `Disconnect` in the UI, or stop one before starting the other.
- **Port 8000 may be blocked** on some Windows setups (endpoint security); the server uses **8090**.
- **After a fake-GPS session, reboot the FC** so the real serial GPS re-detects.

Full findings and the why behind each of these: **[NOTES.md](NOTES.md)**.

---

## License

MIT — see [LICENSE](LICENSE).

*Built with [Claude Code](https://claude.com/claude-code).*
