# Architecture — Mission Control

A single Python process bridges the MAVLink link to the browser. No build step, no database, no framework on the frontend.

```
Pixhawk 6C ──SiK 57600──> COM port ──> server.py (pymavlink)
                                        ├─ reader thread:  MAVLink → shared STATE dict
                                        ├─ WebSocket /ws:  push STATE @ 5 Hz   ↓ to browser
                                        └─ WebSocket /ws:  receive commands    ↑ from browser
                                                              │  (arm / takeoff / land / kill / set_mode)
                                                              ▼
                                                     COMMAND_LONG / SET_MODE → vehicle
                                        static/index.html  ← served at  GET /
```

## Backend (`missioncontrol/server.py`)

- **Connection** is opened in a background worker at startup, so the web server binds and serves **immediately** — the UI is usable with or without the vehicle (it shows LINK LOST until a heartbeat arrives). `Connect` / `Disconnect` re-open / close the COM port at runtime; the single reader thread is resilient to the connection being `None` or closed.
- **`reader()` thread** calls `recv_match()` and folds each message into a shared `STATE` dict (latest value wins). `STATUSTEXT` and `COMMAND_ACK` are kept in short ring buffers.
- **WebSocket `/ws`** runs two coroutines: a *sender* that serialises `STATE` to JSON and pushes at 5 Hz, and a *receiver* that takes command JSON and dispatches it. Commands run in a thread-pool so a serial write never blocks the event loop. A `threading.Lock` serialises all MAVLink writes (reader reads, commands write).
- **Stream rates** are requested with `SET_MESSAGE_INTERVAL` once the first heartbeat arrives — tuned for the bandwidth-limited 57600 SiK link.
- **Heartbeat watchdog:** the sender marks `link=false` if no heartbeat for 3 s, which drives the LINK LOST banner.

## Frontend (`missioncontrol/static/index.html`)

- One self-contained file: HTML + CSS + vanilla JS. Dark "real-time monitoring" theme, tabular figures.
- A single WebSocket connection (auto-reconnecting) receives state frames and updates the DOM; buttons send command JSON back over the same socket.
- The **attitude horizon** is plain SVG — a clipped circle with a `<g>` that rotates by `-roll` and translates by `pitch`. No external library, so it works offline.
- **Safety UX:** slide-to-arm (range input), hold-to-kill (1.5 s fill timer), and takeoff disabled unless `armed && mode==GUIDED && fix>=3`. The LINK LOST banner is `pointer-events:none` and the status bar sits above it, so `Connect` stays clickable when the link drops.

## Commands

| UI action | MAVLink |
|-----------|---------|
| Set GUIDED / LAND / RTL | `SET_MODE` (ArduCopter custom mode) |
| Slide to arm | `MAV_CMD_COMPONENT_ARM_DISARM` p1=1 |
| Takeoff | `MAV_CMD_NAV_TAKEOFF`, altitude in param7 |
| Hold to kill | `MAV_CMD_COMPONENT_ARM_DISARM` p1=0, **p2=21196** (force, works in flight) |
| Connect / Disconnect | open / close the serial link at runtime |
| Refresh | re-request `SET_MESSAGE_INTERVAL` stream rates |
