#!/usr/bin/env python3
r"""X500 Mission Control — backend.

FastAPI + pymavlink. Holds COM13, reads MAVLink into a shared state dict, pushes it
to the browser over a WebSocket at 5 Hz, and accepts commands back over the same socket.
Reuses the exact command sequences proven in the standalone scripts.

Run:  & "C:\Users\Serve\anaconda3\python.exe" server.py
Then open http://127.0.0.1:8000  (close Mission Planner / other scripts — COM13 is exclusive)
"""
import asyncio
import math
import threading
import time
from collections import deque
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pymavlink import mavutil
import uvicorn

PORT = "COM13"
BAUD = 57600
HERE = Path(__file__).parent

# ArduCopter custom_mode -> name (vehicle-specific; do NOT reuse PX4 numbers)
MODES = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
         5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 16: "POSHOLD",
         17: "BRAKE", 20: "GUIDED_NOGPS"}

STATE = {
    "link": False, "last_hb": 0.0,
    "armed": False, "mode": "?",
    "lat": None, "lon": None, "fix": 0, "sats": 0, "hdop": 99.9,
    "rel_alt": 0.0, "gspd": 0.0, "hdg": 0.0, "throttle": 0,
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "volt": 0.0, "batt_pct": -1,
}
STATUSTEXT = deque(maxlen=14)
ACKS = deque(maxlen=6)

mav = None
LOCK = threading.Lock()   # serialize MAVLink writes (reader thread reads; commands write)

# Failsafe policy for a transmitter-less airframe: the SiK link is the only lifeline,
# so the radio failsafe is disabled and replaced by the GCS-heartbeat failsafe (LAND).
# The dashboard's HOLD-TO-KILL is the manual stop; FS_GCS is the automatic one.
GCS_HEARTBEAT_HZ = 2      # must stay well under FS_GCS_TIMEOUT (default 5 s)
FS_GCS_ACTION_LAND = 5    # FS_GCS_ENABLE = 5 -> "Enabled Always Land"


def set_failsafe():
    """Configure the FC for no-RC operation: radio failsafe off, GCS failsafe = LAND."""
    with LOCK:
        for name, val in (("FS_THR_ENABLE", 0), ("FS_GCS_ENABLE", FS_GCS_ACTION_LAND)):
            mav.mav.param_set_send(mav.target_system, mav.target_component,
                                   name.encode(), float(val),
                                   mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            time.sleep(0.05)
    print("-- no-RC failsafe set: FS_THR_ENABLE=0, FS_GCS_ENABLE=5 (link-loss=LAND)", flush=True)


def gcs_heartbeat():
    """Send a GCS heartbeat to the vehicle at GCS_HEARTBEAT_HZ. Without this the
    GCS-heartbeat failsafe never activates — and once armed, if these stop for
    FS_GCS_TIMEOUT (~5 s) the FC LANDs. Runs for the life of the server."""
    while True:
        m = mav
        if m is not None:
            try:
                with LOCK:
                    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            except Exception:
                pass
        time.sleep(1.0 / GCS_HEARTBEAT_HZ)


def set_stream_rates():
    """SiK @57600 is bandwidth-limited — request modest, deliberate rates."""
    rates = {
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 5,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 3,
        mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD: 2,
        mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS: 1,
        mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: 2,
    }
    with LOCK:
        for msg_id, hz in rates.items():
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)
            time.sleep(0.05)


def reader():
    """Background thread: parse MAVLink into STATE. Resilient to mav being None/closed."""
    while True:
        m = mav
        if m is None:
            time.sleep(0.3)
            continue
        try:
            msg = m.recv_match(blocking=True, timeout=1)
        except Exception:
            time.sleep(0.3)
            continue
        if msg is None:
            continue
        t = msg.get_type()
        if t == "HEARTBEAT":
            STATE["last_hb"] = time.time()
            STATE["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            STATE["mode"] = MODES.get(msg.custom_mode, str(msg.custom_mode))
        elif t == "GLOBAL_POSITION_INT":
            STATE["lat"] = msg.lat / 1e7
            STATE["lon"] = msg.lon / 1e7
            STATE["rel_alt"] = msg.relative_alt / 1000.0
            STATE["hdg"] = msg.hdg / 100.0
        elif t == "GPS_RAW_INT":
            STATE["fix"] = msg.fix_type
            STATE["sats"] = msg.satellites_visible
            STATE["hdop"] = msg.eph / 100.0 if msg.eph != 65535 else 99.9
        elif t == "SYS_STATUS":
            STATE["volt"] = msg.voltage_battery / 1000.0
            STATE["batt_pct"] = msg.battery_remaining
        elif t == "VFR_HUD":
            STATE["gspd"] = msg.groundspeed
            STATE["throttle"] = msg.throttle
        elif t == "ATTITUDE":
            STATE["roll"] = math.degrees(msg.roll)
            STATE["pitch"] = math.degrees(msg.pitch)
            STATE["yaw"] = math.degrees(msg.yaw)
        elif t == "STATUSTEXT":
            STATUSTEXT.append({"t": round(time.time(), 1), "text": msg.text.strip()})
        elif t == "COMMAND_ACK":
            ACKS.append({"cmd": msg.command, "result": msg.result})


def _arm(value, force=0):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        value, force, 0, 0, 0, 0, 0)


def do_command(c):
    """Dispatch a command dict from the browser. Reuses proven sequences."""
    cmd = c.get("cmd")
    # link management (handle their own locking — not under the command LOCK)
    if cmd == "connect":
        return {"cmd": cmd, "ok": open_link()}
    if cmd == "disconnect":
        close_link()
        return {"cmd": cmd, "ok": True}
    if cmd == "refresh":
        if mav is None:
            return {"cmd": cmd, "ok": False, "msg": "not connected"}
        set_stream_rates()
        return {"cmd": cmd, "ok": True}
    if mav is None:
        return {"cmd": cmd, "ok": False, "msg": "not connected"}
    with LOCK:
        if cmd == "set_mode":
            try:
                mode_id = mav.mode_mapping()[c["mode"]]
            except KeyError:
                return {"cmd": cmd, "ok": False, "msg": f"unknown mode {c.get('mode')}"}
            mav.set_mode(mode_id)
        elif cmd == "arm":
            _arm(1)
        elif cmd == "disarm":
            _arm(0)
        elif cmd == "kill":
            # force-disarm (works in flight); spam to beat packet loss over SiK
            for _ in range(6):
                _arm(0, force=21196)
                time.sleep(0.05)
        elif cmd == "takeoff":
            alt = float(c.get("alt", 0.5))
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                0, 0, 0, 0, 0, 0, alt)
        else:
            return {"cmd": cmd, "ok": False, "msg": "unknown command"}
    return {"cmd": cmd, "ok": True}


app = FastAPI()


def rate_setter():
    """Wait for the first heartbeat (target_system known) then request stream rates."""
    while mav is not None and STATE["last_hb"] == 0.0:
        time.sleep(0.5)
    if mav is not None:
        set_stream_rates()
        set_failsafe()
        print(f"-- heartbeat from sys={mav.target_system}; stream rates + no-RC failsafe set", flush=True)


def open_link():
    """(Re)open COM13. The single reader thread picks up the new connection."""
    global mav
    with LOCK:
        if mav is not None:
            try:
                mav.close()
            except Exception:
                pass
        try:
            mav = mavutil.mavlink_connection(PORT, baud=BAUD)
        except Exception as e:
            mav = None
            print(f"!! could not open {PORT}: {e}", flush=True)
            return False
    STATE["last_hb"] = 0.0
    threading.Thread(target=rate_setter, daemon=True).start()
    print(f"-- opened {PORT}", flush=True)
    return True


def close_link():
    """Close COM13 and release it (the web server keeps running)."""
    global mav
    with LOCK:
        if mav is not None:
            try:
                mav.close()
            except Exception:
                pass
            mav = None
    STATE["last_hb"] = 0.0
    print(f"-- closed {PORT}", flush=True)


@app.on_event("startup")
def startup():
    """Serve immediately; open the link in the background (UI shows LINK LOST until heartbeat)."""
    print(f"opening {PORT}@{BAUD}...", flush=True)
    threading.Thread(target=reader, daemon=True).start()
    threading.Thread(target=gcs_heartbeat, daemon=True).start()   # no-RC lifeline
    open_link()
    print("-- serving http://127.0.0.1:8090  (telemetry fills in once the drone is on)", flush=True)


@app.get("/")
def index():
    return FileResponse(HERE / "static" / "index.html")


@app.websocket("/ws")
async def ws(websocket: WebSocket):
    await websocket.accept()

    async def sender():
        while True:
            STATE["link"] = (time.time() - STATE["last_hb"]) < 3.0
            payload = dict(STATE)
            payload["statustext"] = list(STATUSTEXT)
            payload["acks"] = list(ACKS)
            await websocket.send_json(payload)
            await asyncio.sleep(0.2)   # 5 Hz

    async def receiver():
        loop = asyncio.get_event_loop()
        while True:
            c = await websocket.receive_json()
            # serial write off the event loop
            res = await loop.run_in_executor(None, do_command, c)
            await websocket.send_json({"ack": res})

    try:
        await asyncio.gather(sender(), receiver())
    except WebSocketDisconnect:
        pass


app.mount("/static", StaticFiles(directory=HERE / "static"), name="static")


if __name__ == "__main__":
    uvicorn.run(app, host="127.0.0.1", port=8090, log_level="warning")
