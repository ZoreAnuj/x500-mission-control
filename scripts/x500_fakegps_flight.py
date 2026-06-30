#!/usr/bin/env python3
r"""X500 indoor autonomous flight via FAKE GPS injection (no real GPS, no extra sensors).

Streams synthetic GPS_INPUT (#232) so the EKF believes it has a 3D fix, auto-sets
origin/home, and the normal GUIDED -> arm -> NAV_TAKEOFF -> hover -> LAND flow works.

Props ON. Close Mission Planner. RC on with kill switch (SF) ready.

SAFETY: indoors there is NO real position feedback — the copter holds against a FIXED
fake position, so it will NOT fight drift and the EKF can diverge. Tether/cage it.

Usage:  python x500_fakegps_flight.py
"""
import sys
import time
import math
import threading
from pymavlink import mavutil

PORT       = "COM13"
BAUD       = 57600
TARGET_ALT = 0.5      # metres AGL
HOVER_SECS = 5

# Fake fix location — any valid WGS84 coord works (College Park, MD).
FAKE_LAT = 38.9897    # deg
FAKE_LON = -76.9378   # deg
FAKE_ALT = 50.0       # m MSL

_stream = True            # background GPS thread run flag
_lock = threading.Lock()  # serialize concurrent MAVLink writes (gps thread vs main)


def connect(timeout=30):
    print(f"connecting {PORT}@{BAUD}...", flush=True)
    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    if m.wait_heartbeat(timeout=timeout) is None:
        print("NO HEARTBEAT", flush=True); sys.exit(1)
    print(f"-- connected sys={m.target_system} fw=ArduPilot", flush=True)
    return m


def set_param(m, name, value, ptype=mavutil.mavlink.MAV_PARAM_TYPE_INT32):
    m.mav.param_set_send(m.target_system, m.target_component,
                         name.encode(), float(value), ptype)
    time.sleep(0.3)


def save_params(m):
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 0, 1, 0, 0, 0, 0, 0, 0)
    time.sleep(1.5)


def reboot(m):
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)


def cmd(m, *args):
    """Locked command_long_send (safe vs the GPS thread's writes)."""
    with _lock:
        m.mav.command_long_send(m.target_system, m.target_component, *args)

def set_mode(m, mode_name):
    mode_id = m.mode_mapping()[mode_name]
    with _lock:
        m.mav.set_mode_send(m.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    for _ in range(25):
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and mavutil.mode_string_v10(hb) == mode_name:
            return True
    return False


def gps_thread(m):
    """Stream GPS_INPUT at ~5 Hz until _stream goes False."""
    lat = int(FAKE_LAT * 1e7)
    lon = int(FAKE_LON * 1e7)
    epoch = 315964800  # GPS epoch 1980-01-06 in unix seconds
    while _stream:
        t_us = int(time.time() * 1e6)
        gps_s = time.time() - epoch
        week = int(gps_s // (7 * 86400))
        week_ms = int((gps_s % (7 * 86400)) * 1000)
        with _lock:
            m.mav.gps_input_send(
                t_us, 0,            # time_usec, gps_id
                0,                  # ignore_flags = 0 (we supply everything)
                week_ms, week,
                3,                  # fix_type = 3D
                lat, lon, FAKE_ALT,
                0.6, 0.8,           # hdop, vdop
                0.0, 0.0, 0.0,      # vn, ve, vd  (stationary)
                0.2, 0.5, 0.5,      # speed/horiz/vert accuracy
                15,                 # satellites_visible
                0)                  # yaw (0 = not available)
        time.sleep(0.2)


def disarm(m):
    cmd(m, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)


def main():
    global _stream
    m = connect()

    # Enable MAVLink GPS backend, reboot so it takes effect.
    print("setting GPS1_TYPE=14 (MAVLink GPS), rebooting...", flush=True)
    set_param(m, "GPS1_TYPE", 14)
    set_param(m, "ARMING_CHECK", 0)   # bench/indoor — skip remaining pre-arm gates
    save_params(m)
    reboot(m)
    m.close()
    time.sleep(20)
    m = connect()

    # Ask the FC to stream GPS_RAW_INT / GLOBAL_POSITION_INT over the radio
    m.mav.request_data_stream_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

    # Start fake-GPS stream
    print("streaming fake GPS_INPUT @5Hz...", flush=True)
    th = threading.Thread(target=gps_thread, args=(m,), daemon=True)
    th.start()

    # Wait for the EKF to accept the fix
    print("waiting for fix_type>=3...", flush=True)
    t0 = time.time()
    while time.time() - t0 < 40:
        msg = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=3)
        if msg:
            print(f"  fix={msg.fix_type} sats={msg.satellites_visible}", flush=True)
            if msg.fix_type >= 3:
                break
    else:
        print("fake GPS not accepted — aborting", flush=True)
        _stream = False; sys.exit(1)
    print("-- fake fix accepted; EKF has position", flush=True)
    print("letting EKF origin/home/variance settle (15s)...", flush=True)
    time.sleep(15)   # static fake fix needs time for position variance to drop

    # GUIDED + arm + takeoff
    print("setting GUIDED...", flush=True)
    if not set_mode(m, "GUIDED"):
        print("GUIDED failed", flush=True); _stream = False; sys.exit(1)
    print("-- GUIDED", flush=True)

    # Arm — retry a few times, print the FC's rejection reason
    armed = False
    for attempt in range(5):
        print(f"arming (attempt {attempt+1})...", flush=True)
        cmd(m, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        t0 = time.time()
        while time.time() - t0 < 5:
            msg = m.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=2)
            if msg is None: continue
            if msg.get_type() == "STATUSTEXT":
                print(f"  FC: {msg.text.strip()}", flush=True)
            elif msg.get_type() == "COMMAND_ACK":
                if msg.result == 0:
                    armed = True
                break
        if armed: break
        time.sleep(3)
    if not armed:
        print("ARM failed after retries", flush=True)
        _stream = False; sys.exit(1)
    print("-- ARMED", flush=True)
    time.sleep(1)

    print(f"takeoff -> {TARGET_ALT}m...", flush=True)
    cmd(m, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, TARGET_ALT)
    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=10)
    if not ack or ack.result != 0:
        print(f"TAKEOFF failed result={ack.result if ack else 'no ACK'}", flush=True)
        disarm(m); _stream = False; sys.exit(1)

    # climb watch
    print("climbing...", flush=True)
    t0 = time.time()
    while time.time() - t0 < 15:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if msg is None: continue
        rel = msg.relative_alt / 1000.0
        print(f"  alt={rel:.1f}m", end="\r", flush=True)
        if rel >= TARGET_ALT - 0.15:
            print(f"\n-- reached {rel:.1f}m", flush=True); break

    print(f"hovering {HOVER_SECS}s...", flush=True)
    time.sleep(HOVER_SECS)

    print("landing...", flush=True)
    cmd(m, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    while time.time() - t0 < 30:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if msg is None: break
        rel = msg.relative_alt / 1000.0
        print(f"  alt={rel:.1f}m", end="\r", flush=True)
        if rel < 0.25: break
    print("\n-- touchdown", flush=True)

    disarm(m)
    print("-- disarmed.", flush=True)

    # restore real GPS for normal ops
    _stream = False
    time.sleep(0.5)
    print("restoring GPS1_TYPE=1 + ARMING_CHECK=1...", flush=True)
    set_param(m, "GPS1_TYPE", 1)
    set_param(m, "ARMING_CHECK", 1)
    save_params(m)
    reboot(m)
    print("done.", flush=True)


if __name__ == "__main__":
    try:
        main()
    finally:
        _stream = False
