#!/usr/bin/env python3
r"""Compass (magnetometer) calibration for the X500 over the SiK radio (COM13).

Triggers ArduCopter's onboard mag cal, then prints live progress while YOU rotate
the drone through all orientations. Auto-saves when complete and reboots.

Rotate the drone slowly so every side (nose, tail, each wing tip, top, bottom)
points straight down at the ground in turn — keep turning until it hits 100%.
"""
import sys
import threading
import time
from pymavlink import mavutil

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"   # COM13 on Windows
BAUD = 57600

_hb_run = True

def heartbeat_thread(m):
    """Keep the no-RC GCS-heartbeat failsafe (FS_GCS) satisfied during cal."""
    while _hb_run:
        try:
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        except Exception:
            pass
        time.sleep(0.5)


def main():
    print(f"connecting {PORT}@{BAUD}...", flush=True)
    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    if m.wait_heartbeat(timeout=20) is None:
        print("NO HEARTBEAT", flush=True); sys.exit(1)
    print("-- connected", flush=True)
    threading.Thread(target=heartbeat_thread, args=(m,), daemon=True).start()

    # Start magnetometer calibration on all compasses
    print("starting compass calibration...", flush=True)
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL, 0,
        0,    # mag mask 0 = all compasses
        0,    # retry
        1,    # autosave
        0, 0, 0, 0)

    print("\n>>> ROTATE THE DRONE NOW — turn it so each face points at the ground.\n", flush=True)

    done = set()
    t0 = time.time()
    while time.time() - t0 < 180:
        msg = m.recv_match(type=["MAG_CAL_PROGRESS", "MAG_CAL_REPORT", "STATUSTEXT"],
                           blocking=True, timeout=5)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "MAG_CAL_PROGRESS":
            bar = "#" * (msg.completion_pct // 5)
            print(f"  compass {msg.compass_id}: {msg.completion_pct:3d}% [{bar:<20}]",
                  end="\r", flush=True)
        elif t == "MAG_CAL_REPORT":
            ok = msg.cal_status == mavutil.mavlink.MAG_CAL_SUCCESS
            print(f"\n  compass {msg.compass_id}: "
                  f"{'SUCCESS' if ok else f'status={msg.cal_status}'} "
                  f"fitness={msg.fitness:.1f}", flush=True)
            done.add(msg.compass_id)
            if len(done) >= 1 and ok:
                # at least primary compass done — good enough; keep collecting briefly
                pass
        elif t == "STATUSTEXT":
            if "Compass" in msg.text or "Mag" in msg.text or "calib" in msg.text.lower():
                print(f"\n  FC: {msg.text.strip()}", flush=True)
                if "calibration successful" in msg.text.lower() or "rebooting" in msg.text.lower():
                    break

    print("\n-- calibration done. Rebooting to apply...", flush=True)
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
    print("-- done.", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopped.", flush=True)
