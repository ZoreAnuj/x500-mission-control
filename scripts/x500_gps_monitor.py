#!/usr/bin/env python3
r"""Live GPS monitor for the X500 over the SiK radio (COM13).

Run it, take the drone outside, and watch the line update. It flags the moment a
real 3D fix appears (fix_type>=3) and again when it's good enough to fly (>=6 sats,
HDOP<=2.0). Close Mission Planner first. Ctrl+C to quit.
"""
import sys
import time
from pymavlink import mavutil

PORT = "COM13"
BAUD = 57600
FIX_NAMES = {0: "no-GPS", 1: "no-fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK-float", 6: "RTK-fixed"}


def main():
    print(f"connecting {PORT}@{BAUD}...", flush=True)
    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    if m.wait_heartbeat(timeout=20) is None:
        print("NO HEARTBEAT — Mission Planner closed? radio linked?", flush=True)
        sys.exit(1)
    print("-- connected. Requesting GPS stream...", flush=True)
    m.mav.request_data_stream_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 2, 1)

    print("\nTake the drone outside. Watching GPS...\n", flush=True)
    announced_3d = False
    announced_ready = False

    while True:
        msg = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=5)
        if msg is None:
            print("  (no GPS message — link slow/dropped)", flush=True)
            continue
        fix = msg.fix_type
        sats = msg.satellites_visible
        hdop = msg.eph / 100.0 if msg.eph != 65535 else 99.9
        name = FIX_NAMES.get(fix, str(fix))
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7

        line = f"fix={name:9s} sats={sats:2d} hdop={hdop:4.1f} lat={lat:.6f} lon={lon:.6f}"
        print("  " + line, end="\r", flush=True)

        if fix >= 3 and not announced_3d:
            announced_3d = True
            print(f"\n>>> 3D FIX ACQUIRED — {line}", flush=True)
        if fix >= 3 and sats >= 6 and hdop <= 2.0 and not announced_ready:
            announced_ready = True
            print(f"\n>>> READY TO FLY — {sats} sats, HDOP {hdop:.1f}. LOITER will hold position.", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopped.", flush=True)
