#!/usr/bin/env python3
r"""X500 accelerometer 6-point calibration + level trim (ArduPilot/pymavlink).

Interactive: the flight controller asks for each orientation; you physically hold the
drone in that position, press ENTER, and this script tells the FC "I'm in position".
No RC transmitter needed. PROPS OFF. Close Mission Planner / the mission-control server
first (the COM port is exclusive).

Because the vehicle is configured for no-RC operation (FS_GCS_ENABLE=5), this script
streams GCS heartbeats in the background so it doesn't trip the "GCS failsafe on" nag.

Usage:
  python x500_accel_cal.py                 # default port /dev/ttyUSB0
  python x500_accel_cal.py /dev/ttyUSB0    # or COM13 on Windows

The six positions (hold each still until you've pressed ENTER):
  1 LEVEL      — flat, right-side up
  2 LEFT       — resting on its left side
  3 RIGHT      — resting on its right side
  4 NOSE DOWN  — front pointing at the floor
  5 NOSE UP    — front pointing at the ceiling
  6 BACK       — upside down
Then optionally LEVEL trim (leave it flat and level). A reboot is required afterwards.
"""
import sys
import threading
import time
from pymavlink import mavutil

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = 57600

POS_NAME = {1: "LEVEL", 2: "LEFT side", 3: "RIGHT side",
            4: "NOSE DOWN", 5: "NOSE UP", 6: "on its BACK (upside down)"}

_lock = threading.Lock()      # serialize MAVLink writes (heartbeat thread vs main)
_hb_run = True


def heartbeat_thread(m):
    """Keep the FC's GCS-heartbeat failsafe satisfied during the interactive cal."""
    while _hb_run:
        try:
            with _lock:
                m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                     mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        except Exception:
            pass
        time.sleep(0.5)


def cmd(m, *args):
    with _lock:
        m.mav.command_long_send(m.target_system, m.target_component, *args)


def pos_from_text(t):
    """Map an ArduPilot 'Place vehicle ...' prompt to an ACCELCAL_VEHICLE_POS value."""
    t = t.lower()
    if "level" in t: return 1
    if "left" in t:  return 2
    if "right" in t: return 3
    if "down" in t:  return 4   # "nose down"
    if "up" in t:    return 5   # "nose up"
    if "back" in t:  return 6
    return None


def start_cal(m):
    cmd(m, mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
        0, 0, 0, 0, 1, 0, 0)   # param5=1 -> accelerometer 6-point


def drain(m, secs):
    """Read+print FC STATUSTEXT for `secs`. Return True/False if a cal result appears."""
    t0 = time.time()
    while time.time() - t0 < secs:
        msg = m.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
        if msg is None:
            continue
        text = msg.text.strip()
        low = text.lower()
        print(f"  FC: {text}", flush=True)
        if "calibration successful" in low or ("accel" in low and "success" in low):
            return True
        if "calibration failed" in low or ("accel" in low and "fail" in low and "failsafe" not in low):
            return False
    return None


def wait_start(m, timeout=15):
    """Kick off the cal. Return True on the first 'Place vehicle level' prompt OR on an
    accepted COMMAND_ACK (the prompt is often dropped over SiK, so the ack is enough)."""
    start_cal(m)
    last = time.time()
    t0 = time.time()
    acked = None
    while time.time() - t0 < timeout:
        msg = m.recv_match(type=["STATUSTEXT", "COMMAND_ACK"], blocking=True, timeout=2)
        if msg is None:
            if acked and time.time() - acked > 4:
                print("  (no 'level' prompt over the radio; proceeding on ack=accepted)", flush=True)
                return True
            if time.time() - last > 6:
                start_cal(m); last = time.time()
            continue
        if msg.get_type() == "COMMAND_ACK":
            if msg.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION:
                print(f"  [ack] calibration start result={msg.result} (0=accepted,5=in-progress)",
                      flush=True)
                if msg.result in (0, 5) and acked is None:
                    acked = time.time()
            continue
        print(f"  FC: {msg.text.strip()}", flush=True)
        if "place vehicle" in msg.text.lower():
            return True
    return acked is not None


def accel_cal(m):
    print("\n=== ACCEL 6-POINT CALIBRATION ===  (PROPS OFF)\n", flush=True)
    print("(driving the 6 positions in fixed order — SiK drops the FC's prompts,\n"
          " so we don't wait for them; each position is confirmed by your ENTER)\n", flush=True)
    if not wait_start(m):
        print(">>> calibration did not start (no prompt/ack) — check the link", flush=True)
        return False

    # Fixed ArduPilot order: level, left, right, nose-down, nose-up, back.
    for pos in range(1, 7):
        input(f"\n  Position {pos}/6: place the drone **{POS_NAME[pos]}**, hold still, "
              f"then press ENTER... ")
        for _ in range(3):                     # spam to beat SiK packet loss
            cmd(m, mavutil.mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS, 0, pos, 0, 0, 0, 0, 0, 0)
            time.sleep(0.2)
        print(f"  -> reported position {pos} ({POS_NAME[pos]}); sampling...", flush=True)
        res = drain(m, 3)                      # show any FC chatter / early result
        if res is not None:
            print(f"\n>>> ACCEL CAL {'SUCCESS' if res else 'FAILED'}", flush=True)
            return res

    print("\n  all 6 positions sent; waiting for the FC's result...", flush=True)
    res = drain(m, 20)
    if res is None:
        print(">>> no explicit result seen — verify via a param read (B4)", flush=True)
        return False
    print(f"\n>>> ACCEL CAL {'SUCCESS' if res else 'FAILED'}", flush=True)
    return res


def level_trim(m):
    ans = input("\nDo LEVEL trim now? Leave the drone flat & level on a surface. (y/N) ")
    if ans.strip().lower() != "y":
        print("  skipped level trim.", flush=True)
        return
    cmd(m, mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
        0, 0, 0, 0, 2, 0, 0)   # param5=2 -> board level / accel trim
    t0 = time.time()
    while time.time() - t0 < 5:
        msg = m.recv_match(type=["STATUSTEXT", "COMMAND_ACK"], blocking=True, timeout=2)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            print(f"  FC: {msg.text.strip()}", flush=True)
        elif msg.get_type() == "COMMAND_ACK":
            print(f"  level-trim ack result={msg.result}", flush=True)
            break
    print("  level trim sent (sets AHRS_TRIM_X/Y).", flush=True)


def main():
    global _hb_run
    print(f"connecting {PORT}@{BAUD}...", flush=True)
    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    if m.wait_heartbeat(timeout=20) is None:
        print("NO HEARTBEAT — is the port free and the drone on?", flush=True)
        sys.exit(1)
    print(f"-- connected sys={m.target_system} fw=ArduPilot", flush=True)
    threading.Thread(target=heartbeat_thread, args=(m,), daemon=True).start()

    try:
        ok = accel_cal(m)
        if ok:
            level_trim(m)
            print("\n-- calibration done. A REBOOT is required before arming.", flush=True)
            if input("Reboot the flight controller now? (y/N) ").strip().lower() == "y":
                cmd(m, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
                    1, 0, 0, 0, 0, 0, 0)
                print("-- reboot command sent.", flush=True)
    finally:
        _hb_run = False
        time.sleep(0.2)
        m.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        _hb_run = False
        print("\nstopped.", flush=True)
