#!/usr/bin/env python3
r"""X500 motor ORDER + DIRECTION test (ArduCopter/pymavlink, Quad-X).

Spins ONE motor at a time at low throttle via MAV_CMD_DO_MOTOR_TEST so you can verify
each motor is in the right position and spinning the right way. No arming needed; the
firmware auto-stops each motor at the timeout.

SAFETY — read before running:
  * RESTRAIN THE FRAME (clamp/strap it down) — a wrong-direction motor can drag it.
  * Keep hands / cables / faces OUT of every prop arc. Eye protection on.
  * Throttle is capped low; still, keep a way to cut power within reach.
  * Runs the props at low throttle. If you can, verify direction with PROPS OFF.

Usage:
  python x500_motor_direction.py                 # /dev/ttyUSB0, 8% throttle
  python x500_motor_direction.py /dev/ttyUSB0 8

ArduCopter Quad-X test order is CLOCKWISE starting front-right. Expected for each test:
  seq 1  Front-Right  CCW
  seq 2  Rear-Right   CW
  seq 3  Rear-Left    CCW
  seq 4  Front-Left   CW
If a DIFFERENT motor spins than stated, the output mapping is wrong. If the DIRECTION is
wrong (PWM ESCs on this build): swap any two of that motor's three wires.
"""
import sys
import threading
import time
from pymavlink import mavutil

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
THROTTLE_PCT = int(sys.argv[2]) if len(sys.argv) > 2 else 8   # keep < 10
BAUD = 57600
SPIN_SECS = 6           # long enough to check BOTH direction and airflow
THROTTLE_TYPE_PERCENT = 0

# test-sequence -> (physical position, expected spin direction)
SEQ = {1: ("Front-Right", "CCW"),
       2: ("Rear-Right",  "CW"),
       3: ("Rear-Left",   "CCW"),
       4: ("Front-Left",  "CW")}

ACK = {0: "ACCEPTED", 1: "TEMP_REJECTED", 2: "DENIED", 3: "UNSUPPORTED",
       4: "FAILED", 5: "IN_PROGRESS", 6: "CANCELLED"}

_hb_run = True

def heartbeat_thread(m):
    while _hb_run:
        try:
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        except Exception:
            pass
        time.sleep(0.5)


def spin(m, seq):
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
        seq,                     # p1: motor sequence number (1..4, clockwise test order)
        THROTTLE_TYPE_PERCENT,   # p2: throttle type = percent
        THROTTLE_PCT,            # p3: throttle %
        SPIN_SECS,               # p4: timeout (s)
        0, 0, 0)                 # p5 count=0 (single), p6, p7
    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    return ACK.get(ack.result, f"result={ack.result}") if ack else "no ACK"


def main():
    global _hb_run
    if THROTTLE_PCT > 15:
        print(f"refusing throttle {THROTTLE_PCT}% > 15% for a props-on single-motor test.", flush=True)
        sys.exit(1)
    print(f"connecting {PORT}@{BAUD}...", flush=True)
    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    if m.wait_heartbeat(timeout=20) is None:
        print("NO HEARTBEAT — port free? drone on battery?", flush=True); sys.exit(1)
    print(f"-- connected sys={m.target_system} fw=ArduPilot", flush=True)
    threading.Thread(target=heartbeat_thread, args=(m,), daemon=True).start()

    print(f"\n!! PROPS-ON MOTOR TEST @ {THROTTLE_PCT}% — FRAME RESTRAINED, PROP ARC CLEAR")
    input("Press ENTER when the frame is secured and everyone is clear... ")

    print("For AIRFLOW: hold a light tissue/paper just ABOVE the prop (never in the disc).")
    print("Correct = air pushed DOWN (tissue pushed away/down). Inverted prop = air pulled UP.\n")
    results = {}
    try:
        for seq in (1, 2, 3, 4):
            pos, direction = SEQ[seq]
            input(f"\n  --> ENTER to spin test motor {seq}: expect **{pos}**, turning **{direction}**, "
                  f"blowing air DOWN ")
            res = spin(m, seq)
            print(f"      command: {res}; spinning ~{SPIN_SECS}s — check DIRECTION and AIRFLOW", flush=True)
            time.sleep(SPIN_SECS + 0.5)
            dir_ok = input(f"      Did the **{pos}** motor spin **{direction}**? (y/n) ").strip().lower() == "y"
            air = input(f"      Airflow: DOWN (correct) or UP (inverted prop)?  (d/u) ").strip().lower()
            results[seq] = (pos, direction, dir_ok, air.startswith("d"))
    finally:
        _hb_run = False
        time.sleep(0.2)

    print("\n=== RESULT ===")
    dir_bad = [s for s, (_, _, d, _) in results.items() if not d]
    air_bad = [s for s, (_, _, _, a) in results.items() if not a]
    for seq, (pos, direction, d, a) in results.items():
        flags = []
        if not d: flags.append("WRONG DIRECTION")
        if not a: flags.append("INVERTED PROP (air up)")
        print(f"  motor {seq} {pos:<12} {direction:<4} {'OK' if not flags else '<-- ' + ', '.join(flags)}")
    if not dir_bad and not air_bad:
        print("\nAll four correct (direction + airflow down).")
    if air_bad:
        print(f"\n>>> {len(air_bad)} INVERTED PROP(S) — this is the topple cause. Flip that prop over "
              f"(the CW/CCW prop pair are different parts; make sure each corner has the right one, "
              f"lettering up, curved edge scooping air down).")
    if dir_bad:
        print(f"\n>>> {len(dir_bad)} wrong DIRECTION — PWM ESCs: swap any two of that motor's 3 wires; "
              f"wrong POSITION: check ESC->output wiring.")
    m.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        _hb_run = False
        print("\nstopped.", flush=True)
