#!/usr/bin/env python3
r"""Connect to the X500 over the SiK radio and bench-test ALL motors together.

Spins all 4 motors at the same low throttle at once (via MAV_CMD_DO_MOTOR_TEST, no
arming, firmware auto-stops at the timeout) so you can compare their RPM side by side.

SAFETY:
  * Ideally PROPS OFF. If running PROPS ON for a spin/RPM check, keep throttle <=10%,
    RESTRAIN THE FRAME firmly, clear the prop arc, wear eye protection.
  * Sends GCS heartbeats so the no-RC FS_GCS failsafe stays satisfied.

Usage:
  python x500_motor_test.py                  # /dev/ttyUSB0, 8% throttle
  python x500_motor_test.py /dev/ttyUSB0 8
"""
import sys
import threading
import time
from pymavlink import mavutil

_args = [a for a in sys.argv[1:] if not a.startswith("--")]
PORT = _args[0] if _args else "/dev/ttyUSB0"
BAUD = 57600
N_MOTORS = 4          # X500 = quad
THROTTLE_PCT = int(_args[1]) if len(_args) > 1 else 8   # capped <=10 for props-on
SPIN_SECONDS = 10
THROTTLE_TYPE_PERCENT = 0  # MOTOR_TEST_THROTTLE_PERCENT

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


def main():
    if THROTTLE_PCT > 10:
        print(f"refusing throttle {THROTTLE_PCT}% > 10% for a props-on test.", flush=True)
        sys.exit(1)
    print(f"connecting {PORT}@{BAUD} ...", flush=True)
    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    hb = m.wait_heartbeat(timeout=20)
    if hb is None:
        print("NO HEARTBEAT — is the port free and the radio linked?", flush=True)
        sys.exit(1)
    threading.Thread(target=heartbeat_thread, args=(m,), daemon=True).start()
    ap = {3: "ArduPilot", 12: "PX4"}.get(hb.autopilot, f"autopilot={hb.autopilot}")
    print(f"-- connected: sys={m.target_system} comp={m.target_component} fw={ap} type={hb.type}", flush=True)

    gps = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=5)
    if gps:
        print(f"-- GPS fix={gps.fix_type} sats={gps.satellites_visible}", flush=True)
    batt = m.recv_match(type="SYS_STATUS", blocking=True, timeout=5)
    if batt:
        print(f"-- batt {batt.voltage_battery / 1000:.2f}V  {batt.battery_remaining}%", flush=True)

    print(f"\n!! PROPS-OFF MOTOR TEST: {N_MOTORS} motors @ {THROTTLE_PCT}% for {SPIN_SECONDS}s", flush=True)
    # Fire one DO_MOTOR_TEST per motor, back-to-back, each with a SPIN_SECONDS timeout,
    # so all motors are spinning together for the window (works on PX4; ArduPilot may
    # serialize — if so, say the word and I'll switch to the sequence form).
    for motor in range(1, N_MOTORS + 1):
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
            motor,                  # p1: motor instance (1..N)
            THROTTLE_TYPE_PERCENT,  # p2: throttle type = percent
            THROTTLE_PCT,           # p3: throttle %
            SPIN_SECONDS,           # p4: timeout (s)
            0,                      # p5: motor count (0/1 = just this motor)
            0,                      # p6: test order (default)
            0)
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
        label = ACK.get(ack.result, f"result={ack.result}") if ack else "no ACK"
        print(f"   motor {motor}: {label}", flush=True)

    for s in range(SPIN_SECONDS, 0, -1):
        print(f"   spinning... {s}s remaining ", end="\r", flush=True)
        time.sleep(1)
    global _hb_run
    _hb_run = False
    print("\n-- done; motors auto-stopped at timeout.", flush=True)


if __name__ == "__main__":
    main()
