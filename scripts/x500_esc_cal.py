#!/usr/bin/env python3
r"""X500 ESC calibration (PWM ESCs, no RC) via the ESC_CALIBRATION parameter.

Equalises all four ESCs' throttle endpoints so equal throttle -> equal RPM.

  * PROPS MUST BE OFF — ESC calibration drives every motor to FULL THROTTLE.
  * The FC must boot from the BATTERY (that's what powers the ESCs).

Two steps:
  1)  python x500_esc_cal.py           # confirm props off -> sets ESC_CALIBRATION=3
      Then POWER-CYCLE the battery (props off). On boot the FC auto-calibrates:
      you'll hear the ESC tones — a musical tone, then N beeps (cell count),
      then one long tone = done. Motors may twitch. ESC_CALIBRATION resets to 0.
  2)  python x500_esc_cal.py --verify  # after the power-cycle, confirm it reset to 0

Usage:
  python x500_esc_cal.py [/dev/ttyUSB0] [--verify]
"""
import sys
import threading
import time
from pymavlink import mavutil

args = [a for a in sys.argv[1:] if not a.startswith("--")]
PORT = args[0] if args else "/dev/ttyUSB0"
BAUD = 57600
VERIFY = "--verify" in sys.argv

_hb_run = True

def heartbeat_thread(m):
    while _hb_run:
        try:
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        except Exception:
            pass
        time.sleep(0.5)


def read_param(m, name, timeout=6):
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        if msg and msg.param_id == name:
            return msg.param_value
    return None


def main():
    global _hb_run
    print(f"connecting {PORT}@{BAUD}...", flush=True)
    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    if m.wait_heartbeat(timeout=20) is None:
        print("NO HEARTBEAT — port free? on battery?", flush=True); sys.exit(1)
    print(f"-- connected sys={m.target_system}", flush=True)
    threading.Thread(target=heartbeat_thread, args=(m,), daemon=True).start()

    if VERIFY:
        v = read_param(m, "ESC_CALIBRATION")
        print(f"\nESC_CALIBRATION = {int(v) if v is not None else '?'} "
              f"({'reset to 0 -> calibration completed' if v == 0 else 'NOT 0 — cal may not have run'})",
              flush=True)
        _hb_run = False; time.sleep(0.2); m.close(); return

    print("\n" + "="*60)
    print("  ESC CALIBRATION — motors will go to FULL THROTTLE on boot.")
    print("  ALL FOUR PROPELLERS MUST BE OFF.")
    print("="*60)
    if input('\nType exactly  PROPS OFF  to proceed: ').strip() != "PROPS OFF":
        print("aborted — remove props and rerun.", flush=True)
        _hb_run = False; time.sleep(0.2); m.close(); return

    m.mav.param_set_send(m.target_system, m.target_component, b"ESC_CALIBRATION",
                         3.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(0.5)
    v = read_param(m, "ESC_CALIBRATION")
    print(f"-- ESC_CALIBRATION set to {int(v) if v is not None else '?'} (want 3)", flush=True)
    print("\nNEXT (props still OFF):", flush=True)
    print("  1. Disconnect the flight battery, wait ~3 s.", flush=True)
    print("  2. Reconnect the battery.", flush=True)
    print("  3. Listen: musical tone -> N beeps (cell count) -> one LONG tone = done.", flush=True)
    print("  4. Then run:  python x500_esc_cal.py --verify", flush=True)
    _hb_run = False; time.sleep(0.2); m.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        _hb_run = False
        print("\nstopped.", flush=True)
