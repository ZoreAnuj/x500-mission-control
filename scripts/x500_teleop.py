#!/usr/bin/env python3
r"""Setpoint-nudge teleop — keys move a LATCHED position+heading target; the FC's own
400 Hz loops close every axis. Replaces rate/velocity teleop, which limit-cycled: with
~0.9 s command->response lag, bang-bang vz/yaw_rate/vx keys made the human the relay in
a relay-with-delay oscillator (measured on run_01: 3.2 s altitude cycle, 97% of vz
commands opposing altitude error; drone coasted 0.1-0.4 m/s whenever keys were released).

Position+yaw targets are LATCHED by ArduCopter: between keypresses the vehicle
station-keeps against wind. Momentum, lag, and drift are the FC's problem, not yours.

Keys (tap, don't hold):
  w / s   forward / back   0.5 m   (along current TARGET heading)
  a / d   left / right     0.5 m
  r / f   up / down        0.25 m
  q / e   yaw left / right 15 deg
  SPACE   brake: target <- current position + heading
  l       LAND (and exit)
The target is clamped to a 15 m radius / 0.5-4.5 m altitude box around takeoff.

  python x500_teleop.py                      # Linux GCS (/dev/ttyUSB0)
  python x500_teleop.py --connect COM13      # Windows
  python x500_teleop.py --connect tcp:127.0.0.1:5760 --force --alt 1.5   # SITL (in WSL)
"""
import argparse
import csv
import math
import sys
import time

import numpy as np
from pymavlink import mavutil

from field_replay import (init_st, connect, set_failsafe_norc, set_stream_rates,
                          poll, beat, preflight_gates, set_mode, wait_ekf_ready,
                          arm, takeoff, land_and_disarm, wrap)

STEP_XY = 0.5
STEP_Z = 0.25
STEP_YAW = math.radians(15)
BOX_RADIUS = 15.0
ALT_MIN, ALT_MAX = 0.5, 4.5
SEND_HZ = 5.0            # latched anyway; resend for link robustness

# type_mask: set bit = IGNORE. Use pos(0,1,2) + yaw(10); ignore vel/accel/force/yaw_rate.
USE_POS_YAW = (7 << 3) | (7 << 6) | (1 << 9) | (1 << 11)      # 3064


# ---- single-keypress input, Windows + Linux ----
if sys.platform == "win32":
    import msvcrt

    def getkey():
        if msvcrt.kbhit():
            ch = msvcrt.getwch()
            return ch
        return None
else:
    import select
    import termios
    import tty

    class _Raw:
        def __enter__(self):
            self.fd = sys.stdin.fileno()
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            return self

        def __exit__(self, *a):
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    _raw_ctx = None

    def getkey():
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None


def send_pos_yaw(m, tgt, yaw):
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, USE_POS_YAW,
        tgt[0], tgt[1], tgt[2], 0, 0, 0, 0, 0, 0, yaw, 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--alt", type=float, default=1.5)
    ap.add_argument("--out", default="teleop_run.csv")
    ap.add_argument("--force", action="store_true", help="skip preflight gates (SITL)")
    a = ap.parse_args()

    st = init_st()
    m = connect(a.connect, a.baud)
    set_stream_rates(m)
    set_failsafe_norc(m)
    if not a.force:
        if not preflight_gates(m, st):
            sys.exit("ABORT: preflight gates failed")
    if not wait_ekf_ready(m, st):
        sys.exit("ABORT: no EKF position")
    if not set_mode(m, st, "GUIDED"):
        sys.exit("GUIDED failed")
    print("\n*** Type GO to arm + takeoff into teleop: ***")
    if input().strip().upper() != "GO":
        sys.exit("aborted")
    if not arm(m, st):
        sys.exit("arm failed")
    if not takeoff(m, st, a.alt):
        land_and_disarm(m, st)
        sys.exit("takeoff failed -> landed")
    for _ in range(15):
        beat(m)
        poll(m, st)
        time.sleep(0.1)
    while st["pos"] is None or st["att"] is None:
        beat(m)
        poll(m, st)
        time.sleep(0.05)

    take = st["pos"].copy()
    tgt = st["pos"].copy()
    tyaw = st["att"][2]
    print(__doc__.split("Keys")[1].split("The target")[0])
    print("-- TELEOP: nudging a latched target; FC holds it between keys. 'l' = land.\n")

    f = open(a.out, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t", "tgt_n", "tgt_e", "tgt_d", "tgt_yaw",
                "p_n", "p_e", "p_d", "yaw", "rel_alt"])
    t0 = time.monotonic()
    nxt = t0
    raw = (None if sys.platform == "win32" or not sys.stdin.isatty()
           else _Raw().__enter__())    # piped stdin (tests) still works via select
    try:
        while True:
            beat(m)
            poll(m, st)
            k = getkey()
            if k:
                cy, sy = math.cos(tyaw), math.sin(tyaw)
                if k == "w":
                    tgt[0] += STEP_XY * cy; tgt[1] += STEP_XY * sy
                elif k == "s":
                    tgt[0] -= STEP_XY * cy; tgt[1] -= STEP_XY * sy
                elif k == "a":
                    tgt[0] += STEP_XY * sy; tgt[1] -= STEP_XY * cy
                elif k == "d":
                    tgt[0] -= STEP_XY * sy; tgt[1] += STEP_XY * cy
                elif k == "r":
                    tgt[2] -= STEP_Z
                elif k == "f":
                    tgt[2] += STEP_Z
                elif k == "q":
                    tyaw = wrap(tyaw - STEP_YAW)
                elif k == "e":
                    tyaw = wrap(tyaw + STEP_YAW)
                elif k == " ":
                    tgt = st["pos"].copy(); tyaw = st["att"][2]
                    print("  [brake] target <- here")
                elif k in ("l", "x", "\x1b"):
                    print("  [land]")
                    break
                # clamp target to the safety box (the target simply can't leave it)
                dn, de = tgt[0] - take[0], tgt[1] - take[1]
                r = math.hypot(dn, de)
                if r > BOX_RADIUS:
                    tgt[0] = take[0] + dn * BOX_RADIUS / r
                    tgt[1] = take[1] + de * BOX_RADIUS / r
                tgt[2] = float(np.clip(tgt[2], take[2] - (ALT_MAX - a.alt), take[2] + (a.alt - ALT_MIN)))
                print(f"  tgt N{tgt[0]:+.2f} E{tgt[1]:+.2f} alt{-(tgt[2]-take[2])+a.alt:.2f} "
                      f"yaw{math.degrees(tyaw):+.0f}  |  pos N{st['pos'][0]:+.2f} E{st['pos'][1]:+.2f}")
            send_pos_yaw(m, tgt, tyaw)
            w.writerow([round(time.monotonic() - t0, 3), *np.round(tgt, 3), round(tyaw, 4),
                        *np.round(st["pos"], 3), round(st["att"][2], 4), round(st["rel_alt"], 2)])
            nxt += 1.0 / SEND_HZ
            dtw = nxt - time.monotonic()
            if dtw > 0:
                time.sleep(dtw)
            else:
                nxt = time.monotonic()
    except KeyboardInterrupt:
        print("\n  [ctrl-c] -> land")
    finally:
        if raw:
            raw.__exit__()
        f.close()
        land_and_disarm(m, st)


if __name__ == "__main__":
    main()
