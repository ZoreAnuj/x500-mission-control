#!/usr/bin/env python3
r"""X500 keyboard teleop + WORLD-COORDINATE trajectory record / replay.  (local only — gitignored)

TELEOP : fly with the keyboard (GUIDED body-frame velocities) and log the actual
         GPS/EKF world trajectory (lat/lon/rel_alt/yaw) at 10 Hz.
REPLAY : fly the drone through the exact recorded world coordinates via
         SET_POSITION_TARGET_GLOBAL_INT, so it repeats the teleoped motion (closed-loop
         on position — it does NOT drift like re-sent velocities would).

Keys (the pygame window must have focus):
  W / S            up / down
  UP / DOWN arrow  forward / back
  LEFT / RIGHT     yaw left / right
  SPACE            hover (zero all)
  L                land now
  ESC / Q          quit -> land

  # teleop + record (real flight):
  python x500_teleop.py --connect /dev/ttyUSB0 --alt 2 --out teleop_run1.csv
  # replay that log (follows the same world coordinates):
  python x500_teleop.py --replay teleop_run1.csv --connect /dev/ttyUSB0

Body velocities auto-hover if the script/link dies (GUID_TIMEOUT ~3 s). No-RC failsafe
(FS_GCS=LAND) + preflight gates enforced. ENTER (terminal) also lands, any time.
"""
import argparse
import csv
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
from pymavlink import mavutil

from field_replay import (STOP, stop_watcher, init_st, connect, set_failsafe_norc,
                          set_stream_rates, poll, beat, preflight_gates, set_mode,
                          wait_ekf_ready, arm, takeoff, land_and_disarm, set_param)
from cv_hoop_pass import send_body_vel, flight_guard_failures

HZ = 10.0
KP_ALT = 1.5                  # altitude-hold gain (m/s per m of error) -> tight vertical hold
REC_HDR = ["t", "vx", "vz", "yaw_rate", "lat", "lon", "rel_alt", "p_n", "p_e", "p_d", "yaw"]
# GLOBAL_INT replay setpoint: use position + yaw; ignore vel(3,4,5) accel(6,7,8) force(9) yawrate(11)
POS_YAW_MASK = (7 << 3) | (7 << 6) | (1 << 9) | (1 << 11)   # = 3064

CAM_W, CAM_H = 320, 240        # ESP32 QVGA, per esp32cam_policy_stream.ino


def derive_cam_out(csv_path, explicit=None):
    """Video output path: explicit override, or --out with its extension swapped to .mp4."""
    if explicit:
        return explicit
    root, _ = os.path.splitext(csv_path)
    return root + ".mp4"


def frame_to_surface(frame_bgr, scale=2):
    """BGR numpy frame (H,W,3) -> scaled pygame Surface (RGB), ready to blit."""
    import pygame
    rgb = frame_bgr[:, :, ::-1]
    surf = pygame.surfarray.make_surface(np.transpose(rgb, (1, 0, 2)))
    if scale != 1:
        surf = pygame.transform.scale(surf, (surf.get_width() * scale, surf.get_height() * scale))
    return surf


def send_global_pos(m, lat, lon, rel_alt, yaw):
    m.mav.set_position_target_global_int_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, POS_YAW_MASK,
        int(lat * 1e7), int(lon * 1e7), rel_alt, 0, 0, 0, 0, 0, 0, yaw, 0)


def teleop(m, st, a, take):
    import pygame
    pygame.init()
    pygame.display.set_caption("X500 teleop  (focus this window)")
    screen = pygame.display.set_mode((540, 210))
    font = pygame.font.SysFont("monospace", 18)
    f = open(a.out, "w", newline="")
    w = csv.writer(f); w.writerow(REC_HDR)
    print(f"-- TELEOP recording -> {a.out}   (click the pygame window; L=land ESC/Q=quit)", flush=True)
    dt = 1.0 / HZ
    target_alt = a.alt                   # hold the commanded takeoff altitude; W/S move it
    t0 = time.monotonic(); k = 0; landing = False
    while not landing and not STOP.is_set():
        beat(m); poll(m, st)
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                landing = True
            elif ev.type == pygame.KEYDOWN and ev.key in (pygame.K_l, pygame.K_ESCAPE, pygame.K_q):
                landing = True
        keys = pygame.key.get_pressed()
        cur_alt = -st["pos"][2]
        if keys[pygame.K_w]:                     # W = up  -> raise target
            target_alt += a.vz * dt
        elif keys[pygame.K_s]:                   # S = down -> lower target
            target_alt -= a.vz * dt
        if keys[pygame.K_SPACE]:
            target_alt = cur_alt                 # freeze at current altitude
        vx = a.fwd * (keys[pygame.K_UP] - keys[pygame.K_DOWN])        # arrows: fwd/back (body)
        yr = a.yawr * (keys[pygame.K_RIGHT] - keys[pygame.K_LEFT])    # right = +yaw rate
        if keys[pygame.K_SPACE]:
            vx = yr = 0.0
        # active altitude hold: drive vz to chase target_alt (NED down +) -> no drift
        vz = float(np.clip(KP_ALT * (cur_alt - target_alt), -a.vz, a.vz))
        send_body_vel(m, vx, vz, yr)
        yaw = st["att"][2] if st["att"] else 0.0
        w.writerow([round(time.monotonic() - t0, 3), round(vx, 3), round(vz, 3), round(yr, 3),
                    st.get("lat"), st.get("lon"), round(st.get("rel_alt", 0.0), 3),
                    *np.round(st["pos"], 3), round(yaw, 4)])
        f.flush()
        alt = -st["pos"][2]
        rad = math.hypot(st["pos"][0] - take[0], st["pos"][1] - take[1])
        screen.fill((16, 16, 26))
        rows = [f"vx {vx:+.2f}  vz {vz:+.2f}  yawrate {yr:+.2f}",
                f"alt {alt:+.2f} ->tgt {target_alt:+.2f} m   radius {rad:.1f} m   yaw {math.degrees(yaw):+.0f}",
                "W/S up/down   UP/DOWN fwd/back   LEFT/RIGHT yaw",
                "SPACE hover    L land    ESC/Q quit"]
        for i, ln in enumerate(rows):
            screen.blit(font.render(ln, True, (0, 235, 130) if i == 0 else (205, 205, 215)), (14, 16 + i * 30))
        pygame.display.flip()
        fails = [x for x in flight_guard_failures(st, take) if not x.startswith("alt ")]  # no altitude cap
        if fails:
            print("\n-- guard tripped:", "; ".join(fails), "-> landing", flush=True)
            break
        k += 1
        lag = t0 + k * dt - time.monotonic()
        if lag > 0:
            time.sleep(lag)
    f.close(); pygame.quit()
    print(f"-- teleop ended; recorded {k} ticks ({k / HZ:.1f} s) -> {a.out}", flush=True)


def replay(m, st, a, take):
    with open(a.replay) as fh:
        rows = [r for r in csv.DictReader(fh) if r.get("lat") not in (None, "", "None")]
    if not rows:
        sys.exit("replay log has no GPS rows")
    print(f"-- REPLAY {len(rows)} world-coordinate setpoints from {a.replay}  (ENTER=land)", flush=True)
    print("   NOTE: it flies to the recorded GPS positions — it will move to the teleop's", flush=True)
    print("   start point first, then follow the path. Keep clear.", flush=True)
    t0 = time.monotonic()
    for i, r in enumerate(rows):
        if STOP.is_set():
            print("\n-- ENTER -> land", flush=True)
            break
        beat(m); poll(m, st)
        send_global_pos(m, float(r["lat"]), float(r["lon"]), float(r["rel_alt"]), float(r["yaw"]))
        fails = [x for x in flight_guard_failures(st, take) if not x.startswith("alt ")]  # no altitude cap
        if fails:
            print("\n-- guard tripped:", "; ".join(fails), "-> landing", flush=True)
            break
        rel = -st["pos"][2]
        print(f"  [{i+1}/{len(rows)}] alt={rel:+.2f}m   ", end="\r", flush=True)
        t_next = float(rows[i + 1]["t"]) if i + 1 < len(rows) else float(r["t"]) + 1.0 / HZ
        lag = t0 + t_next - time.monotonic()
        if lag > 0:
            time.sleep(lag)
    print("\n-- replay done", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", default="/dev/ttyUSB0")   # Windows: COM13
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--alt", type=float, default=2.0, help="takeoff altitude (m)")
    ap.add_argument("--out", default="teleop_run.csv", help="teleop: trajectory log path")
    ap.add_argument("--replay", default=None, help="replay this recorded log instead of teleop")
    ap.add_argument("--fwd", type=float, default=0.5, help="forward/back speed m/s")
    ap.add_argument("--vz", type=float, default=0.4, help="up/down speed m/s")
    ap.add_argument("--yawr", type=float, default=0.5, help="yaw rate rad/s")
    ap.add_argument("--hover", type=float, default=0.7, help="MOT_THST_HOVER startup throttle (firmer liftoff)")
    ap.add_argument("--force", action="store_true", help="skip preflight gates (bench/SITL)")
    ap.add_argument("--yes", action="store_true", help="skip the GO prompt")
    a = ap.parse_args()

    st = init_st()
    m = connect(a.connect, a.baud)
    set_stream_rates(m)
    set_failsafe_norc(m)
    # Hold heading unless yaw is explicitly commanded: stop ArduCopter from auto-yawing
    # the nose toward the direction of travel during velocity control.
    set_param(m, "WP_YAW_BEHAVIOR", 0)
    # firmer liftoff: raise the takeoff throttle feedforward
    set_param(m, "MOT_THST_HOVER", a.hover, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    print(f"-- WP_YAW_BEHAVIOR=0 (hold heading); MOT_THST_HOVER={a.hover} (startup throttle)", flush=True)
    if not a.force:
        if not preflight_gates(m, st):
            sys.exit("ABORT: preflight gates failed")
    if not wait_ekf_ready(m, st):
        sys.exit("ABORT: no EKF position")
    if not set_mode(m, st, "GUIDED"):
        sys.exit("GUIDED failed")

    mode = "REPLAY" if a.replay else "TELEOP"
    if not a.yes:
        print(f"\n*** Type GO to arm + takeoff to {a.alt:.1f} m for {mode}: ***", flush=True)
        if input().strip().upper() != "GO":
            sys.exit("aborted")
    threading.Thread(target=stop_watcher, daemon=True).start()
    print("\n>>>>>>  ENTER (terminal) -> LAND at any time  <<<<<<\n", flush=True)
    if not arm(m, st):
        sys.exit("arm failed")
    print(f"-- takeoff -> {a.alt} m", flush=True)
    if not takeoff(m, st, a.alt):
        land_and_disarm(m, st)
        sys.exit("takeoff failed -> landed")
    while st["pos"] is None or st["att"] is None:
        beat(m); poll(m, st); time.sleep(0.05)
    for _ in range(15):
        beat(m); poll(m, st); time.sleep(0.1)
    take = st["pos"].copy()

    try:
        if a.replay:
            replay(m, st, a, take)
        else:
            teleop(m, st, a, take)
    finally:
        land_and_disarm(m, st)


if __name__ == "__main__":
    main()
