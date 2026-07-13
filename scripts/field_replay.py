#!/usr/bin/env python3
r"""X500 episode replay flight — modeled on x500_first_flight.py.

GUIDED -> preflight-gate -> arm -> takeoff -> yaw to the episode's recorded heading
-> replay a drone_hoop_30hz_v5 episode through the policy control shim as
SET_POSITION_TARGET_LOCAL_NED (pos+vel+yaw) setpoints -> LAND.

The recorded episodes have NO landing tail, so landing is triggered by ANY of:
  1. episode complete
  2. commanded descent after the straight/climb phase  ("losing altitude")
  3. operator presses ENTER                            (manual land kill)
  4. tracking / radius guard trips
Landing uses the repo's sequence: NAV_LAND -> watch descent while beating the GCS
heartbeat (so a slow descent isn't cut by FS_GCS) -> wait for auto-disarm.

Props ON. Close Mission Planner / the dashboard (COM13 is exclusive).

Usage:
  python field_replay.py --episode 276                    # real flight (COM13), gated
  python field_replay.py --episode 276 --hz 10 --alt 2
  python field_replay.py --hover 60                       # hover-only drift baseline
  python field_replay.py --episode 276 --force            # bypass preflight gates (bench)
  # SITL test (from WSL):
  python3 field_replay.py --connect tcp:127.0.0.1:5760 --dataset /mnt/d/lucky_drone_il/data/drone_hoop_30hz_v5 --episode 276 --force
"""
import argparse
import csv
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pandas as pd
from pymavlink import mavutil

# ---- control shim (verbatim from play_cmenc_imle_v1.py) ----
LOOKAHEAD_S = 0.5
CLIP_POS = 0.5
CLIP_YAW = 0.3
VMAX = 1.5
NATIVE_HZ = 30
# type_mask: set bit = IGNORE. pos+vel+ABSOLUTE-yaw -> ignore accel(6,7,8), force(9), yaw_rate(11)
USE_POS_VEL_YAW = (7 << 6) | (1 << 9) | (1 << 11)        # 3008
# pos+vel+YAW-RATE -> ignore accel(6,7,8), force(9), absolute-yaw(10); USE yaw_rate(11).
# Fix for the real-hardware yaw runaway: the 0.5 s-lookahead absolute-yaw command
# (yaw = live + dyaw) cumulates when the yaw controller is fast enough to reach the
# setpoint each tick, mirroring the trajectory (SITL's slow yaw hid it). Commanding the
# yaw RATE (dyaw / lookahead) integrates once -> correct total turn, slew-independent.
# Symmetric with the velocity feed-forward we already use for position.
USE_POS_VEL_YAWRATE = (7 << 6) | (1 << 9) | (1 << 10)    # 1984

# ---- landing triggers (dataset has no landing tail) ----
DESCENT_MARGIN = 0.4      # m below peak commanded altitude = "losing altitude"
STRAIGHT_DIST = 1.0       # m horizontal traveled before the descent detector arms
DESCENT_SECS = 0.5        # sustained commanded descent required to trip it
# ---- in-flight guards -> land ----
MAX_TRACK_ERR = 2.0       # m flown-vs-commanded horizontal error
MAX_RADIUS = 12.0         # m from takeoff (keep inside the geofence)
GUARD_SECS = 0.4
# ---- preflight quality gates (from x500_first_flight) ----
MIN_SATS, MAX_HDOP, MAX_EKF_VAR, MIN_VOLTS, LEVEL_DEG = 10, 1.5, 0.5, 13.0, 3.0
PRED_POS_HORIZ_ABS = 512  # EKF_STATUS_REPORT flag: absolute horizontal position ready

# Manual abort: first line on stdin (ENTER) -> immediate LAND.
STOP = threading.Event()


def stop_watcher():
    try:
        for _ in sys.stdin:
            STOP.set()
            break
    except Exception:
        pass


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def init_st():
    return {"pos": None, "vel": None, "att": None, "rel_alt": 0.0,
            "lat": None, "lon": None, "gspd": 0.0,
            "fix": 0, "sats": 0, "hdop": 99.9,          # running best for gates
            "ekf_var": None, "ekf_flags": 0, "volt": None,
            "armed": False, "mode": -1, "acks": {}}


# ---- link / commands ----

def connect(port, baud, timeout=30):
    print(f"connecting {port}@{baud}...", flush=True)
    m = mavutil.mavlink_connection(port, baud=baud)
    if m.wait_heartbeat(timeout=timeout) is None:
        sys.exit("NO HEARTBEAT")
    print(f"-- connected sys={m.target_system}", flush=True)
    return m


def set_param(m, name, value, ptype=mavutil.mavlink.MAV_PARAM_TYPE_INT32):
    m.mav.param_set_send(m.target_system, m.target_component,
                         name.encode(), float(value), ptype)
    time.sleep(0.2)


def beat(m):
    """GCS heartbeat. With no RC, FS_GCS LANDs if these stop -> call it in every loop."""
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


def set_failsafe_norc(m):
    print("-- no-RC failsafe: FS_THR_ENABLE=0, FS_GCS_ENABLE=5 (link-loss=LAND)", flush=True)
    set_param(m, "FS_THR_ENABLE", 0)
    set_param(m, "FS_GCS_ENABLE", 5)
    # learn+save true hover thrust: bad trim (MOT_THST_HOVER=0.6 read high on the first
    # flight, +0.3 m steady over-climb) makes every vertical maneuver asymmetric/hunty
    set_param(m, "MOT_HOVER_LEARN", 2)
    beat(m)


def set_stream_rates(m):
    """SiK-friendly rates: yaw (ATTITUDE) and NED pose fast; the rest slow."""
    rates = {mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 15,
             mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 10,
             mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 3,
             mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: 2,
             mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 2,
             mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS: 1}
    for mid, hz in rates.items():
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                mid, int(1e6 / hz), 0, 0, 0, 0, 0)
        time.sleep(0.05)


def poll(m, st):
    """Non-blocking: fold all pending messages into st (single thread, no locks)."""
    while True:
        msg = m.recv_match(blocking=False)
        if msg is None:
            return
        t = msg.get_type()
        if t == "LOCAL_POSITION_NED":
            st["pos"] = np.array([msg.x, msg.y, msg.z])
            st["vel"] = np.array([msg.vx, msg.vy, msg.vz])
        elif t == "ATTITUDE":
            st["att"] = (msg.roll, msg.pitch, msg.yaw)
        elif t == "GLOBAL_POSITION_INT":
            st["rel_alt"] = msg.relative_alt / 1000.0
            st["lat"], st["lon"] = msg.lat / 1e7, msg.lon / 1e7
            st["gspd"] = math.hypot(msg.vx, msg.vy) / 100.0
        elif t == "GPS_RAW_INT":
            st["fix"] = max(st["fix"], msg.fix_type)
            st["sats"] = max(st["sats"], msg.satellites_visible)
            hd = msg.eph / 100.0 if msg.eph != 65535 else 99.9
            st["hdop"] = min(st["hdop"], hd)
        elif t == "EKF_STATUS_REPORT":
            st["ekf_var"] = (msg.pos_horiz_variance, msg.velocity_variance,
                             msg.compass_variance)
            st["ekf_flags"] = msg.flags
        elif t == "SYS_STATUS":
            st["volt"] = msg.voltage_battery / 1000.0
        elif t == "HEARTBEAT":
            st["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            st["mode"] = msg.custom_mode
        elif t == "STATUSTEXT":
            print(f"   [fc] {msg.text.strip()}", flush=True)
        elif t == "COMMAND_ACK":
            st["acks"][msg.command] = msg.result


def send_target(m, setpt, v, yaw=0.0, yaw_rate=0.0, use_rate=False):
    mask = USE_POS_VEL_YAWRATE if use_rate else USE_POS_VEL_YAW
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
        setpt[0], setpt[1], setpt[2], v[0], v[1], v[2], 0, 0, 0, yaw, yaw_rate)


# ---- preflight gates (ported from x500_first_flight) ----

def gate_failures(st):
    f = []
    if st["fix"] < 3:
        f.append(f"GPS fix {st['fix']} < 3D")
    if st["sats"] < MIN_SATS:
        f.append(f"sats {st['sats']} < {MIN_SATS}")
    if st["hdop"] > MAX_HDOP:
        f.append(f"HDOP {st['hdop']:.1f} > {MAX_HDOP}")
    if st["ekf_var"] is None:
        f.append("no EKF_STATUS_REPORT yet")
    else:
        ph, ve, co = st["ekf_var"]
        if ph > MAX_EKF_VAR:
            f.append(f"EKF pos_var {ph:.2f} > {MAX_EKF_VAR}")
        if ve > MAX_EKF_VAR:
            f.append(f"EKF vel_var {ve:.2f} > {MAX_EKF_VAR}")
        if co > MAX_EKF_VAR:
            f.append(f"EKF compass_var {co:.2f} > {MAX_EKF_VAR} (run compass cal)")
    if st["volt"] is not None and 0.5 < st["volt"] <= MIN_VOLTS:
        f.append(f"battery {st['volt']:.1f}V <= {MIN_VOLTS}V")
    if st["att"] is None:
        f.append("no ATTITUDE yet")
    elif abs(math.degrees(st["att"][0])) > LEVEL_DEG or abs(math.degrees(st["att"][1])) > LEVEL_DEG:
        f.append("not level")
    return f


def preflight_gates(m, st, timeout=90):
    print(f"-- preflight gates (fix>=3, sats>={MIN_SATS}, HDOP<={MAX_HDOP}, "
          f"EKF var<={MAX_EKF_VAR}, level<{LEVEL_DEG}deg), up to {timeout}s...", flush=True)
    t0, good, last = time.time(), 0, 0.0
    while time.time() - t0 < timeout:
        if STOP.is_set():
            return False
        beat(m)
        poll(m, st)
        fails = gate_failures(st)
        if not fails:
            good += 1
            if good >= 2:
                print("\n-- preflight gates PASSED", flush=True)
                return True
        else:
            good = 0
            if time.time() - last > 3:
                last = time.time()
                print(f"  waiting: {'; '.join(fails)}      ", end="\r", flush=True)
        time.sleep(0.2)
    print("\n-- preflight gates FAILED: " + "; ".join(gate_failures(st)), flush=True)
    return False


# ---- flight primitives ----

def set_mode(m, st, name, timeout=8):
    mode_id = m.mode_mapping()[name]
    t0 = time.time()
    while time.time() - t0 < timeout:
        m.mav.set_mode_send(m.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
        beat(m)
        poll(m, st)
        if st["mode"] == mode_id:
            return True
        time.sleep(0.4)
    return False


def wait_ekf_ready(m, st, timeout=120):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if STOP.is_set():
            return False
        beat(m)
        poll(m, st)
        if st["ekf_flags"] & PRED_POS_HORIZ_ABS:
            return True
        time.sleep(0.2)
    return False


def arm(m, st, timeout=60):
    ARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    t0 = time.time()
    while time.time() - t0 < timeout:
        if STOP.is_set():
            return False
        m.mav.command_long_send(m.target_system, m.target_component, ARM, 0,
                                1, 0, 0, 0, 0, 0, 0)
        for _ in range(15):          # beat ~1.5s while waiting on the ACK/heartbeat
            beat(m)
            poll(m, st)
            if st["armed"]:
                return True
            time.sleep(0.1)
    return False


def takeoff(m, st, alt, timeout=25):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                            0, 0, 0, 0, 0, 0, alt)
    t0 = time.time()
    peak = 0.0
    while time.time() - t0 < timeout:
        if STOP.is_set():
            return False
        beat(m)
        poll(m, st)
        peak = max(peak, st["rel_alt"])
        print(f"  climbing alt={st['rel_alt']:+.2f}m (peak {peak:.2f})   ", end="\r", flush=True)
        if st["rel_alt"] >= 0.9 * alt:
            print(f"\n-- reached {st['rel_alt']:.2f}m", flush=True)
            return True
        time.sleep(0.1)
    print(f"\n-- takeoff window ended, peak {peak:.2f}m", flush=True)
    return peak > 0.5


def land_and_disarm(m, st, timeout=35):
    """NAV_LAND, watch descent while beating (FS_GCS-safe), wait for auto-disarm."""
    print("-- LAND (NAV_LAND); waiting for touchdown + auto-disarm...", flush=True)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    while time.time() - t0 < timeout:
        beat(m)               # keep sending so a slow descent isn't cut by FS_GCS
        poll(m, st)
        if not st["armed"]:
            print(f"\n-- auto-disarmed at {st['rel_alt']:+.2f}m; landed.", flush=True)
            return
        print(f"  descending alt={st['rel_alt']:+.2f}m   ", end="\r", flush=True)
        time.sleep(0.1)
    print("\n-- still armed after landing window; explicit disarm", flush=True)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                            0, 0, 0, 0, 0, 0, 0)


CSV_HEADER = ["tick", "t", "ax", "ay", "az", "ayaw",
              "sp_n", "sp_e", "sp_d", "v_n", "v_e", "v_d", "yaw_cmd",
              "p_n", "p_e", "p_d", "vel_n", "vel_e", "vel_d", "roll", "pitch", "yaw"]


def yaw_to(m, st, setpt, yaw_target, timeout=8):
    """Hold position, rotate to yaw_target. ENTER-abortable. Returns True unless STOP."""
    print(f"-- yawing to recorded yaw0 {math.degrees(yaw_target):.0f} deg (ENTER=land)", flush=True)
    t0 = time.time()
    while time.time() - t0 < timeout:
        if STOP.is_set():
            return False
        beat(m)
        poll(m, st)
        send_target(m, setpt, np.zeros(3), yaw_target)
        if st["att"] and abs(wrap(st["att"][2] - yaw_target)) < math.radians(3):
            return True
        time.sleep(0.1)
    return True


def do_replay(m, st, acts, rec_yaw0, hz, out, use_yaw_rate=True):
    """Stream the episode through the shim. Returns the reason replay ended."""
    dt = 1.0 / hz
    while st["pos"] is None or st["att"] is None:   # need NED pos + attitude first
        beat(m)
        poll(m, st)
        time.sleep(0.05)
    setpt = st["pos"].copy()
    sp0 = setpt.copy()
    take = st["pos"].copy()

    if not yaw_to(m, st, setpt, rec_yaw0):
        return "ENTER(pre-replay)"

    print(f"-- REPLAYING {len(acts)} setpoints @ {hz:g}Hz  (ENTER=land)  -> {out}", flush=True)
    descent_ticks = max(3, round(DESCENT_SECS * hz))
    guard_ticks = max(2, round(GUARD_SECS * hz))
    peak_up, descending, track_bad, reason = -setpt[2], 0, 0, "episode-complete"
    t0 = time.time()
    k = 0
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_HEADER)
        for k, act in enumerate(acts):
            if STOP.is_set():
                reason = "ENTER"
                break
            beat(m)
            poll(m, st)
            yaw = st["att"][2]
            d = np.clip(act[:3], -CLIP_POS, CLIP_POS)
            dyaw = float(np.clip(act[3], -CLIP_YAW, CLIP_YAW))
            cy, sy = math.cos(yaw), math.sin(yaw)
            dn = np.array([cy * d[0] - sy * d[1], sy * d[0] + cy * d[1], d[2]])
            v = dn / LOOKAHEAD_S
            nn = float(np.linalg.norm(v))
            if nn > VMAX:
                v *= VMAX / nn
            setpt = setpt + v * dt
            yaw_cmd = wrap(yaw + dyaw)             # notional heading (logged only)
            if use_yaw_rate:                       # FIX: command yaw RATE, not abs heading
                send_target(m, setpt, v, yaw_rate=dyaw / LOOKAHEAD_S, use_rate=True)
            else:
                send_target(m, setpt, v, yaw=yaw_cmd)
            w.writerow([k, round(time.time() - t0, 4), *act, *setpt, *v, yaw_cmd,
                        *st["pos"], *st["vel"], *st["att"]])
            f.flush()

            # trigger 2: commanded descent after the straight/climb phase
            up = -setpt[2]
            peak_up = max(peak_up, up)
            straight = math.hypot(setpt[0] - sp0[0], setpt[1] - sp0[1]) > STRAIGHT_DIST
            descending = descending + 1 if (straight and up < peak_up - DESCENT_MARGIN) else 0
            if descending >= descent_ticks:
                reason = f"descent({peak_up - up:.2f}m below peak)"
                break
            # trigger 4: tracking / radius guard
            terr = math.hypot(st["pos"][0] - setpt[0], st["pos"][1] - setpt[1])
            rad = math.hypot(st["pos"][0] - take[0], st["pos"][1] - take[1])
            track_bad = track_bad + 1 if (terr > MAX_TRACK_ERR or rad > MAX_RADIUS) else 0
            if track_bad >= guard_ticks:
                reason = f"guard(track_err={terr:.1f}m, radius={rad:.1f}m)"
                break

            lag = t0 + (k + 1) * dt - time.time()
            if lag > 0:
                time.sleep(lag)
    hz_act = (k + 1) / (time.time() - t0)
    print(f"\n-- replay ended [{reason}] at tick {k}/{len(acts) - 1}, {hz_act:.1f}Hz; "
          f"wrote {out}", flush=True)
    return reason


def do_hover(m, st, secs):
    """Hover-only drift baseline: hold the takeoff position, log drift, land."""
    while st["pos"] is None:
        beat(m)
        poll(m, st)
        time.sleep(0.05)
    hold = st["pos"].copy()
    yaw0 = st["att"][2] if st["att"] else 0.0
    print(f"-- HOVER hold {secs:g}s (ENTER=land); measuring drift...", flush=True)
    t0, maxr = time.time(), 0.0
    while time.time() - t0 < secs:
        if STOP.is_set():
            print("\n-- ENTER -> land", flush=True)
            break
        beat(m)
        poll(m, st)
        send_target(m, hold, np.zeros(3), yaw0)
        r = math.hypot(st["pos"][0] - hold[0], st["pos"][1] - hold[1])
        maxr = max(maxr, r)
        print(f"  drift={r:.2f}m (max {maxr:.2f})  spd={math.hypot(*st['vel'][:2]):.2f}m/s   ",
              end="\r", flush=True)
        time.sleep(0.1)
    print(f"\n-- hover done, max drift {maxr:.2f}m", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", default="/dev/ttyUSB0")   # Windows: COM13 ; SITL: tcp:127.0.0.1:5760
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--episode-file", default=None,
                    help="standalone episode parquet (default: bundled data/drone_hoop_ep276.parquet)")
    ap.add_argument("--dataset", default=None,
                    help="full LeRobot dataset dir (alternative to --episode-file, for other episodes)")
    ap.add_argument("--episode", type=int, default=276)
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--alt", type=float, default=2.0)
    ap.add_argument("--stride", type=int, default=0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--hover", type=float, default=0.0, help="hover-only baseline N s (no replay)")
    ap.add_argument("--force", action="store_true", help="bypass preflight gates (bench/SITL)")
    ap.add_argument("--yaw-abs", action="store_true",
                    help="command absolute yaw=live+dyaw (old behavior; runs away on a fast "
                         "yaw controller -> mirrored path). Default = yaw RATE (dyaw/lookahead).")
    a = ap.parse_args()

    replay = a.hover <= 0
    acts = rec_yaw0 = None
    if replay:
        if a.dataset:
            src = f"{a.dataset}/data/chunk-000/file-000.parquet"
        else:
            src = a.episode_file or str(
                Path(__file__).resolve().parent.parent / "data" / "drone_hoop_ep276.parquet")
        df = pd.read_parquet(src, columns=["episode_index", "action", "observation.state"])
        ep = df[df["episode_index"] == a.episode]
        if not len(ep):
            sys.exit(f"episode {a.episode} not in {src}")
        acts = np.stack(ep["action"].values).astype(float)
        s0 = np.stack(ep["observation.state"].values)[0]
        rec_yaw0 = math.atan2(float(s0[2]), float(s0[3]))
        stride = a.stride or max(1, round(NATIVE_HZ / a.hz))
        acts = acts[::stride]
        print(f"-- episode {a.episode}: {len(acts)} setpoints @ {a.hz:g}Hz "
              f"(stride {stride}, {len(acts) / a.hz:.1f}s), yaw0 {math.degrees(rec_yaw0):.0f}deg",
              flush=True)

    st = init_st()
    m = connect(a.connect, a.baud)
    set_stream_rates(m)
    set_failsafe_norc(m)

    threading.Thread(target=stop_watcher, daemon=True).start()
    print("\n>>>>>>  PRESS ENTER AT ANY TIME -> LAND  <<<<<<\n", flush=True)

    if not a.force:
        if not preflight_gates(m, st):
            sys.exit("ABORT: preflight gates not satisfied (use --force for bench)")
    else:
        print("!! --force: skipping preflight gates", flush=True)
    if not wait_ekf_ready(m, st):
        sys.exit("ABORT: EKF never reported a position estimate")

    print("-- GUIDED...", flush=True)
    if not set_mode(m, st, "GUIDED"):
        sys.exit("GUIDED failed")
    print("-- arming...", flush=True)
    if not arm(m, st):
        sys.exit("ENTER -> aborted before flight" if STOP.is_set() else "arm failed")
    print(f"-- takeoff -> {a.alt}m...", flush=True)
    if not takeoff(m, st, a.alt):
        if STOP.is_set():
            print("-- ENTER during takeoff -> landing", flush=True)
        land_and_disarm(m, st)
        sys.exit("landed (ENTER)" if STOP.is_set() else "takeoff failed -> landed")

    for _ in range(20):          # ~2s settle at hover, beating
        beat(m)
        poll(m, st)
        time.sleep(0.1)

    try:
        if replay:
            out = a.out or f"replay_ep{a.episode:03d}_field.csv"
            do_replay(m, st, acts, rec_yaw0, a.hz, out, use_yaw_rate=not a.yaw_abs)
        else:
            do_hover(m, st, a.hover)
    finally:
        land_and_disarm(m, st)   # always land, even on exception


if __name__ == "__main__":
    main()
