#!/usr/bin/env python3
"""Replay a drone_hoop_30hz_v5 episode's actions through the policy control shim
into ArduCopter (SITL or real X500) and log commanded vs actual state.

The shim math is verbatim from play_cmenc_imle_v1.py live inference:
  clip +-0.5 m / +-0.3 rad -> rotate body->NED by LIVE yaw -> v = delta/0.5 s
  (clamp 1.5 m/s) -> setpt += v*dt -> yaw_cmd = wrap(live_yaw + dyaw)
Streamed as SET_POSITION_TARGET_LOCAL_NED (pos + vel_ff + yaw) in GUIDED.

SITL:  python3 sitl_replay.py --connect tcp:127.0.0.1:5760 \
           --dataset /mnt/d/lucky_drone_il/data/drone_hoop_30hz_v5 --episode 0
Real:  same, --connect COM13 --baud 57600 --hz 10   (reduced rate over SiK)
"""
import argparse
import csv
import math
import time

import numpy as np
import pandas as pd
from pymavlink import mavutil

LOOKAHEAD_S = 0.5          # action = body waypoint over this horizon (dataset def)
CLIP_POS = 0.5             # live-inference clips (parity with play_cmenc_imle_v1.py)
CLIP_YAW = 0.3
VMAX = 1.5
NATIVE_HZ = 30            # dataset native rate (drone_hoop_30hz_v5)
# type_mask: set bit = IGNORE. Use pos+vel+yaw -> ignore accel(6,7,8), force(9), yaw_rate(11)
USE_POS_VEL_YAW = (7 << 6) | (1 << 9) | (1 << 11)   # 3008 = 0xBC0

MODES = None  # filled from mode_mapping()


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def connect(cs, baud):
    m = mavutil.mavlink_connection(cs, baud=baud)
    m.wait_heartbeat(timeout=30)
    print(f"-- heartbeat sys={m.target_system}")
    return m


def drain(m, st):
    """Non-blocking: fold all pending messages into st. Single thread, no locks."""
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
        elif t == "EKF_STATUS_REPORT":
            st["ekf"] = msg.flags
        elif t == "HEARTBEAT":
            st["armed"] = bool(msg.base_mode
                               & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            st["mode"] = msg.custom_mode
        elif t == "STATUSTEXT":
            print(f"   [fc] {msg.text}")
        elif t == "COMMAND_ACK":
            st["acks"][msg.command] = msg.result


def wait_for(m, st, cond, timeout, what):
    t0 = time.time()
    while time.time() - t0 < timeout:
        drain(m, st)
        if cond(st):
            return True
        time.sleep(0.05)
    raise TimeoutError(f"timed out waiting for {what}")


def set_rates(m, hz):
    for mid in (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT):
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                mid, int(1e6 / hz), 0, 0, 0, 0, 0)
        time.sleep(0.05)


def send_target(m, setpt, v, yaw):
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, USE_POS_VEL_YAW,
        setpt[0], setpt[1], setpt[2], v[0], v[1], v[2], 0, 0, 0, yaw, 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", default="tcp:127.0.0.1:5760")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--episode", type=int, default=0)
    ap.add_argument("--hz", type=float, default=30.0)
    ap.add_argument("--alt", type=float, default=2.0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--wind", default=None, metavar="SPD,DIR,TURB",
                    help="SITL wind: speed m/s, direction deg (from), turbulence "
                         "m/s. Austin TX typical: 4,165,2")
    ap.add_argument("--stride", type=int, default=0,
                    help="subsample dataset actions (0 = auto: round(30 / hz)). "
                         "SiK-rate rehearsal: --hz 10 gives stride 3.")
    a = ap.parse_args()

    df = pd.read_parquet(f"{a.dataset}/data/chunk-000/file-000.parquet",
                         columns=["episode_index", "action", "observation.state"])
    ep = df[df["episode_index"] == a.episode]
    if not len(ep):
        raise SystemExit(f"episode {a.episode} not found")
    acts = np.stack(ep["action"].values).astype(float)
    s0 = np.stack(ep["observation.state"].values)[0]
    rec_yaw0 = math.atan2(float(s0[2]), float(s0[3]))
    stride = a.stride or max(1, round(NATIVE_HZ / a.hz))
    acts = acts[::stride]   # SiK-rate rehearsal: keep wall-clock + per-step motion true
    clip_pct = float((np.abs(acts[:, 0]) > CLIP_POS).mean() * 100)
    print(f"-- episode {a.episode}: {len(acts)} actions @ {a.hz:g} Hz "
          f"(stride {stride}, {len(acts)/a.hz:.1f} s), {clip_pct:.1f}% dx clipped, "
          f"recorded yaw0 {math.degrees(rec_yaw0):.0f} deg")

    m = connect(a.connect, a.baud)
    st = {"pos": None, "vel": None, "att": None, "rel_alt": 0.0, "acks": {},
          "ekf": 0, "armed": False, "mode": -1}
    set_rates(m, max(30, int(a.hz)))

    if a.wind:
        spd, wdir, turb = (float(x) for x in a.wind.split(","))
        for name, val in (("SIM_WIND_SPD", spd), ("SIM_WIND_DIR", wdir),
                          ("SIM_WIND_TURB", turb)):
            m.mav.param_set_send(m.target_system, m.target_component,
                                 name.encode(), val,
                                 mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            time.sleep(0.1)
        print(f"-- SITL wind: {spd:g} m/s from {wdir:g} deg, turb {turb:g} m/s")

    # EKF position ready -> GUIDED (verified) -> arm -> takeoff (retry, re-arm)
    PRED_POS_HORIZ_ABS = 512
    print("-- waiting for EKF position estimate...")
    wait_for(m, st, lambda s: s["ekf"] & PRED_POS_HORIZ_ABS, 120, "EKF position")
    guided = m.mode_mapping()["GUIDED"]
    print("-- setting GUIDED...")
    while st["mode"] != guided:      # setting mode too early gets ignored -> verify
        m.set_mode(guided)
        time.sleep(0.5)
        drain(m, st)
    ARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM

    def arm():
        for _ in range(40):
            m.mav.command_long_send(m.target_system, m.target_component, ARM, 0,
                                    1, 0, 0, 0, 0, 0, 0)
            time.sleep(1.5)
            drain(m, st)
            if st["armed"]:
                return
        raise SystemExit("arm rejected after 60 s (see prearm STATUSTEXT above)")

    print("-- arming...")
    arm()
    print("-- armed; takeoff")
    for _ in range(10):
        if not st["armed"]:          # auto-disarmed while takeoff was rejected
            arm()
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                                0, 0, 0, 0, 0, 0, a.alt)
        time.sleep(2.0)
        drain(m, st)
        if st["rel_alt"] > 0.5:
            break
    wait_for(m, st, lambda s: s["rel_alt"] >= 0.9 * a.alt, 60, "takeoff altitude")
    time.sleep(3)  # let the hover settle
    drain(m, st)

    # yaw to the episode's recorded initial yaw so replay starts frame-aligned
    setpt = st["pos"].copy()
    print(f"-- yawing to recorded yaw0 {math.degrees(rec_yaw0):.0f} deg")
    for _ in range(int(6 / 0.1)):
        send_target(m, setpt, np.zeros(3), rec_yaw0)
        time.sleep(0.1)
        drain(m, st)
        if abs(wrap(st["att"][2] - rec_yaw0)) < math.radians(3):
            break

    # replay — rows streamed to disk per tick so a viewer can tail the file live
    out = a.out or f"replay_ep{a.episode:03d}.csv"
    dt = 1.0 / a.hz
    print(f"-- replaying {len(acts)} actions @ {a.hz:g} Hz -> {out}")
    t0 = time.time()
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["tick", "t", "ax", "ay", "az", "ayaw",
                    "sp_n", "sp_e", "sp_d", "v_n", "v_e", "v_d", "yaw_cmd",
                    "p_n", "p_e", "p_d", "vel_n", "vel_e", "vel_d",
                    "roll", "pitch", "yaw"])
        for k, act in enumerate(acts):
            drain(m, st)
            yaw = st["att"][2]
            d = np.clip(act[:3], -CLIP_POS, CLIP_POS)
            dyaw = float(np.clip(act[3], -CLIP_YAW, CLIP_YAW))
            cy, sy = math.cos(yaw), math.sin(yaw)
            dn = np.array([cy * d[0] - sy * d[1], sy * d[0] + cy * d[1], d[2]])
            v = dn / LOOKAHEAD_S
            n = float(np.linalg.norm(v))
            if n > VMAX:
                v *= VMAX / n
            setpt = setpt + v * dt
            yaw_cmd = wrap(yaw + dyaw)
            send_target(m, setpt, v, yaw_cmd)
            w.writerow([k, round(time.time() - t0, 4), *act, *setpt, *v, yaw_cmd,
                        *st["pos"], *st["vel"], *st["att"]])
            f.flush()
            # pace to hz (absolute schedule, no cumulative slip)
            lag = t0 + (k + 1) * dt - time.time()
            if lag > 0:
                time.sleep(lag)
    hz_act = len(acts) / (time.time() - t0)
    print(f"-- done, achieved {hz_act:.1f} Hz; landing")
    m.set_mode(m.mode_mapping()["LAND"])
    print(f"-- wrote {out} ({len(acts)} rows)")


if __name__ == "__main__":
    main()
