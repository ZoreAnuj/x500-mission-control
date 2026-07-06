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
                mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT):
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
    a = ap.parse_args()

    df = pd.read_parquet(f"{a.dataset}/data/chunk-000/file-000.parquet",
                         columns=["episode_index", "action", "observation.state"])
    ep = df[df["episode_index"] == a.episode]
    if not len(ep):
        raise SystemExit(f"episode {a.episode} not found")
    acts = np.stack(ep["action"].values).astype(float)
    s0 = np.stack(ep["observation.state"].values)[0]
    rec_yaw0 = math.atan2(float(s0[2]), float(s0[3]))
    clip_pct = float((np.abs(acts[:, 0]) > CLIP_POS).mean() * 100)
    print(f"-- episode {a.episode}: {len(acts)} frames ({len(acts)/30:.1f} s), "
          f"{clip_pct:.1f}% dx clipped, recorded yaw0 {math.degrees(rec_yaw0):.0f} deg")

    m = connect(a.connect, a.baud)
    st = {"pos": None, "vel": None, "att": None, "rel_alt": 0.0, "acks": {}}
    set_rates(m, max(30, int(a.hz)))

    # GUIDED -> arm (retry while prearm settles) -> takeoff
    m.set_mode(m.mode_mapping()["GUIDED"])
    print("-- arming (retries while prearm clears)...")
    ARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    for i in range(40):
        m.mav.command_long_send(m.target_system, m.target_component, ARM, 0,
                                1, 0, 0, 0, 0, 0, 0)
        time.sleep(1.5)
        drain(m, st)
        if st["acks"].get(ARM) == 0:
            break
    else:
        raise SystemExit("arm rejected after 60 s (check prearm STATUSTEXT above)")
    print("-- armed; takeoff")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                            0, 0, 0, 0, 0, 0, a.alt)
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

    # replay
    out = a.out or f"replay_ep{a.episode:03d}.csv"
    dt = 1.0 / a.hz
    print(f"-- replaying {len(acts)} actions @ {a.hz:g} Hz -> {out}")
    rows = []
    t0 = time.time()
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
        rows.append([k, time.time() - t0, *act, *setpt, *v, yaw_cmd,
                     *st["pos"], *st["vel"], *st["att"]])
        # pace to hz (absolute schedule, no cumulative slip)
        lag = t0 + (k + 1) * dt - time.time()
        if lag > 0:
            time.sleep(lag)
    hz_act = len(acts) / (time.time() - t0)
    print(f"-- done, achieved {hz_act:.1f} Hz; landing")
    m.set_mode(m.mode_mapping()["LAND"])

    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["tick", "t", "ax", "ay", "az", "ayaw",
                    "sp_n", "sp_e", "sp_d", "v_n", "v_e", "v_d", "yaw_cmd",
                    "p_n", "p_e", "p_d", "vel_n", "vel_e", "vel_d",
                    "roll", "pitch", "yaw"])
        w.writerows(rows)
    print(f"-- wrote {out} ({len(rows)} rows, {hz_act:.1f} Hz achieved)")


if __name__ == "__main__":
    main()
