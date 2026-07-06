#!/usr/bin/env python3
"""Score a sitl_replay.py log: tracking error (flown vs commanded setpoint) and
open-loop divergence (flown vs offline reference integrated with RECORDED yaw).

  python3 analyze_replay.py replay_ep000.csv --dataset .../drone_hoop_30hz_v5 --episode 0
"""
import argparse
import math

import numpy as np
import pandas as pd

LOOKAHEAD_S, CLIP_POS, CLIP_YAW, VMAX = 0.5, 0.5, 0.3, 1.5
NATIVE_HZ = 30


def reference(acts, yaw_rec, hz):
    """Offline shim integration using recorded yaw (the intended path)."""
    pos = np.zeros(3)
    traj = [pos.copy()]
    for t in range(len(acts)):
        d = np.clip(acts[t, :3], -CLIP_POS, CLIP_POS)
        cy, sy = math.cos(yaw_rec[t]), math.sin(yaw_rec[t])
        v = np.array([cy * d[0] - sy * d[1], sy * d[0] + cy * d[1], d[2]]) / LOOKAHEAD_S
        n = np.linalg.norm(v)
        if n > VMAX:
            v *= VMAX / n
        pos = pos + v / hz
        traj.append(pos.copy())
    return np.array(traj[1:])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--episode", type=int, required=True)
    ap.add_argument("--hz", type=float, default=30.0)
    ap.add_argument("--stride", type=int, default=0,
                    help="dataset subsample (0 = auto: round(30 / hz)); must match the replay")
    ap.add_argument("--plot", default=None, help="output png (default <csv>.png)")
    a = ap.parse_args()

    log = pd.read_csv(a.csv)
    df = pd.read_parquet(f"{a.dataset}/data/chunk-000/file-000.parquet",
                         columns=["episode_index", "action", "observation.state"])
    ep = df[df["episode_index"] == a.episode]
    stride = a.stride or max(1, round(NATIVE_HZ / a.hz))
    acts = np.stack(ep["action"].values).astype(float)[::stride][:len(log)]
    st = np.stack(ep["observation.state"].values)[::stride][:len(log)]
    yaw_rec = np.arctan2(st[:, 2], st[:, 3])

    flown = log[["p_n", "p_e", "p_d"]].to_numpy()
    flown = flown - flown[0]                       # both start at origin
    cmd = log[["sp_n", "sp_e", "sp_d"]].to_numpy()
    cmd = cmd - cmd[0]
    ref = reference(acts, yaw_rec, a.hz)

    def rmse(e):
        return float(np.sqrt((e ** 2).mean()))

    track_xy = rmse(np.linalg.norm(flown[:, :2] - cmd[:, :2], axis=1))
    track_z = rmse(flown[:, 2] - cmd[:, 2])
    yaw_err = np.arctan2(np.sin(log["yaw"] - log["yaw_cmd"]),
                         np.cos(log["yaw"] - log["yaw_cmd"]))
    div = np.linalg.norm(flown - ref, axis=1)
    hz_act = (len(log) - 1) / (log["t"].iloc[-1] - log["t"].iloc[0])
    clip_pct = (np.abs(acts[:, 0]) > CLIP_POS).mean() * 100

    print(f"episode {a.episode}  ({len(log)} ticks, {hz_act:.1f} Hz achieved, "
          f"{clip_pct:.1f}% dx clipped)")
    print(f"  tracking  RMSE  XY {track_xy:.3f} m   Z {track_z:.3f} m   "
          f"yaw {math.degrees(rmse(yaw_err.to_numpy())):.1f} deg")
    print(f"  divergence vs reference: mean {div.mean():.2f} m,  "
          f"end {div[-1]:.2f} m,  max {div.max():.2f} m")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))
    ax[0].plot(ref[:, 1], ref[:, 0], label="reference (recorded yaw)")
    ax[0].plot(cmd[:, 1], cmd[:, 0], "--", label="commanded setpt")
    ax[0].plot(flown[:, 1], flown[:, 0], label="flown")
    ax[0].set_xlabel("East (m)"); ax[0].set_ylabel("North (m)")
    ax[0].axis("equal"); ax[0].legend(); ax[0].set_title("top-down")
    t = log["t"]
    ax[1].plot(t, -ref[:, 2], label="reference")
    ax[1].plot(t, -cmd[:, 2], "--", label="commanded")
    ax[1].plot(t, -flown[:, 2], label="flown")
    ax[1].set_xlabel("t (s)"); ax[1].set_ylabel("up (m)"); ax[1].legend()
    ax[1].set_title("altitude (rel)")
    ax[2].plot(t, np.linalg.norm(flown[:, :2] - cmd[:, :2], axis=1), label="track err XY")
    ax[2].plot(t, div, label="divergence vs ref")
    ax[2].set_xlabel("t (s)"); ax[2].set_ylabel("m"); ax[2].legend()
    ax[2].set_title("errors")
    fig.suptitle(f"replay ep{a.episode}  track XY {track_xy:.2f} m  "
                 f"div end {div[-1]:.2f} m")
    out = a.plot or a.csv.replace(".csv", ".png")
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    print(f"  plot -> {out}")


if __name__ == "__main__":
    main()
