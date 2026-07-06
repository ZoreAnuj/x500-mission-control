#!/usr/bin/env python3
"""Render a replay CSV to an mp4 that reproduces the live preview
(growing commanded-vs-flown trails, tick/err title). Realtime speed.

  python render_replay_video.py runs/replay_ep050.csv --hz 30
"""
import argparse

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter

ap = argparse.ArgumentParser()
ap.add_argument("csv")
ap.add_argument("--hz", type=float, default=30.0, help="replay rate (for timing)")
ap.add_argument("--stride", type=int, default=2, help="ticks per video frame")
ap.add_argument("--out", default=None)
a = ap.parse_args()

df = pd.read_csv(a.csv)
sp = df[["sp_n", "sp_e", "sp_d"]].to_numpy()
sp = sp - sp[0]
p = df[["p_n", "p_e", "p_d"]].to_numpy()
p = p - p[0]
t = df["t"].to_numpy()
n = len(df)

fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(11, 5.5), dpi=110)
(l_sp,) = ax0.plot([], [], "--", color="tab:orange", label="commanded")
(l_p,) = ax0.plot([], [], color="tab:green", label="flown")
(dot,) = ax0.plot([], [], "o", color="tab:green", ms=8)
ax0.set_xlabel("East (m)"); ax0.set_ylabel("North (m)")
ax0.legend(loc="upper left"); ax0.set_title("top-down"); ax0.grid(alpha=0.3)
(l_spz,) = ax1.plot([], [], "--", color="tab:orange", label="commanded")
(l_pz,) = ax1.plot([], [], color="tab:green", label="flown")
ax1.set_xlabel("t (s)"); ax1.set_ylabel("up (m)")
ax1.legend(loc="upper left"); ax1.set_title("altitude (rel)"); ax1.grid(alpha=0.3)

# fixed limits from the full run (no autoscale jitter in the video);
# square window so aspect=equal never rescales the view mid-video
allxy = np.vstack([sp[:, :2], p[:, :2]])
ce = (allxy[:, 1].min() + allxy[:, 1].max()) / 2
cn = (allxy[:, 0].min() + allxy[:, 0].max()) / 2
half = max(np.ptp(allxy[:, 1]), np.ptp(allxy[:, 0])) / 2 + 0.5
ax0.set_xlim(ce - half, ce + half)
ax0.set_ylim(cn - half, cn + half)
ax0.set_aspect("equal", adjustable="box")
ax1.set_xlim(0, t[-1] + 0.5)
allz = np.hstack([-sp[:, 2], -p[:, 2]])
ax1.set_ylim(allz.min() - 0.3, allz.max() + 0.3)

out = a.out or a.csv.replace(".csv", ".mp4")
fps = max(1, round(a.hz / a.stride))
writer = FFMpegWriter(fps=fps, bitrate=2000)
with writer.saving(fig, out, dpi=110):
    for k in range(1, n + 1, a.stride):
        l_sp.set_data(sp[:k, 1], sp[:k, 0]); l_p.set_data(p[:k, 1], p[:k, 0])
        dot.set_data([p[k - 1, 1]], [p[k - 1, 0]])
        l_spz.set_data(t[:k], -sp[:k, 2]); l_pz.set_data(t[:k], -p[:k, 2])
        err = float(np.linalg.norm(p[k - 1, :2] - sp[k - 1, :2]))
        fig.suptitle(f"tick {k - 1}   t={t[k - 1]:.1f}s   XY err {err:.2f} m")
        writer.grab_frame()
print(f"-- wrote {out} ({(n // a.stride)} frames @ {fps} fps)")
