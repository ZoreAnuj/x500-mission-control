#!/usr/bin/env python3
"""Live preview of a running sitl_replay.py: tails the CSV and animates
commanded vs flown. Run on Windows while the replay runs in WSL.

  python watch_replay.py runs/replay_ep000.csv
"""
import argparse
import os
import time

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

ap = argparse.ArgumentParser()
ap.add_argument("csv")
ap.add_argument("--interval", type=int, default=400, help="refresh ms")
a = ap.parse_args()

print(f"-- watching {a.csv} (close the window to stop)")
while not os.path.exists(a.csv):
    time.sleep(0.3)

fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(11, 5.5))
fig.canvas.manager.set_window_title("replay preview")
(l_sp,) = ax0.plot([], [], "--", color="tab:orange", label="commanded")
(l_p,) = ax0.plot([], [], color="tab:green", label="flown")
(dot,) = ax0.plot([], [], "o", color="tab:green", ms=8)
ax0.set_xlabel("East (m)"); ax0.set_ylabel("North (m)")
ax0.legend(loc="upper left"); ax0.set_title("top-down"); ax0.grid(alpha=0.3)
(l_spz,) = ax1.plot([], [], "--", color="tab:orange", label="commanded")
(l_pz,) = ax1.plot([], [], color="tab:green", label="flown")
ax1.set_xlabel("t (s)"); ax1.set_ylabel("up (m)")
ax1.legend(loc="upper left"); ax1.set_title("altitude (rel)"); ax1.grid(alpha=0.3)


def update(_):
    try:
        df = pd.read_csv(a.csv, on_bad_lines="skip")
    except Exception:
        return
    if len(df) < 2:
        return
    sp = df[["sp_n", "sp_e", "sp_d"]].to_numpy() - df[["sp_n", "sp_e", "sp_d"]].to_numpy()[0]
    p = df[["p_n", "p_e", "p_d"]].to_numpy() - df[["p_n", "p_e", "p_d"]].to_numpy()[0]
    l_sp.set_data(sp[:, 1], sp[:, 0]); l_p.set_data(p[:, 1], p[:, 0])
    dot.set_data([p[-1, 1]], [p[-1, 0]])
    l_spz.set_data(df["t"], -sp[:, 2]); l_pz.set_data(df["t"], -p[:, 2])
    for ax in (ax0, ax1):
        ax.relim(); ax.autoscale_view()
    ax0.set_aspect("equal", adjustable="datalim")
    err = float(np.linalg.norm(p[-1, :2] - sp[-1, :2]))
    fig.suptitle(f"tick {int(df['tick'].iloc[-1])}   t={df['t'].iloc[-1]:.1f}s   "
                 f"XY err {err:.2f} m")


ani = FuncAnimation(fig, update, interval=a.interval, cache_frame_data=False)
plt.show()
