#!/usr/bin/env python3
r"""Classical-CV autonomous hoop pass (no learned policy, no ground truth).

TAKEOFF -> SCAN (yaw) -> CENTER (yaw+climb servo) -> APPROACH (creep forward,
keep centered) -> BLIND dash through the hoop -> LAND.

Detection: HSV color threshold (2-band red default, tuned on the sim hoop probe)
-> morphology -> largest contour -> fitEllipse -> gates -> EMA track. Range from
known geometry: d ~ F_PX * HOOP_OUTER_M / major_px.

Control: SET_POSITION_TARGET_LOCAL_NED in MAV_FRAME_BODY_NED — body velocities
[vx fwd, 0, vz] + yaw_rate. Velocity targets are NOT latched: if this script or
the camera dies, ArduCopter stops and hovers after GUID_TIMEOUT (~3 s).

  # on-site color calibration (no flight):
  python cv_hoop_pass.py --tune --cam-url http://192.168.4.1:81/stream
  # SITL smoke (dataset video as camera, inside WSL):
  python3 cv_hoop_pass.py --connect tcp:127.0.0.1:5760 --force --yes --cam-video <ep.mp4>
  # real flight:
  python cv_hoop_pass.py --connect COM13 --cam-url http://192.168.4.1:81/stream
"""
import argparse
import csv
import math
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from field_replay import (STOP, stop_watcher, init_st, connect, set_failsafe_norc,
                          set_stream_rates, poll, beat, preflight_gates, set_mode,
                          wait_ekf_ready, arm, takeoff, land_and_disarm)
from pymavlink import mavutil

W, H = 320, 240
CX, CY = W / 2, H / 2
F_PX = 143.0                  # rectilinear focal (sim CameraFront VFOV80; undistort target)
HOOP_OUTER_M = 1.092

# HSV defaults measured on the REAL hoop photo + sim probe. The real paint is true red
# H~178 / S~172; brown wood-floor false positives sit at H~15 / S<=104 -> S floor 90
# separates them (kept a touch low vs the mat's p95 for outdoor specular desaturation).
# Sim maroon: H~0-8, S high, V floor ~25. Always sanity-check with --tune on site.
HSV1_LO, HSV1_HI = (0, 90, 25), (10, 255, 255)
HSV2_LO, HSV2_HI = (168, 90, 25), (180, 255, 255)

# detector gates / tracking
MIN_AREA_PX = 15
MAX_ASPECT = 3.5
EMA = 0.5                     # per-frame smoothing on (cx, cy, major)
N_ACQ = 3                     # consecutive detections to acquire
LOST_S = 3.0                  # seconds without detection -> lost

# servo gains / limits
K_PSI = 0.006                 # rad/s per px of horizontal error
K_Z = 0.004                   # m/s per px of vertical error
YAWRATE_MAX = 0.6             # rad/s
VZ_MAX = 0.5                  # m/s
CENTER_PX = 20                # |err| below this = centered
CENTER_STABLE_S = 1.0
BLIND_RANGE_M = 1.2           # est. range to switch to the blind dash
LOST_BIG_M = 1.8              # lost while closer than this also triggers the dash
BLIND_S = 4.0                 # dash duration (pass + cruise a few secs)
SCAN_YAWRATE = math.radians(20)
SCAN_TIMEOUT_S = 30.0
MAX_RADIUS_M = 12.0
MAX_ALT_M = 4.5

# type_mask: set bit = IGNORE. Use vel(3,4,5) + yaw_rate(11); ignore pos/accel/force/yaw.
USE_VEL_YAWRATE = 7 | (7 << 6) | (1 << 9) | (1 << 10)      # 1991


def send_body_vel(m, vx, vz, yaw_rate):
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, USE_VEL_YAWRATE,
        0, 0, 0, vx, 0.0, vz, 0, 0, 0, 0, yaw_rate)


# ---- camera sources (return BGR 320x240, newest frame) ----

class MJPEGCam:
    def __init__(self, url, calib_path):
        c = np.load(calib_path)
        K_new = np.array([[F_PX, 0, CX], [0, F_PX, CY], [0, 0, 1]])
        self.m1, self.m2 = cv2.fisheye.initUndistortRectifyMap(
            c["K"], c["D"], np.eye(3), K_new, (W, H), cv2.CV_16SC2)
        self.cap = cv2.VideoCapture(url)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.frame = None
        self.lock = threading.Lock()
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            ok, fr = self.cap.read()
            if ok:
                with self.lock:
                    self.frame = fr

    def read(self):
        with self.lock:
            fr = self.frame
        if fr is None:
            return None
        return cv2.remap(fr, self.m1, self.m2, cv2.INTER_LINEAR)


class VideoSrc:
    """Dataset mp4 for desk tests; steps `stride` frames per read, holds last at EOF."""
    def __init__(self, path, stride):
        self.cap = cv2.VideoCapture(str(path))
        if not self.cap.isOpened():
            raise SystemExit(f"cannot open {path}")
        self.stride = max(1, stride)
        self.last = None

    def read(self):
        for _ in range(self.stride):
            ok, fr = self.cap.read()
            if not ok:
                break
            self.last = fr
        return self.last


# ---- detector ----

class HoopDetector:
    def __init__(self, lo1, hi1, lo2, hi2, outer_m=HOOP_OUTER_M):
        self.outer_m = outer_m
        self.b = (np.array(lo1), np.array(hi1), np.array(lo2), np.array(hi2))
        self.cx = self.cy = self.major = None
        self.hits = 0
        self.last_seen = 0.0
        self.kernel = np.ones((5, 5), np.uint8)

    def mask(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        lo1, hi1, lo2, hi2 = self.b
        m = cv2.inRange(hsv, lo1, hi1) | cv2.inRange(hsv, lo2, hi2)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # kill speckle
        return cv2.morphologyEx(m, cv2.MORPH_CLOSE, self.kernel)

    def detect(self, bgr):
        """Returns (detected, err_x, err_y, major_px, range_m).

        Fits the ellipse over the UNION of red contours near the largest one: the real
        hoop's dark corner brackets + mount gap break the ring into arcs, and a
        single-contour fit on a 'C' shape badly underestimates size/center."""
        m = self.mask(bgr)
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = [c for c in cnts if cv2.contourArea(c) >= MIN_AREA_PX and len(c) >= 5]
        got = False
        if cnts:
            big = max(cnts, key=cv2.contourArea)
            area_big = cv2.contourArea(big)
            bx, by, bw, bh = cv2.boundingRect(big)
            bcx, bcy = bx + bw / 2, by + bh / 2
            r_near = 1.6 * math.hypot(bw, bh)          # gathers ring arcs, rejects far blobs
            join_min = max(2 * MIN_AREA_PX, 0.02 * area_big)   # speckle can't join the fit
            pts = [big.reshape(-1, 2)]
            for c in cnts:
                if c is big or cv2.contourArea(c) < join_min:
                    continue
                x, y, w_, h_ = cv2.boundingRect(c)
                if math.hypot(x + w_ / 2 - bcx, y + h_ / 2 - bcy) < r_near:
                    pts.append(c.reshape(-1, 2))
            u = np.vstack(pts).astype(np.float32)
            if len(u) >= 5:
                (x, y), (MA, ma), _ = cv2.fitEllipse(u)
                major, minor = max(MA, ma), max(min(MA, ma), 1e-3)
                if major / minor <= MAX_ASPECT or major > 0.5 * W:   # partial views distort
                    if self.cx is None:
                        self.cx, self.cy, self.major = x, y, major
                    else:
                        a = EMA
                        self.cx = a * x + (1 - a) * self.cx
                        self.cy = a * y + (1 - a) * self.cy
                        self.major = a * major + (1 - a) * self.major
                    got = True
        if got:
            self.hits += 1
            self.last_seen = time.monotonic()
        else:
            self.hits = 0
        if self.cx is None:
            return False, 0.0, 0.0, 0.0, float("inf")
        rng = F_PX * self.outer_m / max(self.major, 1e-3)
        return got, self.cx - CX, self.cy - CY, self.major, rng

    @property
    def acquired(self):
        return self.hits >= N_ACQ

    def lost_for(self):
        return time.monotonic() - self.last_seen if self.last_seen else float("inf")


def tune(cam):
    """Live HSV trackbars; prints the final CLI string. q quits."""
    win = "tune (q=quit)"
    cv2.namedWindow(win)
    for name, val, mx in (("H1_hi", 10, 40), ("H2_lo", 168, 180),
                          ("S_min", 90, 255), ("V_min", 25, 255)):
        cv2.createTrackbar(name, win, val, mx, lambda _: None)
    while True:
        fr = cam.read()
        if fr is None:
            time.sleep(0.1)
            continue
        h1 = cv2.getTrackbarPos("H1_hi", win)
        h2 = cv2.getTrackbarPos("H2_lo", win)
        s = cv2.getTrackbarPos("S_min", win)
        v = cv2.getTrackbarPos("V_min", win)
        det = HoopDetector((0, s, v), (h1, 255, 255), (h2, s, v), (180, 255, 255))
        got, ex, ey, major, rng = det.detect(fr)
        vis = fr.copy()
        if det.cx is not None:
            cv2.circle(vis, (int(det.cx), int(det.cy)), 4, (0, 255, 255), -1)
            cv2.putText(vis, f"maj={major:.0f}px d={rng:.1f}m", (4, 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.imshow(win, np.hstack([vis, cv2.cvtColor(det.mask(fr), cv2.COLOR_GRAY2BGR)]))
        if cv2.waitKey(30) & 0xFF == ord("q"):
            print(f"--hsv {h1},{h2},{s},{v}")
            break
    cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", default="COM13")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--cam-url", default=None)
    ap.add_argument("--cam-video", default=None)
    ap.add_argument("--calib", default=str(Path(__file__).resolve().parent.parent
                                           / "esp32cam" / "calib.npz"))
    ap.add_argument("--hsv", default=None, metavar="H1HI,H2LO,SMIN,VMIN",
                    help="from --tune (default 20,168,50,25)")
    ap.add_argument("--hoop-dia", type=float, default=HOOP_OUTER_M,
                    help="real hoop OUTER diameter m (drives the range estimate)")
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--alt", type=float, default=1.5)
    ap.add_argument("--vapp", type=float, default=0.5, help="approach speed m/s")
    ap.add_argument("--max-secs", type=float, default=120.0)
    ap.add_argument("--out", default="cv_hoop_run.csv")
    ap.add_argument("--tune", action="store_true", help="HSV calibration only, no flight")
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--yes", action="store_true")
    a = ap.parse_args()
    if bool(a.cam_url) == bool(a.cam_video):
        sys.exit("pick exactly one: --cam-url or --cam-video")

    cam = (MJPEGCam(a.cam_url, a.calib) if a.cam_url
           else VideoSrc(a.cam_video, stride=max(1, round(30.0 / a.hz))))
    if a.tune:
        tune(cam)
        return
    if a.hsv:
        h1, h2, s, v = (int(x) for x in a.hsv.split(","))
        det = HoopDetector((0, s, v), (h1, 255, 255), (h2, s, v), (180, 255, 255),
                           outer_m=a.hoop_dia)
    else:
        det = HoopDetector(HSV1_LO, HSV1_HI, HSV2_LO, HSV2_HI, outer_m=a.hoop_dia)

    print("-- waiting for camera...")
    t0 = time.time()
    while cam.read() is None:
        if time.time() - t0 > 15:
            sys.exit("no camera frames")
        time.sleep(0.2)

    st = init_st()
    m = connect(a.connect, a.baud)
    set_stream_rates(m)
    set_failsafe_norc(m)
    if not a.force:
        if not preflight_gates(m, st):
            sys.exit("ABORT: preflight gates failed")
    else:
        print("!! --force: skipping preflight gates")
    if not wait_ekf_ready(m, st):
        sys.exit("ABORT: no EKF position")
    if not set_mode(m, st, "GUIDED"):
        sys.exit("GUIDED failed")
    if not a.yes:
        print("\n*** Type GO to arm + takeoff + hunt the hoop: ***")
        if input().strip().upper() != "GO":
            sys.exit("aborted")
    threading.Thread(target=stop_watcher, daemon=True).start()
    print("\n>>>>>>  PRESS ENTER AT ANY TIME -> LAND  <<<<<<\n")
    if not arm(m, st):
        sys.exit("arm failed")
    print(f"-- takeoff -> {a.alt} m")
    if not takeoff(m, st, a.alt):
        land_and_disarm(m, st)
        sys.exit("takeoff failed -> landed")
    for _ in range(20):
        beat(m)
        poll(m, st)
        time.sleep(0.1)

    try:
        run(m, st, cam, det, a)
    finally:
        land_and_disarm(m, st)


def run(m, st, cam, det, a):
    dt = 1.0 / a.hz
    take = st["pos"].copy() if st["pos"] is not None else np.zeros(3)
    state, t_state = "SCAN", time.monotonic()
    center_since = None
    blind_until = None
    rescans = 0
    last_rng = float("inf")
    reason = "max_secs"
    f = open(a.out, "w", newline="")
    w = csv.writer(f)
    w.writerow(["tick", "t", "state", "det", "err_x", "err_y", "major", "range_m",
                "vx", "vz", "yawrate", "p_n", "p_e", "p_d", "yaw"])
    t0 = time.monotonic()
    k = 0
    print(f"-- HUNT @ {a.hz:g} Hz (ENTER=land) -> {a.out}")
    while time.monotonic() - t0 < a.max_secs:
        tick = time.monotonic()
        if STOP.is_set():
            reason = "ENTER"
            break
        beat(m)
        poll(m, st)
        fr = cam.read()
        if fr is None:
            time.sleep(dt / 4)
            continue
        got, ex, ey, major, rng = det.detect(fr)
        if got:
            last_rng = rng
        vx = vz = yr = 0.0

        if state == "SCAN":
            yr = SCAN_YAWRATE
            if det.acquired:
                state, t_state = "CENTER", tick
                print(f"  [{tick-t0:5.1f}s] ACQUIRED err=({ex:+.0f},{ey:+.0f})px d~{rng:.1f}m")
            elif tick - t_state > SCAN_TIMEOUT_S:
                reason = "scan_timeout"
                break
        elif state in ("CENTER", "APPROACH"):
            if det.lost_for() > LOST_S:
                if state == "APPROACH" and last_rng < LOST_BIG_M:
                    state, blind_until = "BLIND", tick + BLIND_S     # lost because it's huge
                    print(f"  [{tick-t0:5.1f}s] lost-near (d~{last_rng:.1f}m) -> BLIND dash")
                elif rescans < 1:
                    rescans += 1
                    det.last_seen = 0.0
                    state, t_state = "SCAN", tick
                    print(f"  [{tick-t0:5.1f}s] hoop lost -> re-SCAN")
                else:
                    reason = "lost_twice"
                    break
            else:
                yr = float(np.clip(-K_PSI * ex, -YAWRATE_MAX, YAWRATE_MAX))
                vz = float(np.clip(K_Z * ey, -VZ_MAX, VZ_MAX))       # ey<0 (high) -> climb
                centered = abs(ex) < CENTER_PX and abs(ey) < CENTER_PX
                if state == "CENTER":
                    if centered:
                        center_since = center_since or tick
                        if tick - center_since > CENTER_STABLE_S:
                            state, t_state = "APPROACH", tick
                            print(f"  [{tick-t0:5.1f}s] CENTERED d~{rng:.1f}m -> APPROACH")
                    else:
                        center_since = None
                else:                                                # APPROACH
                    fade = max(0.0, 1 - max(abs(ex), abs(ey)) / 60.0)
                    vx = min(a.vapp * fade, 0.4 * max(rng, 0.5))     # taper as it nears
                    if got and rng < BLIND_RANGE_M:
                        state, blind_until = "BLIND", tick + BLIND_S
                        print(f"  [{tick-t0:5.1f}s] d={rng:.1f}m < {BLIND_RANGE_M} -> BLIND dash")
        elif state == "BLIND":
            vx, vz, yr = a.vapp, 0.0, 0.0
            if tick >= blind_until:
                reason = "passed"
                break

        send_body_vel(m, vx, vz, yr)
        w.writerow([k, round(tick - t0, 3), state, int(got), round(ex, 1), round(ey, 1),
                    round(major, 1), round(min(rng, 99), 2), round(vx, 3), round(vz, 3),
                    round(yr, 3), *np.round(st["pos"], 3), round(st["att"][2], 4)])
        f.flush()
        # guards
        rad = math.hypot(st["pos"][0] - take[0], st["pos"][1] - take[1])
        if rad > MAX_RADIUS_M or -st["pos"][2] > MAX_ALT_M:
            reason = f"guard(radius={rad:.1f},alt={-st['pos'][2]:.1f})"
            break
        k += 1
        lag = t0 + k * dt - time.monotonic()
        if lag > 0:
            time.sleep(lag)
    f.close()
    print(f"\n-- ended [{reason}] after {k} ticks; landing")


if __name__ == "__main__":
    main()
