#!/usr/bin/env python3
"""Grab the NEWEST frame from the ESP32-CAM MJPEG stream and remap the 160-deg fisheye
to the sim's ~80-deg rectilinear 320x240 front camera, ready to feed the Drone Hoop policy.

  python esp32cam_capture.py                 # raw stream
  python esp32cam_capture.py --undistort     # fisheye -> 80-deg pinhole (needs calib.npz)
  python esp32cam_capture.py --crop          # quick center-crop ~80-deg (no calibration)

Calibrate first with calibrate_fisheye.py (writes calib.npz). Until then --undistort
falls back to a rough placeholder and prints a warning.
"""
import argparse
import threading

import cv2
import numpy as np

# --- sim front-camera target: 320x240, 80-deg HFOV, pinhole ---
W, H, HFOV_DEG = 320, 240, 80.0
f = (W / 2) / np.tan(np.radians(HFOV_DEG) / 2)          # ~191 px
K_NEW = np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1]])

# --- fisheye intrinsics: from calibrate_fisheye.py, else a rough placeholder ---
try:
    _c = np.load("calib.npz")
    K, D = _c["K"], _c["D"]
    _CALIB = True
except FileNotFoundError:
    K = np.array([[150., 0., W / 2], [0., 150., H / 2], [0., 0., 1.]])
    D = np.zeros((4, 1))
    _CALIB = False


class LatestFrame:
    """Continuously drain the stream so read() always returns the freshest frame (no buffer lag)."""
    def __init__(self, url):
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
            return None if self.frame is None else self.frame.copy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://192.168.4.1:81/stream")
    ap.add_argument("--undistort", action="store_true", help="fisheye -> 80-deg pinhole (calib.npz)")
    ap.add_argument("--crop", action="store_true", help="center-crop ~50%% to ~80-deg (approx)")
    a = ap.parse_args()
    if a.undistort and not _CALIB:
        print("WARNING: no calib.npz -> using placeholder intrinsics; run calibrate_fisheye.py")

    cam = LatestFrame(a.url)
    map1 = map2 = None
    if a.undistort:
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            K, D, np.eye(3), K_NEW, (W, H), cv2.CV_16SC2)

    while True:
        fr = cam.read()
        if fr is None:
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break
            continue
        fr = cv2.resize(fr, (W, H))
        if a.undistort and map1 is not None:
            out = cv2.remap(fr, map1, map2, cv2.INTER_LINEAR)      # rectilinear 80-deg = sim match
        elif a.crop:
            ch, cw = int(H * 0.5), int(W * 0.5)                    # keep central ~50% (approx 80-deg)
            y0, x0 = (H - ch) // 2, (W - cw) // 2
            out = cv2.resize(fr[y0:y0 + ch, x0:x0 + cw], (W, H))
        else:
            out = fr
        cv2.imshow("esp32cam -> policy 320x240", out)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
