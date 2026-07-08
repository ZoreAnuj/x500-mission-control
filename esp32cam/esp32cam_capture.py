#!/usr/bin/env python3
"""Live view of the ESP32-CAM MJPEG stream, side-by-side with the FOV-calibrated feed
that matches the sim's ~80-deg rectilinear 320x240 front camera (for the Drone Hoop policy).

  python esp32cam_capture.py                 # window: RAW | CALIBRATED side-by-side (default)
  python esp32cam_capture.py --undistort     # single window, calibrated only (needs calib.npz)
  python esp32cam_capture.py --crop          # single window, quick center-crop only (no calib)
  python esp32cam_capture.py --raw           # single window, raw only

Calibrate first with calibrate_fisheye.py (writes calib.npz). Until then the "calibrated"
side falls back to a center-crop approximation and says so on-screen.
"""
import argparse
import threading
import time

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


def calibrated(fr, map1, map2):
    """Best available FOV-matched view: real undistort if calibrated, else a labeled crop approx."""
    if map1 is not None:
        return cv2.remap(fr, map1, map2, cv2.INTER_LINEAR), "CALIBRATED (undistorted 80deg)"
    ch, cw = int(H * 0.5), int(W * 0.5)          # keep central ~50% (approx 80-deg of a 160-deg lens)
    y0, x0 = (H - ch) // 2, (W - cw) // 2
    out = cv2.resize(fr[y0:y0 + ch, x0:x0 + cw], (W, H))
    return out, "CROP APPROX (no calib.npz yet)"


def label(img, text, color=(0, 255, 0)):
    out = img.copy()
    cv2.rectangle(out, (0, 0), (W, 18), (0, 0, 0), -1)
    cv2.putText(out, text, (4, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://192.168.4.1:81/stream")
    ap.add_argument("--undistort", action="store_true", help="single window: calibrated only")
    ap.add_argument("--crop", action="store_true", help="single window: center-crop only")
    ap.add_argument("--raw", action="store_true", help="single window: raw only")
    a = ap.parse_args()
    single = a.undistort or a.crop or a.raw
    if a.undistort and not _CALIB:
        print("WARNING: no calib.npz -> using placeholder intrinsics; run calibrate_fisheye.py")

    cam = LatestFrame(a.url)
    map1 = map2 = None
    if _CALIB:
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            K, D, np.eye(3), K_NEW, (W, H), cv2.CV_16SC2)

    win = "esp32cam" if single else "esp32cam: RAW | CALIBRATED (q=quit)"
    t_last, fps = time.time(), 0.0
    while True:
        fr = cam.read()
        if fr is None:
            print("waiting for frames from", a.url, "...")
            if cv2.waitKey(500) & 0xFF == ord('q'):
                break
            continue
        fr = cv2.resize(fr, (W, H))
        now = time.time()
        fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_last, 1e-3))
        t_last = now

        if a.raw:
            out = label(fr, f"RAW  {fps:4.1f} fps")
        elif a.undistort:
            calib_fr, tag = calibrated(fr, map1, map2)
            out = label(calib_fr, f"{tag}  {fps:4.1f} fps")
        elif a.crop:
            calib_fr, _ = calibrated(fr, None, None)   # force crop-approx path
            out = label(calib_fr, f"CROP APPROX  {fps:4.1f} fps")
        else:
            calib_fr, tag = calibrated(fr, map1, map2)
            left = label(fr, f"RAW  {fps:4.1f} fps")
            right = label(calib_fr, tag, (0, 200, 255) if not _CALIB else (0, 255, 0))
            out = np.hstack([left, right])
        cv2.imshow(win, out)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
