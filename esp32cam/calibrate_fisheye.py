#!/usr/bin/env python3
"""Calibrate the ESP32-CAM fisheye so esp32cam_capture.py can remap it to the sim's 80-deg
pinhole. Print a checkerboard (default 9x6 INNER corners), hold it at many angles/distances
filling the frame corners (where fisheye distortion is worst). Writes calib.npz (K, D).

  python calibrate_fisheye.py                       # live grab from the stream
  keys: SPACE = grab a view (only when corners are highlighted), C = calibrate (>=10 views), Q = quit
"""
import argparse

import cv2
import numpy as np

CB = (9, 6)          # inner corners (columns, rows) — change to match your printed board
W, H = 320, 240

objp = np.zeros((1, CB[0] * CB[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CB[0], 0:CB[1]].T.reshape(-1, 2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://192.168.4.1:81/stream")
    a = ap.parse_args()
    cap = cv2.VideoCapture(a.url)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    objpts, imgpts, shape = [], [], (W, H)

    while True:
        ok, fr = cap.read()
        if not ok:
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break
            continue
        fr = cv2.resize(fr, (W, H))
        gray = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY)
        shape = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(
            gray, CB, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        disp = fr.copy()
        if found:
            cv2.drawChessboardCorners(disp, CB, corners, found)
        cv2.putText(disp, f"grabbed {len(objpts)}  SPACE=grab C=calibrate Q=quit",
                    (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        cv2.imshow("fisheye calibration", disp)
        k = cv2.waitKey(1) & 0xFF
        if k == ord(' ') and found:
            objpts.append(objp)
            imgpts.append(corners)
            print("grabbed", len(objpts))
        elif k == ord('c') and len(objpts) >= 10:
            K = np.zeros((3, 3))
            D = np.zeros((4, 1))
            rms, _, _, _, _ = cv2.fisheye.calibrate(
                objpts, imgpts, shape, K, D,
                flags=cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW)
            np.savez("calib.npz", K=K, D=D)
            hfov = 2 * np.degrees(np.arctan((W / 2) / K[0, 0]))
            print(f"saved calib.npz  rms={rms:.3f}  measured HFOV~{hfov:.0f} deg\nK=\n{K}\nD={D.ravel()}")
        elif k == ord('q'):
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
