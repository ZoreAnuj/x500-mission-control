#!/usr/bin/env python3
"""Calibrate the ESP32-CAM fisheye against a checkerboard shown ON THIS SCREEN.

The script draws the 9x6 (inner-corner) checkerboard in a window itself, so you just
point the drone camera at the screen and move it around — it AUTO-GRABS diverse views
(you don't have to press a key for each). Writes calib.npz (K, D) for esp32cam_capture.py.

  python calibrate_fisheye.py                     # board on screen + live grab from stream
  python calibrate_fisheye.py --manual            # press SPACE to grab instead of auto
  python calibrate_fisheye.py --square 100        # smaller board (if it doesn't fit your screen)

Move the camera so the board fills different parts of the frame — especially the CORNERS,
where the fisheye distortion is worst — at several angles and distances.
Keys:  C = calibrate now (>=10 views)   R = reset grabs   Q = quit
IMPORTANT: the board is shown at 1:1 pixels (not stretched) so the squares stay square —
do NOT fullscreen-stretch it, or the calibration will bake in a false aspect ratio.
"""
import argparse
import threading
import time

import cv2
import numpy as np

# background stream reader so a dead/unreachable stream never blocks the board window
_stop = threading.Event()
_frame = {"img": None, "t": 0.0}


def reader(url):
    while not _stop.is_set():
        cap = cv2.VideoCapture(url)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            cap.release(); time.sleep(1.0); continue
        while not _stop.is_set():
            ok, fr = cap.read()
            if not ok or fr is None:
                break                      # stream dropped -> reopen
            _frame["img"], _frame["t"] = fr, time.time()
        cap.release(); time.sleep(0.5)

CB = (9, 6)          # inner corners (columns, rows) — a 10x7-square board
W, H = 320, 240      # sim front-camera resolution

objp = np.zeros((1, CB[0] * CB[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CB[0], 0:CB[1]].T.reshape(-1, 2)
SUBPIX = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# OpenCV 5 moved these fisheye flags to the top-level cv2 namespace; 4.x had them
# under cv2.fisheye. Resolve from wherever they live so this works on both.
FISHEYE_FLAGS = ((getattr(cv2.fisheye, "CALIB_RECOMPUTE_EXTRINSIC", 0) or cv2.CALIB_RECOMPUTE_EXTRINSIC)
                 | (getattr(cv2.fisheye, "CALIB_FIX_SKEW", 0) or cv2.CALIB_FIX_SKEW))


def make_board(square, cols=10, rows=7, bar_h=48):
    """Checkerboard (cols x rows squares = (cols-1)x(rows-1) inner corners) with a
    1-square white quiet zone, plus a black status bar on top for the operator."""
    margin = square
    bw, bh = cols * square + 2 * margin, rows * square + 2 * margin
    board = np.full((bh, bw), 255, np.uint8)
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                board[margin + r * square:margin + (r + 1) * square,
                      margin + c * square:margin + (c + 1) * square] = 0
    board = cv2.cvtColor(board, cv2.COLOR_GRAY2BGR)
    bar = np.zeros((bar_h, bw, 3), np.uint8)
    return np.vstack([bar, board]), bar_h


def novel(corners, grabbed, min_move):
    """True if this view moved > min_move px (mean per-corner) vs EVERY prior grab."""
    return all(np.linalg.norm(corners - g, axis=-1).mean() > min_move for g in grabbed)


def _run_fisheye(op, ip, shape):
    K, D = np.zeros((3, 3)), np.zeros((4, 1))
    rms, _, _, _, _ = cv2.fisheye.calibrate(op, ip, shape, K, D, flags=FISHEYE_FLAGS)
    return rms, K, D


def calibrate(objpts, imgpts, shape):
    # OpenCV 5's fisheye.calibrate needs BOTH object and image points as (1, N, C).
    op = [np.asarray(o, np.float32).reshape(1, -1, 3) for o in objpts]
    ip = [np.asarray(c, np.float32).reshape(1, -1, 2) for c in imgpts]
    sel, dropped, result = list(range(len(op))), [], None
    # A single degenerate/mis-detected view makes the whole solve throw (InitExtrinsics).
    # Greedily drop the offending view(s) until it converges or too few remain.
    while len(sel) >= 8:
        try:
            result = _run_fisheye([op[i] for i in sel], [ip[i] for i in sel], shape)
            break
        except cv2.error:
            unblock = next((i for i in sel if _ok_without(op, ip, shape, sel, i)), sel[-1])
            sel.remove(unblock); dropped.append(unblock)
            print(f"  dropped a degenerate view ({len(sel)} left)", flush=True)
    if result is None:
        print("!! calibrate failed — views too similar. TILT the camera at steeper, more varied "
              "angles to the screen (not head-on) and re-grab (R then re-capture).", flush=True)
        return None
    rms, K, D = result
    np.savez("calib.npz", K=K, D=D)
    hfov = 2 * np.degrees(np.arctan((W / 2) / K[0, 0]))
    note = f"  (used {len(sel)}/{len(op)} views)" if dropped else ""
    print(f"\nsaved calib.npz  rms={rms:.3f}px  measured HFOV~{hfov:.0f}deg{note}\nK=\n{K}\nD={D.ravel()}",
          flush=True)
    return rms, hfov


def _ok_without(op, ip, shape, sel, i):
    trial = [j for j in sel if j != i]
    try:
        _run_fisheye([op[j] for j in trial], [ip[j] for j in trial], shape)
        return True
    except cv2.error:
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://192.168.4.1:81/stream")
    ap.add_argument("--target", type=int, default=18, help="auto-grab until this many views")
    ap.add_argument("--square", type=int, default=130, help="board square size in px on screen")
    ap.add_argument("--min-move", type=float, default=18.0, help="px the board must shift to auto-grab")
    ap.add_argument("--manual", action="store_true", help="press SPACE to grab instead of auto")
    a = ap.parse_args()

    FONT = cv2.FONT_HERSHEY_SIMPLEX
    win_board = "calibration BOARD (point the drone cam here)"
    win_feed = "camera FEED (what the drone cam sees)"
    board, bar_h = make_board(a.square)
    cv2.namedWindow(win_board, cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow(win_feed, cv2.WINDOW_AUTOSIZE)
    cv2.imshow(win_board, board)                 # show both windows IMMEDIATELY
    cv2.imshow(win_feed, np.zeros((H * 2, W * 2, 3), np.uint8))
    cv2.waitKey(1)
    print("stream:", a.url, "  (both windows up; feed fills in once the cam is reachable)", flush=True)
    threading.Thread(target=reader, args=(a.url,), daemon=True).start()

    objpts, imgpts, shape = [], [], (W, H)
    last_grab, result = 0.0, None

    while True:
        fr = _frame["img"]
        have_stream = (time.time() - _frame["t"]) < 1.0
        found, corners = False, None
        if fr is not None and have_stream:
            gray = cv2.cvtColor(cv2.resize(fr, (W, H)), cv2.COLOR_BGR2GRAY)
            shape = gray.shape[::-1]
            found, corners = cv2.findChessboardCorners(
                gray, CB, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
            if found:
                corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), SUBPIX)

        # auto-grab a novel, well-spaced view
        now = time.time()
        if (found and not a.manual and result is None and len(objpts) < a.target
                and now - last_grab > 0.5 and novel(corners, imgpts, a.min_move)):
            objpts.append(objp); imgpts.append(corners); last_grab = now
            print(f"grabbed {len(objpts)}/{a.target}", flush=True)
            if len(objpts) >= a.target:
                result = calibrate(objpts, imgpts, shape)

        # ---- status text (shared) ----
        if result is None:
            cam = "DETECTED" if found else ("no board in view" if have_stream else "NO STREAM (Ketu wifi + cam on?)")
            msg = f"captured {len(objpts)}/{a.target}   cam: {cam}   " + \
                  ("SPACE=grab " if a.manual else "auto-grab ") + "C=calibrate R=reset Q=quit"
            color = (0, 255, 0) if found else (0, 165, 255)
        else:
            rms, hfov = result
            msg = f"DONE  rms={rms:.2f}px  HFOV~{hfov:.0f}deg  -> calib.npz written   Q=quit R=recalibrate"
            color = (0, 255, 255)

        # ---- board window (clean pattern + status bar) ----
        bdisp = board.copy()
        cv2.putText(bdisp, msg, (12, bar_h - 16), FONT, 0.6, color, 2)
        cv2.imshow(win_board, bdisp)

        # ---- camera feed window (what the cam sees, corners highlighted) ----
        if fr is not None and have_stream:
            view = cv2.resize(fr, (W, H))
            if found:
                cv2.drawChessboardCorners(view, CB, corners, True)
        else:
            view = np.zeros((H, W, 3), np.uint8)
            cv2.putText(view, "NO STREAM", (95, 110), FONT, 0.7, (0, 80, 255), 2)
            cv2.putText(view, "join Ketu wifi + power cam", (28, 138), FONT, 0.4, (0, 140, 255), 1)
        view = cv2.resize(view, (W * 2, H * 2), interpolation=cv2.INTER_NEAREST)
        cv2.putText(view, msg, (8, 20), FONT, 0.42, color, 1)
        cv2.imshow(win_feed, view)

        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        elif k == ord(' ') and found and result is None:
            objpts.append(objp); imgpts.append(corners)
            print(f"grabbed {len(objpts)}", flush=True)
        elif k == ord('c') and len(objpts) >= 10:
            result = calibrate(objpts, imgpts, shape)
        elif k == ord('r'):
            objpts, imgpts, result = [], [], None
            print("reset grabs", flush=True)

    _stop.set()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
