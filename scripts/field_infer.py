#!/usr/bin/env python3
r"""X500 policy inference flight — CMEncIMLE ep010 closing the loop on a real drone.

field_replay.py skeleton (gates, no-RC failsafe, takeoff, ENTER=land kill, NAV_LAND)
with the action source swapped from a recorded episode to live policy inference, and a
camera source feeding the vision input:

  --cam-url    ESP32-S3 MJPEG stream; fisheye frames are remapped in ONE step to the
               sim CameraFront geometry squashed to the policy input: 128x128 with
               fx=64/tan(48.13deg)~57.3, fy=64/tan(40deg)~76.3  (sim = VFOV 80deg,
               HFOV 96.3deg at 4:3, then 4:3->1:1 resize squash -- fold it all in).
  --cam-video  a dataset mp4 (raw session episode) -- SITL dry-run mode; frames are
               already sim-domain 320x240, so just resize 128x128 like live LE did.

Handoff mirrors the sim play scripts: takeoff to 1.4 m (sim: ramp to z=-1.38 NED),
settle, then the policy takes over. Point the drone AT the hoop before arming (the sim
yawed toward ground-truth hoop NED; real flight has none).

  # SITL dry run (SITL in WSL on tcp:5760, dataset video as camera):
  python field_infer.py --connect tcp:127.0.0.1:5760 --force --yes \
      --cam-video "D:/temp/LuckyEngine/LuckyEditor/Captures/DataSessions/session_2026-05-15_20-19-53/videos/observation.images.CameraFront/chunk-000/file-000.mp4"

  # real flight (ESP32 cam on WiFi Ketu, SiK on COM13):
  python field_infer.py --connect COM13 --cam-url http://192.168.4.1:81/stream
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

# proven MAVLink/flight primitives + shim constants from the replay harness
from field_replay import (LOOKAHEAD_S, CLIP_POS, CLIP_YAW, VMAX,
                          MAX_TRACK_ERR, MAX_RADIUS, GUARD_SECS,
                          STOP, stop_watcher, wrap, init_st, connect,
                          set_failsafe_norc, set_stream_rates, poll, send_target,
                          preflight_gates, set_mode, wait_ekf_ready, arm, takeoff,
                          land_and_disarm)

import torch
import torchvision.transforms.functional as TF

# policy source lives in the ML repo
for _p in ("D:/lucky_drone_il/scripts", "/mnt/d/lucky_drone_il/scripts"):
    if Path(_p).exists() and _p not in sys.path:
        sys.path.insert(0, _p)

POLICY_HW = 128
_MEAN = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
_STD = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)
NAN_KILL_STREAK = 10


def load_policy(ckpt_path, encoder_path, device):
    """Verbatim recipe from play_cmenc_imle_v1.py (EMA weights preferred)."""
    from train_cmvae_v1 import CMEncoder, LATENT_DIM
    from train_cmenc_imle_v1 import CMEncIMLEPolicy
    dev = torch.device(device if torch.cuda.is_available() or device == "cpu" else "cpu")
    enc_ckpt = torch.load(str(encoder_path), map_location=dev, weights_only=True)
    encoder = CMEncoder(LATENT_DIM)
    encoder.load_state_dict({k.replace("_orig_mod.", ""): v
                             for k, v in enc_ckpt["model"].items()})
    print(f"[encoder] ep{enc_ckpt['epoch']} val={enc_ckpt['val_loss']:.5f}")
    ckpt = torch.load(str(ckpt_path), map_location=dev, weights_only=True)
    model = CMEncIMLEPolicy(encoder)
    if "ema" in ckpt:
        sd = model.state_dict()
        for k, v in ckpt["ema"].items():
            if k in sd:
                sd[k] = v
        model.load_state_dict(sd)
        print(f"[policy] EMA weights ep={ckpt.get('epoch','?')}")
    else:
        model.load_state_dict({k.replace("_orig_mod.", ""): v
                               for k, v in ckpt["model"].items()})
        print(f"[policy] model weights ep={ckpt.get('epoch','?')}")
    model.to(dev).eval()
    return model, dev


def build_state(att, vel):
    """7-D policy state, exact play_cmenc_imle_v1._build_state formula."""
    r, p, y = att
    cy, sy = math.cos(y), math.sin(y)
    vn, ve, vd = float(vel[0]), float(vel[1]), float(vel[2])
    return np.array([r, p, sy, cy, cy * vn + sy * ve, -sy * vn + cy * ve, vd],
                    dtype=np.float32)


# ---- camera sources (both return RGB uint8, newest-frame semantics) ----

class MJPEGCam:
    """ESP32 stream -> one cv2.remap folding undistort + FOV match + squash -> 128x128."""
    def __init__(self, url, calib_path):
        c = np.load(calib_path)
        K, D = c["K"], c["D"]
        fx = (POLICY_HW / 2) / math.tan(math.radians(96.26 / 2))   # ~57.3
        fy = (POLICY_HW / 2) / math.tan(math.radians(80.0 / 2))    # ~76.3
        K_new = np.array([[fx, 0, POLICY_HW / 2], [0, fy, POLICY_HW / 2], [0, 0, 1]])
        self.m1, self.m2 = cv2.fisheye.initUndistortRectifyMap(
            K, D, np.eye(3), K_new, (POLICY_HW, POLICY_HW), cv2.CV_16SC2)
        self.cap = cv2.VideoCapture(url)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.frame = None
        self.lock = threading.Lock()
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            ok, fr = self.cap.read()          # drain continuously -> read() is freshest
            if ok:
                with self.lock:
                    self.frame = fr

    def read(self):
        with self.lock:
            fr = self.frame
        if fr is None:
            return None
        out = cv2.remap(fr, self.m1, self.m2, cv2.INTER_LINEAR)
        return cv2.cvtColor(out, cv2.COLOR_BGR2RGB)              # 128x128 RGB

    already_policy_size = True


class VideoCam:
    """Dataset mp4 for the SITL dry run. Steps `stride` frames per read() so a 30 fps
    recording advances in real time at a 10 Hz policy rate. Holds last frame at EOF."""
    def __init__(self, path, stride):
        self.cap = cv2.VideoCapture(str(path))
        if not self.cap.isOpened():
            raise SystemExit(f"cannot open video {path}")
        self.stride = max(1, stride)
        self.last = None

    def read(self):
        fr = None
        for _ in range(self.stride):
            ok, f = self.cap.read()
            if not ok:
                break
            fr = f
        if fr is not None:
            self.last = fr
        if self.last is None:
            return None
        return cv2.cvtColor(self.last, cv2.COLOR_BGR2RGB)        # 320x240 RGB

    already_policy_size = False


def frame_to_tensor(rgb, already_sized, dev):
    t = torch.from_numpy(np.ascontiguousarray(rgb)).permute(2, 0, 1).float() / 255.0
    if not already_sized:
        t = TF.resize(t, [POLICY_HW, POLICY_HW], antialias=True)  # 4:3 -> 1:1 squash, as trained
    return ((t - _MEAN) / _STD).unsqueeze(0).to(dev)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", default="COM13")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--checkpoint",
                    default="D:/lucky_drone_il/runs/cmenc_imle_v1_128d_checkpoints/cmenc_imle_v1_ep010.pt")
    ap.add_argument("--encoder",
                    default="D:/lucky_drone_il/runs/cmvae_v2_checkpoints_128d/cmvae_v1_best.pt")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--cam-url", default=None, help="ESP32 MJPEG stream URL")
    ap.add_argument("--cam-video", default=None, help="dataset mp4 (SITL dry run)")
    ap.add_argument("--calib", default=str(Path(__file__).resolve().parent.parent
                                           / "esp32cam" / "calib.npz"))
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--alt", type=float, default=1.4, help="takeoff m (sim handoff 1.38)")
    ap.add_argument("--max-secs", type=float, default=30.0)
    ap.add_argument("--ceiling", type=float, default=4.0, help="altitude ceiling m AGL-ish (NED -z)")
    ap.add_argument("--out", default="infer_run.csv")
    ap.add_argument("--force", action="store_true", help="skip preflight gates (SITL/bench)")
    ap.add_argument("--yes", action="store_true", help="skip the GO confirmation (SITL)")
    a = ap.parse_args()
    if bool(a.cam_url) == bool(a.cam_video):
        sys.exit("pick exactly one camera source: --cam-url or --cam-video")

    print("-- loading policy...")
    model, dev = load_policy(a.checkpoint, a.encoder, a.device)
    cam = (MJPEGCam(a.cam_url, a.calib) if a.cam_url
           else VideoCam(a.cam_video, stride=max(1, round(30.0 / a.hz))))
    print("-- waiting for first camera frame...")
    t0 = time.time()
    while cam.read() is None:
        if time.time() - t0 > 15:
            sys.exit("no camera frames after 15 s")
        time.sleep(0.2)
    # warm up inference (CUDA graphs/JIT) and report latency
    fr = cam.read()
    img_t = frame_to_tensor(fr, cam.already_policy_size, dev)
    st_t = torch.from_numpy(build_state((0, 0, 0), np.zeros(3))).unsqueeze(0).to(dev)
    with torch.no_grad():
        t1 = time.time()
        chunk = model(img_t, st_t)
        torch.cuda.synchronize() if dev.type == "cuda" else None
    print(f"-- camera OK, warmup inference {1e3*(time.time()-t1):.0f} ms, "
          f"chunk shape {tuple(chunk.shape)}")

    st = init_st()
    m = connect(a.connect, a.baud)
    set_stream_rates(m)
    set_failsafe_norc(m)

    if not a.force:
        if not preflight_gates(m, st):
            sys.exit("ABORT: preflight gates failed (--force for SITL/bench)")
    else:
        print("!! --force: skipping preflight gates")
    if not wait_ekf_ready(m, st):
        sys.exit("ABORT: no EKF position estimate")
    print("-- GUIDED...")
    if not set_mode(m, st, "GUIDED"):
        sys.exit("GUIDED failed")

    if not a.yes:
        print("\n*** POINT THE DRONE AT THE HOOP. Type GO to arm+takeoff+infer: ***")
        if input().strip().upper() != "GO":
            sys.exit("aborted at GO gate")
    threading.Thread(target=stop_watcher, daemon=True).start()
    print("\n>>>>>>  PRESS ENTER AT ANY TIME -> LAND  <<<<<<\n")

    print("-- arming...")
    if not arm(m, st):
        sys.exit("arm failed")
    print(f"-- takeoff -> {a.alt} m...")
    if not takeoff(m, st, a.alt):
        land_and_disarm(m, st)
        sys.exit("takeoff failed -> landed")
    from field_replay import beat
    for _ in range(20):                      # ~2 s settle at the handoff hover
        beat(m)
        poll(m, st)
        time.sleep(0.1)

    try:
        run_inference(m, st, model, dev, cam, a)
    finally:
        land_and_disarm(m, st)


def run_inference(m, st, model, dev, cam, a):
    from field_replay import beat
    while st["pos"] is None or st["att"] is None:
        beat(m)
        poll(m, st)
        time.sleep(0.05)
    setpt = st["pos"].copy()
    take = st["pos"].copy()
    dt = 1.0 / a.hz
    guard_ticks = max(2, round(GUARD_SECS * a.hz))
    ceiling_ned_z = -abs(a.ceiling)
    print(f"-- POLICY LOOP @ {a.hz:g} Hz, max {a.max_secs:g}s (ENTER=land) -> {a.out}")
    reason, track_bad, nan_streak, k = "max_secs", 0, 0, 0
    t0 = time.monotonic()
    prev_t = t0
    f = open(a.out, "w", newline="")
    w = csv.writer(f)
    w.writerow(["tick", "t", "ax", "ay", "az", "ayaw",
                "sp_n", "sp_e", "sp_d", "v_n", "v_e", "v_d", "yawrate_cmd",
                "p_n", "p_e", "p_d", "vel_n", "vel_e", "vel_d",
                "roll", "pitch", "yaw", "inf_ms"])
    with torch.no_grad():
        while time.monotonic() - t0 < a.max_secs:
            tick_start = time.monotonic()
            if STOP.is_set():
                reason = "ENTER"
                break
            beat(m)
            poll(m, st)
            rgb = cam.read()
            if rgb is None:
                time.sleep(dt / 4)
                continue
            img_t = frame_to_tensor(rgb, cam.already_policy_size, dev)
            state_t = torch.from_numpy(build_state(st["att"], st["vel"])).unsqueeze(0).to(dev)
            t_inf = time.monotonic()
            a_raw = model(img_t, state_t)[0, 0].float().cpu().numpy()
            inf_ms = 1e3 * (time.monotonic() - t_inf)

            if not np.isfinite(a_raw).all():
                nan_streak += 1
                if nan_streak >= NAN_KILL_STREAK:
                    reason = "nan_kill"
                    break
                d = np.zeros(3); dyaw = 0.0; v = np.zeros(3)
            else:
                nan_streak = 0
                d = np.clip(a_raw[:3], -CLIP_POS, CLIP_POS)
                dyaw = float(np.clip(a_raw[3], -CLIP_YAW, CLIP_YAW))
                yaw = st["att"][2]
                cy, sy = math.cos(yaw), math.sin(yaw)
                v = np.array([cy * d[0] - sy * d[1], sy * d[0] + cy * d[1], d[2]]) / LOOKAHEAD_S
                n = float(np.linalg.norm(v))
                if n > VMAX:
                    v *= VMAX / n
            actual_dt = min(tick_start - prev_t, 3 * dt) if k else dt
            prev_t = tick_start
            setpt = setpt + v * actual_dt
            if setpt[2] < ceiling_ned_z:
                setpt[2] = ceiling_ned_z
            yaw_rate = dyaw / LOOKAHEAD_S                    # yaw-rate FF (runaway fix)
            send_target(m, setpt, v, yaw_rate=yaw_rate, use_rate=True)

            w.writerow([k, round(tick_start - t0, 4), *np.round(a_raw, 5),
                        *np.round(setpt, 4), *np.round(v, 4), round(yaw_rate, 4),
                        *np.round(st["pos"], 4), *np.round(st["vel"], 4),
                        *np.round(st["att"], 4), round(inf_ms, 1)])
            f.flush()
            print(f"  t={tick_start-t0:5.1f}s a=({a_raw[0]:+.2f},{a_raw[1]:+.2f},"
                  f"{a_raw[2]:+.2f},{a_raw[3]:+.2f}) inf={inf_ms:.0f}ms "
                  f"pos=({st['pos'][0]:+.2f},{st['pos'][1]:+.2f},{st['pos'][2]:+.2f})",
                  flush=True)

            terr = math.hypot(st["pos"][0] - setpt[0], st["pos"][1] - setpt[1])
            rad = math.hypot(st["pos"][0] - take[0], st["pos"][1] - take[1])
            track_bad = track_bad + 1 if (terr > MAX_TRACK_ERR or rad > MAX_RADIUS) else 0
            if track_bad >= guard_ticks:
                reason = f"guard(track={terr:.1f}m,radius={rad:.1f}m)"
                break
            k += 1
            lag = t0 + (k) * dt - time.monotonic()
            if lag > 0:
                time.sleep(lag)
    f.close()
    hz_act = k / max(time.monotonic() - t0, 1e-6)
    print(f"\n-- inference ended [{reason}] after {k} ticks ({hz_act:.1f} Hz); landing")


if __name__ == "__main__":
    main()
