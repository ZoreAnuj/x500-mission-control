#!/usr/bin/env python3
"""Offline policy sanity check: feed the EXACT frames + states the policy was trained on
(drone_hoop_30hz_v5 frame_cache_160x120 + parquet states) and compare its output actions
to the recorded dataset actions. IMLE is stochastic, so expect strong correlation and
sign agreement, not equality.

  python check_policy_offline.py --episodes 0 1 50
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import torch
import torchvision.transforms.functional as TF

for _p in ("D:/lucky_drone_il/scripts", "/mnt/d/lucky_drone_il/scripts"):
    if Path(_p).exists() and _p not in sys.path:
        sys.path.insert(0, _p)
from field_infer import load_policy, POLICY_HW, _MEAN, _STD  # reuse the flight loader

DS = Path("D:/lucky_drone_il/data/drone_hoop_30hz_v5")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, nargs="+", default=[0, 1, 50])
    ap.add_argument("--checkpoint",
                    default="D:/lucky_drone_il/runs/cmenc_imle_v1_128d_checkpoints/cmenc_imle_v1_ep010.pt")
    ap.add_argument("--encoder",
                    default="D:/lucky_drone_il/runs/cmvae_v2_checkpoints_128d/cmvae_v1_best.pt")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--batch", type=int, default=64)
    a = ap.parse_args()

    model, dev = load_policy(a.checkpoint, a.encoder, a.device)
    # raw headerless memmap + meta.json, exactly as the training dataloader reads it
    import json
    cdir = DS / "frame_cache_160x120"
    cmeta = json.loads((cdir / "meta.json").read_text())
    ep_starts = np.array(cmeta["ep_starts"], dtype=np.int64)
    ep_lengths = np.array(cmeta["ep_lengths"], dtype=np.int64)
    cache = np.memmap(cdir / "observation.images.CameraFront.npy", dtype=np.uint8,
                      mode="r", shape=(int(cmeta["total_frames"]), 120, 160, 3))
    data = pd.read_parquet(DS / "data" / "chunk-000" / "file-000.parquet",
                           columns=["episode_index", "observation.state", "action"])
    print(f"cache {cache.shape} {cache.dtype}")

    all_pred, all_rec = [], []
    for ep in a.episodes:
        i0 = int(ep_starts[ep]); i1 = i0 + int(ep_lengths[ep])
        frames = cache[i0:i1]                                     # (N,120,160,3) uint8
        epd = data[data["episode_index"] == ep]
        states = np.stack(epd["observation.state"].values).astype(np.float32)
        actions = np.stack(epd["action"].values).astype(np.float32)
        assert len(frames) == len(states) == len(actions), (len(frames), len(states), len(actions))

        preds = []
        with torch.no_grad():
            for b0 in range(0, len(frames), a.batch):
                fb = np.ascontiguousarray(frames[b0:b0 + a.batch])
                t = torch.from_numpy(fb).permute(0, 3, 1, 2).float().to(dev) / 255.0
                t = TF.resize(t, [POLICY_HW, POLICY_HW], antialias=True)
                t = (t - _MEAN.to(dev)) / _STD.to(dev)
                s = torch.from_numpy(states[b0:b0 + a.batch]).to(dev)
                preds.append(model(t, s)[:, 0].float().cpu().numpy())
        pred = np.concatenate(preds)
        all_pred.append(pred)
        all_rec.append(actions)

        names = ["dx", "dy", "dz", "dyaw"]
        print(f"\nepisode {ep}  ({len(pred)} frames)")
        for i, nm in enumerate(names):
            r = np.corrcoef(pred[:, i], actions[:, i])[0, 1]
            mae = np.abs(pred[:, i] - actions[:, i]).mean()
            sign = (np.sign(pred[:, i]) == np.sign(actions[:, i]))[np.abs(actions[:, i]) > 0.02].mean()
            print(f"  {nm:5s} corr={r:+.3f}  MAE={mae:.4f}  sign-agree={sign:.2f}  "
                  f"pred[{pred[:,i].min():+.3f},{pred[:,i].max():+.3f}] "
                  f"rec[{actions[:,i].min():+.3f},{actions[:,i].max():+.3f}]")

    P, R = np.concatenate(all_pred), np.concatenate(all_rec)
    print("\n=== OVERALL ===")
    ok = True
    for i, nm in enumerate(["dx", "dy", "dz", "dyaw"]):
        r = np.corrcoef(P[:, i], R[:, i])[0, 1]
        print(f"  {nm:5s} corr={r:+.3f}")
        if i in (0, 2) and r < 0.5:          # dx/dz drive the task
            ok = False
    print("finite:", np.isfinite(P).all(), " |pred|max:", np.abs(P).max(0).round(3))
    print("VERDICT:", "PASS - policy reproduces dataset actions" if ok and np.isfinite(P).all()
          else "FAIL - investigate before flying")


if __name__ == "__main__":
    main()
