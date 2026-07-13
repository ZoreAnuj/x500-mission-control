# TODO — next field test (2026-07-07)

## Confirm the yaw-rate fix on real hardware
The ep276 replay flew the **East-mirror** of the sim because the yaw command ran away
(0.5 s-lookahead absolute-yaw cumulated on the fast real yaw controller — see
`docs/REPLAY_RESULTS.md`). Fix is committed (**yaw-rate feed-forward, default-on**),
**offline-verified but not yet flown.** Tomorrow's flight is the confirmation.

**Expected:** the default (fixed) run now flies **WEST** to match the sim; the
`--yaw-abs` run reproduces the **East** mirror.

- [ ] Preflight: props on, compass OK, GPS 3D lock (≥10 sats, HDOP ≤1.5), battery charged
- [ ] Geofence: `FENCE_ENABLE=1`, `FENCE_RADIUS=15`, `FENCE_ALT_MAX=6`
- [ ] **Clearance ≥6 m to BOTH West and East** — the fixed run goes West, the `--yaw-abs` run goes East
- [ ] Hover baseline: `python field_replay.py --hover 60`  (note real drift)
- [ ] **A — fix (default):** `python field_replay.py --episode 276 --hz 10 --alt 2`  → expect **WEST**
- [ ] **B — old behavior:** `python field_replay.py --episode 276 --hz 10 --alt 2 --yaw-abs`  → expect **EAST mirror**
- [ ] Analyze both: `python analyze_replay.py results/<csv> --dataset <ds> --episode 276 --hz 10`
      → the fix run should show **East end ≈ −4 m** and **divergence-vs-reference drop from ~8 m to <~1 m**
- [ ] ENTER-to-land ready; `python watch_replay.py results/<csv>` preview running
- [ ] Set `MOT_HOVER_LEARN=2` to trim the ~0.3 m altitude overshoot seen on the first flight

Once **A flies West**, the yaw-rate shim is validated → carry it into the Phase-3 policy loop
(the policy emits only Δyaw, so it needs this same fix).

## Policy inference (`field_infer.py`) — READY, gated on the yaw-fix flight above
Desk-verified 2026-07-08 (see `docs/`/commit message):
- offline: ep010 vs dataset actions on training frames — dx corr 0.999 / dz 0.953 / dyaw 0.939, PASS
- SITL dry run (dataset video as camera): 250 ticks @ 10.0 Hz, inference 5.7 ms mean,
  track err 0.07 m, −4 m ceiling clamp held, yaw-rate FF exercised, clean NAV_LAND
- real-ESP32-frame fisheye remap: 100% coverage, geometry sane

## CV baseline flight (`cv_hoop_pass.py`) — desk-reviewed 2026-07-09
Review found + fixed a mission-fatal inverted yaw-servo sign (hoop right -> yawed left);
all 4 servo directions now verified on synthetic frames. Field-only assumptions to check:
- [ ] `--tune` on the real hoop in real light (indoor-photo defaults: H 0-10+168-180, S>=90, V>=25)
- [ ] Hand-wave mirror check (right hand -> image right), else yaw servo inverts
- [ ] `--hoop-dia <m>` = the built hoop's real outer diameter
- [ ] ESP32 auto-exposure sanity outdoors (watch the --tune preview for blowout)
- Wind note: SCAN/CENTER hold zero *velocity*, not position -- drift is bounded only by
  the 12 m radius guard; pick a calm window for the first try
- [ ] Teleop re-test: `x500_teleop.py` (setpoint-nudge rewrite) — confirm no up/down swing, no coast-on-release, yaw taps land where expected
- [ ] Fly: `python cv_hoop_pass.py --connect COM13 --cam-url http://192.168.4.1:81/stream --hoop-dia <m>`

First real inference flight (AFTER A/B passes):
- [ ] ESP32 cam powered, WiFi `Ketu` up, PC joined; `python esp32cam_capture.py` sanity view
- [ ] Place drone ~5 m from hoop, FACING it (sim yawed to hoop; real has no hoop NED)
- [ ] `python field_infer.py --connect COM13 --cam-url http://192.168.4.1:81/stream --hz 10 --alt 1.4 --max-secs 30`
- [ ] GO gate → ENTER=land anytime; watch for the climb-and-scan then approach
- SITL gotcha: run SITL clients INSIDE WSL — a Windows client through the localhost relay
  makes SITL exit code 1 on connect (serial0 disconnect is fatal to it)
