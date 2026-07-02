# X500 Bring-up / Test & Calibration Checklist

Ordered checklist for validating the Holybro X500 (Pixhawk 6C, ArduCopter, **no RC transmitter**)
after a **propeller change**. Do these **one at a time, in order**. Compiled from official
ArduPilot docs (see `docs/MAVLINK_REFERENCE.md` and the sources cited per item).

## Legend — what each item needs
- 🔌 **USB** ok / 🔋 **flight battery required** (ESCs and the power monitor need pack power — USB is not enough)
- 🛰️ needs **3D GPS lock** (do it outdoors)
- 🪶 **props OFF** / 🌀 **props ON** (≤10% throttle, frame restrained)

> **Current state:** vehicle on USB (battery 0 V), GPS was reaching fix/HDOP≤1.5 outdoors, no RC.
> No-RC failsafe already applied: `FS_THR_ENABLE=0`, `FS_GCS_ENABLE=5`, GCS heartbeats in script+server.

---

## Group A — Config sanity  🔌 ✅ DONE 2026-07-01
- **A1. Frame** ✅ `FRAME_CLASS=1` (Quad), `FRAME_TYPE=1` (X).
- **A2. No-RC arming check** — user chose to **skip battery + RC arming checks**. Applied
  `ARMING_CHECK=15550` (Baro+Compass+GPS+INS+Params+BoardV+Logging+Safety+GPScfg+System;
  **RC(64) and Battery(256) excluded**). Verified by read-back.
- **A3. Failsafe/safety** ✅ `FS_THR_ENABLE=0`, `FS_GCS_ENABLE=5`, `BRD_SAFETY_DEFLT=0`. (`ARMING_RUDDER=2`, harmless — no yaw stick.)
- **A4. ESC protocol** ✅ `MOT_PWM_TYPE=0` = **Normal PWM (not DShot)** → **ESC calibration D1 IS required**, and wrong motor direction is fixed by **swapping two motor wires** (not `SERVO_BLH_RVMASK`).

## Group B — Sensor calibration  🪶 (props OFF)  ✅ DONE 2026-07-01
- **B1. Accelerometer 6-point** ✅ DONE 2026-07-01 via `scripts/x500_accel_cal.py`. Verified: `INS_ACCSCAL`≈(1.0003,0.9990,1.0002), offsets set, no PreArm accel complaint. (Note: SiK drops the FC's position prompts, so the script drives the 6 positions in fixed order, paced by ENTER, and spams each `ACCELCAL_VEHICLE_POS`; needs a clean/rebooted state to start.)
- **B2. Level trim** ✅ DONE — `AHRS_TRIM`≈(-0.03°, -0.83°).
- **B3. Compass onboard cal** ✅ DONE 2026-07-01 via `x500_compass_cal.py` (patched: `/dev/ttyUSB0` port + GCS heartbeats). Fitness: Mag1 (external)=3.7, Mag0 (internal)=19.7. After reboot: Compass1 |ofs|=128, Compass2 |ofs|=56 (both in use, < 600), no PreArm compass complaints.
- **B4. Gyro/baro/health** ✅ checked — no accel/altitude pre-arm complaints.

## Group C — Power / battery  🔋 — ⏭️ SKIPPED (user)
Battery monitor calibration and battery failsafe deferred by user request. Note: the battery
arming check is also disabled (A2), so the FC won't block arming on battery state.

## Group D — Motors & ESCs  🔋 (prop-change core — do carefully)
- **D1. ESC calibration** ✅ DONE 2026-07-01 via `scripts/x500_esc_cal.py` (`ESC_CALIBRATION=3` → battery power-cycle → auto-reset to 0 = completed). Motivated by CCW motors spinning faster than CW at equal throttle (mismatched endpoints).
- **D2. Motor order + direction** ✅ DONE via `scripts/x500_motor_direction.py` (props-on, 8%). All four correct: M1 Front-Right CCW, M2 Rear-Left CCW, M3 Front-Left CW, M4 Rear-Right CW.
- **D3. Prop orientation** — implicitly OK (correct spin directions confirmed with props mounted); re-confirm leading edge scoops air **down** on each.
- **D4. Props-on spin test** 🌀 — re-run `x500_motor_direction.py` after ESC cal to confirm **even RPM** across all four (validates D1). Smooth spin-up, no wobble.
- **D5. Spin thresholds** 🌀 — PENDING. Find min % where all four reliably start; `MOT_SPIN_ARM = min%+2%`, `MOT_SPIN_MIN = MOT_SPIN_ARM + 0.03`.
- **D6. Thrust curve** — PENDING. Set `MOT_THST_EXPO` by prop diameter (10"→0.65 default · 5"→0.55 · ≥20"→0.75); leave `MOT_SPIN_MAX=0.95`.

## Group E — Pre-hover tuning presets  🔌 (set before first hover)
- **E1.** ⚠️ `MOT_THST_HOVER` — **set it near the vehicle's REAL hover point, not blindly low.** We first set 0.25 (per the generic "prevent takeoff jump" advice) but this airframe's real hover is high (pinned at the 0.6875 max = marginal thrust-to-weight). 0.25 caused a **weak, tippy liftoff → topple**. Raising to **0.5 → 0.6** gave a firm, clean liftoff. Keep `MOT_HOVER_LEARN=2` so it refines in the air. Lesson: only pre-set *low* if the real hover is genuinely ~0.4–0.5.
- **E2.** Confirm initial-tune filters are sane (X500 defaults are normally fine): `INS_ACCEL_FILTER=10`, D-term filters ≈ `INS_GYRO_FILTER/2`.

## Group F — First hover & verification  🔋🛰️🌀  ✅ liftoff successful 2026-07-01
- **F1.** ✅ Gentle 2 m hover via `x500_first_flight.py` succeeded once `MOT_THST_HOVER` was raised to ~0.6. Earlier topples were the weak-liftoff (E1) issue; motors verified even, props verified (direction + airflow). Watch for oscillation — land + halve `ATC_RAT_RLL/PIT_*` if it appears.
- **F2. Vibration** — from the log: `VIBE` < 30 m/s/s, clipping ≈ 0. New props can be unbalanced.
- **F3. Hover throttle** — should settle ~40–60% (`CTUN.ThO`); let `MOT_THST_HOVER` converge (hover ≥30 s in AltHold).
- **F4. AutoTune** — later, only after a clean low-vibration hover (`AUTOTUNE_AXES=15`, `AUTOTUNE_AGGR=0.1`). Note: AutoTune's test-toggle normally wants an RC aux switch — GCS-only AutoTune is awkward; treat as advanced.

---

### Two hard safety rules for the prop-change work
1. **Verify motor direction & prop orientation with props OFF (D2–D3) *before* any props-on spin (D4) or takeoff.** A single wrong prop/direction flips the aircraft instantly on takeoff.
2. **Motor tests need the flight battery, not USB** — the ESCs are powered from the pack. On USB the motors won't spin.
