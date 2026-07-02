#!/usr/bin/env python3
"""X500 first flight: GUIDED -> arm -> takeoff -> hover -> land (ArduPilot/pymavlink).
Props must be ON. Close Mission Planner first.

Robust GPS path: refuses to arm until the *quality* of the position estimate is good
(sats + HDOP + EKF variances), takes off to a safe altitude that clears ground effect,
and waits to actually REACH altitude before holding — instead of a fixed sub-ramp timer.
This targets the two real-world failures we hit: drift on GPS, and toppling flat on the
ground during takeoff (dynamic rollover from correcting a bad position estimate while a
landing leg is still touching down).

Usage:
  python x500_first_flight.py           # normal — aborts unless preflight gates pass
  python x500_first_flight.py --force   # bypass the preflight gates (deliberate bench test)
  python x500_first_flight.py --nogps   # disable GPS in firmware, reboot, fly on baro+IMU
                                        # drone will drift laterally — keep clear of walls
"""
import math
import sys
import threading
import time
from pymavlink import mavutil

_ports      = [a for a in sys.argv[1:] if not a.startswith("--")]
PORT        = _ports[0] if _ports else "/dev/ttyUSB0"   # or COM13 on Windows
BAUD        = 57600
TARGET_ALT  = 2.0   # metres AGL — clears ground effect (~1 rotor diameter) so lateral
                    # corrections happen in the air, not while a leg is on the ground
HOVER_SECS  = 5     # seconds to hold at altitude (GUIDED station-keeps; position latched)

# Preflight quality gates (all must pass before arming, unless --force)
MIN_SATS    = 10    # satellites
MAX_HDOP    = 1.5   # horizontal dilution of precision
MAX_EKF_VAR = 0.5   # ceiling for EKF pos_horiz / velocity / compass variance
MIN_VOLTS   = 13.0  # refuse to "fly" on USB power (~0 V) or a flat pack
LEVEL_DEG   = 3.0   # max |roll|,|pitch| at arm

# Drift guard: if the vehicle wanders past a limit, land now.
MAX_DRIFT_M       = 1.5   # hover: horizontal distance from the hover start point (m)
MAX_CLIMB_DRIFT_M = 2.0   # climb: a bit more lenient — the climb is a dynamic phase
MAX_DRIFT_SPD     = 0.8   # horizontal ground speed limit (m/s), both phases

NOGPS       = "--nogps" in sys.argv
FORCE       = "--force" in sys.argv

# Manual abort: pressing ENTER in the flight terminal sets this -> immediate LAND.
STOP = threading.Event()

def stop_watcher():
    """Block on stdin; the first line (ENTER / any text) requests a STOP -> LAND."""
    try:
        for _ in sys.stdin:
            STOP.set()
            break
    except Exception:
        pass

# ── helpers ──────────────────────────────────────────────────────────────────

def connect(timeout=30):
    print(f"connecting {PORT}@{BAUD}...", flush=True)
    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    hb = m.wait_heartbeat(timeout=timeout)
    if hb is None:
        print("NO HEARTBEAT", flush=True); sys.exit(1)
    print(f"-- connected  sys={m.target_system}  fw=ArduPilot", flush=True)
    return m

def horiz_dist(lat1, lon1, lat2, lon2):
    """Metres between two lat/lon (equirectangular approx — fine at these scales)."""
    R = 6371000.0
    x = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
    y = math.radians(lat2 - lat1)
    return R * math.hypot(x, y)

def set_param(m, name, value, ptype=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    m.mav.param_set_send(m.target_system, m.target_component,
                         name.encode(), float(value), ptype)
    time.sleep(0.3)

def set_mode(m, mode_name):
    mode_id = m.mode_mapping()[mode_name]
    m.mav.set_mode_send(m.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    for _ in range(20):
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and mavutil.mode_string_v10(hb) == mode_name:
            return True
    return False

def wait_alt(m, target, margin=0.05, watch_secs=15):
    """Watch the real climb, guarding lateral drift. Returns (alt, drifted):
      alt      = altitude reached (or peak if the window ended)
      drifted  = True if it wandered past MAX_CLIMB_DRIFT_M / MAX_DRIFT_SPD -> caller lands."""
    print(f"climbing toward {target}m (drift guard {MAX_CLIMB_DRIFT_M}m / {MAX_DRIFT_SPD}m/s)...",
          flush=True)
    t0 = time.time()
    peak = 0.0
    ref = None          # liftoff lat/lon
    drift_hits = 0
    while time.time() - t0 < watch_secs:
        if STOP.is_set():
            print("\n>>> STOP requested — LANDING NOW", flush=True)
            return peak, True
        beat(m)   # keep GCS failsafe satisfied while climbing
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if msg is None:
            continue
        rel = msg.relative_alt / 1000.0
        peak = max(peak, rel)
        lat, lon = msg.lat / 1e7, msg.lon / 1e7
        spd = math.hypot(msg.vx, msg.vy) / 100.0
        if ref is None:
            ref = (lat, lon)
        dist = horiz_dist(ref[0], ref[1], lat, lon)
        print(f"  alt={rel:+.2f}m (peak {peak:.2f})  drift={dist:.2f}m spd={spd:.2f}m/s   ",
              end="\r", flush=True)
        if dist > MAX_CLIMB_DRIFT_M or spd > MAX_DRIFT_SPD:
            drift_hits += 1
            if drift_hits >= 2:
                print(f"\n>>> DRIFT {dist:.2f}m / {spd:.2f}m/s during climb — LANDING NOW",
                      flush=True)
                return peak, True
        else:
            drift_hits = 0
        if rel >= target - margin:
            print(f"\n-- reached {rel:.2f}m", flush=True)
            return rel, False
    print(f"\n-- climb window ended; peak altitude {peak:.2f}m "
          f"({'LIFTED' if peak > 0.15 else 'NO LIFTOFF'})", flush=True)
    return peak, False

def disarm(m):
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, 0, 0, 0, 0, 0, 0)

def land_and_disarm(m, touchdown_alt=0.25, disarm_timeout=25):
    """LAND, watch the descent, then wait for ArduPilot's own auto-disarm.
    Force-disarming mid-descent could drop the vehicle, so we only fall back to an
    explicit disarm if auto-disarm hasn't happened well after touchdown."""
    print("landing...", flush=True)
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
        0, 0, 0, 0, 0, 0, 0)
    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=8)
    print(f"LAND ack={ack.result if ack else 'no ACK'}", flush=True)

    print("descending; waiting for touchdown + auto-disarm...", flush=True)
    t0 = time.time()
    touched = False
    while time.time() - t0 < disarm_timeout:
        beat(m)   # keep sending until disarmed, so a slow descent isn't cut by FS_GCS
        msg = m.recv_match(type=["GLOBAL_POSITION_INT", "HEARTBEAT", "STATUSTEXT"],
                           blocking=True, timeout=1)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "GLOBAL_POSITION_INT":
            rel = msg.relative_alt / 1000.0
            touched = touched or rel < touchdown_alt
            print(f"  alt={rel:+.2f}m   ", end="\r", flush=True)
        elif t == "HEARTBEAT":
            if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("\n-- auto-disarmed.", flush=True)
                return
        elif t == "STATUSTEXT":
            print(f"\n  FC: {msg.text.strip()}", flush=True)
    # fallback: touched down (or timed out) but still armed → disarm explicitly
    print("\n-- still armed after landing window; sending disarm", flush=True)
    disarm(m)
    print("-- disarmed.", flush=True)

# ── --nogps setup: disable GPS in firmware and reboot ───────────────────────

def save_params(m):
    """Explicitly flush params to EEPROM before reboot."""
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 0,
        1, 0, 0, 0, 0, 0, 0)
    time.sleep(1.5)

def setup_nogps(m):
    print("-- --nogps: GPS1_TYPE=0 + ARMING_CHECK=0 + BRD_SAFETY_DEFLT=0, rebooting...", flush=True)
    set_param(m, "GPS1_TYPE",        0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    set_param(m, "ARMING_CHECK",     0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    set_param(m, "BRD_SAFETY_DEFLT",0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    set_param(m, "FS_EKF_ACTION",    0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    save_params(m)
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
        1, 0, 0, 0, 0, 0, 0)
    print("-- rebooting... waiting 20s", flush=True)
    m.close()
    time.sleep(20)
    m2 = connect(timeout=30)
    print("-- reconnected after reboot", flush=True)
    return m2

def restore_gps(m):
    print("restoring GPS1_TYPE=1 + ARMING_CHECK=1 + BRD_SAFETY_DEFLT=0, rebooting...", flush=True)
    set_param(m, "GPS1_TYPE",        1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    set_param(m, "ARMING_CHECK",     1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    set_param(m, "FS_EKF_ACTION",    1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    # ponytail: keep BRD_SAFETY_DEFLT=0 — Holybro ships it this way, we want it off permanently
    save_params(m)
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
        1, 0, 0, 0, 0, 0, 0)
    print("-- GPS restored. Drone rebooting (normal ops again).", flush=True)

# ── no-RC failsafe + GCS heartbeat ───────────────────────────────────────────

def beat(m):
    """Send one GCS heartbeat. With no RC transmitter, the GCS-heartbeat failsafe
    (FS_GCS) is our safety net — the FC LANDs if these stop. We must send them
    continuously while armed, so call this inside every in-flight loop."""
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

def set_failsafe_norc(m):
    """Configure the FC for transmitter-less operation:
      FS_THR_ENABLE=0  -> disable the radio failsafe (else it disarms ~1s after arm)
      FS_GCS_ENABLE=5  -> GCS-heartbeat failsafe action = LAND (replaces RC failsafe)
    ArduPilot persists PARAM_SET immediately, so these survive across runs."""
    print("-- no-RC setup: FS_THR_ENABLE=0 (radio FS off), FS_GCS_ENABLE=5 (link-loss=LAND)",
          flush=True)
    set_param(m, "FS_THR_ENABLE", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    set_param(m, "FS_GCS_ENABLE", 5, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    beat(m)   # register a GCS with the FC so the failsafe becomes active

# ── preflight quality gates ──────────────────────────────────────────────────

def gate_failures(best, st):
    """Return a list of human-readable reasons the vehicle is NOT ready to arm.
    Empty list == every gate passes. `best` = best GPS seen, `st` = latest others."""
    f = []
    if best["fix"] < 3:
        f.append(f"GPS fix {best['fix']} < 3D")
    if best["sats"] < MIN_SATS:
        f.append(f"sats {best['sats']} < {MIN_SATS}")
    if best["hdop"] > MAX_HDOP:
        f.append(f"HDOP {best['hdop']:.1f} > {MAX_HDOP}")
    e = st["ekf"]
    if e is None:
        f.append("no EKF_STATUS_REPORT received yet")
    else:
        ph, ve, co = e
        if ph > MAX_EKF_VAR:
            f.append(f"EKF pos_horiz_variance {ph:.2f} > {MAX_EKF_VAR}")
        if ve > MAX_EKF_VAR:
            f.append(f"EKF velocity_variance {ve:.2f} > {MAX_EKF_VAR}")
        if co > MAX_EKF_VAR:
            f.append(f"EKF compass_variance {co:.2f} > {MAX_EKF_VAR}  "
                     f"-> run x500_compass_cal.py, then retry")
    if st["volt"] is None or st["volt"] <= 0.5:
        pass   # battery monitor disabled (BATT_MONITOR=0) -> can't gate on voltage.
               # Battery testing was intentionally skipped; watch the pack / time-limit the hover.
    elif st["volt"] <= MIN_VOLTS:
        f.append(f"battery {st['volt']:.1f}V <= {MIN_VOLTS}V (flat pack?)")
    if st["roll"] is None:
        f.append("no ATTITUDE received yet")
    elif abs(st["roll"]) > LEVEL_DEG or abs(st["pitch"]) > LEVEL_DEG:
        f.append(f"not level (roll {st['roll']:.0f} pitch {st['pitch']:.0f}, "
                 f"limit {LEVEL_DEG})")
    # NOTE: no RC gate — this airframe has no transmitter. The radio failsafe is
    # disabled and the GCS-heartbeat failsafe (FS_GCS) replaces it; the kill switch
    # lives in the web UI. See set_failsafe_norc().
    return f

def preflight_gates(m, timeout=60):
    """Poll telemetry until every safety gate passes (return True), or timeout (False).
    Prints exactly which gates still fail. Requires the streams to be requested first."""
    print(f"-- preflight gates: fix>=3, sats>={MIN_SATS}, HDOP<={MAX_HDOP}, "
          f"EKF var<={MAX_EKF_VAR}, level<{LEVEL_DEG}deg "
          f"(battery only if monitored; no RC gate) (up to {timeout}s)...", flush=True)
    # make sure EKF variances actually stream over the radio
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, int(1e6 / 2), 0, 0, 0, 0, 0)

    best = {"fix": 0, "sats": 0, "hdop": 99.9}
    st = {"ekf": None, "volt": None, "roll": None, "pitch": None}
    t0 = time.time()
    last_print = 0.0
    good_streak = 0
    while time.time() - t0 < timeout:
        beat(m)   # keep the GCS-heartbeat failsafe registered/satisfied
        msg = m.recv_match(type=["GPS_RAW_INT", "EKF_STATUS_REPORT", "SYS_STATUS",
                                 "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=2)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "GPS_RAW_INT":
            best["fix"] = max(best["fix"], msg.fix_type)
            best["sats"] = max(best["sats"], msg.satellites_visible)
            hd = msg.eph / 100.0 if msg.eph != 65535 else 99.9
            best["hdop"] = min(best["hdop"], hd)
        elif t == "EKF_STATUS_REPORT":
            st["ekf"] = (msg.pos_horiz_variance, msg.velocity_variance, msg.compass_variance)
        elif t == "SYS_STATUS":
            st["volt"] = msg.voltage_battery / 1000.0
        elif t == "ATTITUDE":
            st["roll"] = math.degrees(msg.roll)
            st["pitch"] = math.degrees(msg.pitch)
        elif t == "STATUSTEXT":
            if "prearm" in msg.text.lower():
                print(f"\n  FC: {msg.text.strip()}", flush=True)

        fails = gate_failures(best, st)
        if not fails:
            good_streak += 1
            if good_streak >= 2:   # require it to be stable, not a single lucky sample
                print("\n-- all preflight gates PASSED", flush=True)
                return True
        else:
            good_streak = 0
            if time.time() - last_print > 3:
                last_print = time.time()
                print(f"  waiting: {'; '.join(fails)}      ", end="\r", flush=True)

    print("\n-- preflight gates FAILED:", flush=True)
    for reason in gate_failures(best, st):
        print(f"     - {reason}", flush=True)
    return False

# ── main ─────────────────────────────────────────────────────────────────────

def main():
    m = connect()
    # Ask for VFR_HUD / GPS / position streams over the radio (for throttle capture).
    m.mav.request_data_stream_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

    if NOGPS:
        m = setup_nogps(m)
        set_failsafe_norc(m)   # no transmitter here either
    else:
        # No transmitter: disable the radio failsafe and switch on the GCS-heartbeat
        # failsafe (LAND) as its replacement, before we gate/arm.
        set_failsafe_norc(m)
        # Quality gate: refuse to arm until the position estimate is trustworthy.
        # ARMING_CHECK stays ENABLED — we no longer blanket-disable the checks that
        # would catch exactly the conditions that made it drift/topple.
        if not preflight_gates(m):
            if FORCE:
                print("!! --force: continuing despite failed preflight gates", flush=True)
            else:
                print("ABORT: preflight gates not satisfied (use --force to override)", flush=True)
                sys.exit(1)

    # Re-apply ARMING_CHECK=0 after reboot (belt-and-suspenders)
    if NOGPS:
        set_param(m, "ARMING_CHECK", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(m, "FS_EKF_ACTION", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        time.sleep(1)

    # Manual abort: press ENTER in this terminal at any time -> immediate LAND.
    threading.Thread(target=stop_watcher, daemon=True).start()
    print("\n>>>>>>  PRESS ENTER AT ANY TIME TO STOP -> LAND  <<<<<<\n", flush=True)

    # Arm directly in GUIDED (real GPS gives position — this is what the flight
    # that actually lifted did; arming in STABILIZE first was a no-GPS leftover).
    print("setting GUIDED for arm...", flush=True)
    if not set_mode(m, "GUIDED"):
        print("GUIDED mode failed", flush=True); sys.exit(1)

    # Arm — also drain STATUSTEXT messages to show why it fails
    print("arming...", flush=True)
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    ack = None
    t0 = time.time()
    while time.time() - t0 < 10:
        beat(m)
        msg = m.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=2)
        if msg is None: continue
        if msg.get_type() == "STATUSTEXT":
            print(f"  FC: {msg.text.strip()}", flush=True)
        elif msg.get_type() == "COMMAND_ACK":
            ack = msg; break
    if not ack or ack.result != 0:
        print(f"ARM failed result={ack.result if ack else 'no ACK'}", flush=True)
        if NOGPS: restore_gps(m)
        sys.exit(1)
    print("-- ARMED (GUIDED)", flush=True)
    beat(m)
    time.sleep(0.5)

    # Takeoff
    print(f"takeoff -> {TARGET_ALT}m...", flush=True)
    beat(m)
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, TARGET_ALT)
    # beat while waiting for the ACK — a 10s blocking wait would exceed FS_GCS (5s)
    # and trigger a spurious LAND right after arming.
    ack = None
    t0 = time.time()
    while time.time() - t0 < 10:
        beat(m)
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
        if ack is not None:
            break
    if not ack or ack.result != 0:
        print(f"TAKEOFF failed result={ack.result if ack else 'no ACK'}", flush=True)
        land_and_disarm(m)
        if NOGPS: restore_gps(m)
        sys.exit(1)

    # Climb-watch: wait until we actually REACH altitude (up to 15s) instead of a
    # fixed sub-ramp timer. Letting the 3-4s GUIDED throttle ramp complete means the
    # vehicle is cleanly airborne and clear of ground effect before it ever applies a
    # lateral position correction — which is what prevents the on-ground rollover.
    peak, drifted = wait_alt(m, TARGET_ALT, margin=0.15, watch_secs=15)
    if drifted:
        land_and_disarm(m)
        if NOGPS: restore_gps(m)
        sys.exit(1)
    if peak < 0.15:
        print("NO LIFTOFF — landing & disarming", flush=True)
        land_and_disarm(m)
        if NOGPS: restore_gps(m)
        sys.exit(1)

    # Hold at altitude. GUIDED station-keeps automatically (position target is latched),
    # so no streaming is needed — just monitor and wait.
    print(f"-- holding up to {HOVER_SECS}s at {peak:.2f}m "
          f"(GUIDED station-keep; drift guard {MAX_DRIFT_M}m / {MAX_DRIFT_SPD}m/s -> LAND)...",
          flush=True)
    ref = None          # hover-start lat/lon
    drift_hits = 0      # consecutive over-limit samples (debounce GPS noise)
    t0 = time.time()
    while time.time() - t0 < HOVER_SECS:
        if STOP.is_set():
            print("\n>>> STOP requested — LANDING NOW", flush=True)
            break
        beat(m)   # critical: stop these and the FC will LAND (FS_GCS)
        msg = m.recv_match(type=["GLOBAL_POSITION_INT", "STATUSTEXT"],
                           blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            print(f"\n  FC: {msg.text.strip()}", flush=True)
            continue
        lat, lon = msg.lat / 1e7, msg.lon / 1e7
        alt = msg.relative_alt / 1000.0
        spd = math.hypot(msg.vx, msg.vy) / 100.0     # horizontal ground speed (m/s)
        if ref is None:
            ref = (lat, lon)
        dist = horiz_dist(ref[0], ref[1], lat, lon)
        print(f"  hold alt={alt:+.2f}m  drift={dist:.2f}m  spd={spd:.2f}m/s   ",
              end="\r", flush=True)
        if dist > MAX_DRIFT_M or spd > MAX_DRIFT_SPD:
            drift_hits += 1
            if drift_hits >= 2:                       # need 2 in a row, not one GPS blip
                print(f"\n>>> DRIFT {dist:.2f}m / {spd:.2f}m/s exceeds limit "
                      f"({MAX_DRIFT_M}m / {MAX_DRIFT_SPD}m/s) — LANDING NOW", flush=True)
                break
        else:
            drift_hits = 0
    print()

    # Land, then wait for ArduPilot's own auto-disarm (don't force-disarm mid-descent,
    # which could drop the vehicle).
    land_and_disarm(m)

    if NOGPS:
        restore_gps(m)


if __name__ == "__main__":
    main()
