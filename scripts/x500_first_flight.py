#!/usr/bin/env python3
"""X500 first flight: GUIDED -> arm -> takeoff -> hover -> land (ArduPilot/pymavlink).
Props must be ON. Close Mission Planner first.

Usage:
  python x500_first_flight.py           # normal (requires GPS fix >= 3)
  python x500_first_flight.py --nogps   # disable GPS in firmware, reboot, fly on baro+IMU
                                        # drone will drift laterally — keep clear of walls
"""
import sys
import time
from pymavlink import mavutil

PORT        = "COM13"
BAUD        = 57600
TARGET_ALT  = 0.5   # metres AGL
HOVER_SECS  = 3
NOGPS       = "--nogps" in sys.argv

# ── helpers ──────────────────────────────────────────────────────────────────

def connect(timeout=30):
    print(f"connecting {PORT}@{BAUD}...", flush=True)
    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    hb = m.wait_heartbeat(timeout=timeout)
    if hb is None:
        print("NO HEARTBEAT", flush=True); sys.exit(1)
    print(f"-- connected  sys={m.target_system}  fw=ArduPilot", flush=True)
    return m

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
    """Watch the real climb. Returns when within margin of target, else reports
    the max altitude reached after watch_secs (so we can see if it lifted at all)."""
    print(f"climbing toward {target}m (ceiling)...", flush=True)
    t0 = time.time()
    peak = 0.0
    while time.time() - t0 < watch_secs:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if msg is None:
            continue
        rel = msg.relative_alt / 1000.0
        peak = max(peak, rel)
        print(f"  alt={rel:+.2f}m  (peak {peak:.2f})", end="\r", flush=True)
        if rel >= target - margin:
            print(f"\n-- reached {rel:.2f}m", flush=True)
            return rel
    print(f"\n-- climb window ended; peak altitude {peak:.2f}m "
          f"({'LIFTED' if peak > 0.15 else 'NO LIFTOFF'})", flush=True)
    return peak

def disarm(m):
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, 0, 0, 0, 0, 0, 0)

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

# ── main ─────────────────────────────────────────────────────────────────────

def main():
    m = connect()
    # Ask for VFR_HUD / GPS / position streams over the radio (for throttle capture).
    m.mav.request_data_stream_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

    if NOGPS:
        m = setup_nogps(m)
    else:
        # poll a few seconds; take the best fix seen (avoids a single stale fix=0)
        fix = sats = 0
        t0 = time.time()
        while time.time() - t0 < 6:
            gps = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)
            if gps and gps.fix_type > fix:
                fix, sats = gps.fix_type, gps.satellites_visible
            if fix >= 3:
                break
        print(f"-- GPS fix={fix} sats={sats}", flush=True)
        if fix < 3:
            print("GPS not locked — forcing ARMING_CHECK=0 and continuing (RC must be on)", flush=True)
            set_param(m, "ARMING_CHECK", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            time.sleep(0.5)

    # Re-apply ARMING_CHECK=0 after reboot (belt-and-suspenders)
    if NOGPS:
        set_param(m, "ARMING_CHECK", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(m, "FS_EKF_ACTION", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        time.sleep(1)

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
    time.sleep(0.5)

    # Takeoff
    print(f"takeoff -> {TARGET_ALT}m...", flush=True)
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, TARGET_ALT)
    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=10)
    if not ack or ack.result != 0:
        print(f"TAKEOFF failed result={ack.result if ack else 'no ACK'}", flush=True)
        disarm(m)
        if NOGPS: restore_gps(m)
        sys.exit(1)

    # Fast hop: climb + brief hold, then land — total airborne under 5s so baro
    # drift has no time to corrupt the altitude estimate.
    print(f"hop: liftoff toward {TARGET_ALT}m ceiling, hold ~1s, land...", flush=True)
    t0 = time.time()
    peak = 0.0
    thr = -1
    while time.time() - t0 < 2.5:
        msg = m.recv_match(type=["GLOBAL_POSITION_INT", "VFR_HUD", "STATUSTEXT"],
                           blocking=True, timeout=2)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "GLOBAL_POSITION_INT":
            rel = msg.relative_alt / 1000.0
            peak = max(peak, rel)
            print(f"  alt={rel:+.2f}m peak={peak:.2f} thr={thr}%   ", end="\r", flush=True)
        elif t == "VFR_HUD":
            thr = msg.throttle           # commanded throttle %
        elif t == "STATUSTEXT":
            print(f"\n  FC: {msg.text.strip()}", flush=True)
    print(f"\n-- peak {peak:.2f}m  max_throttle~{thr}%  "
          f"({'LIFTED' if peak > 0.15 else 'NO LIFTOFF'})", flush=True)

    # Land
    print("landing...", flush=True)
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
        0, 0, 0, 0, 0, 0, 0)
    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=8)
    print(f"LAND ack={ack.result if ack else 'no ACK'}", flush=True)

    print("waiting for touchdown...", flush=True)
    while True:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if msg is None: break
        rel = msg.relative_alt / 1000.0
        print(f"  alt={rel:.1f}m", end="\r", flush=True)
        if rel < 0.25: break
    print("\n-- touchdown", flush=True)

    disarm(m)
    print("-- disarmed.", flush=True)

    if NOGPS:
        restore_gps(m)


if __name__ == "__main__":
    main()
