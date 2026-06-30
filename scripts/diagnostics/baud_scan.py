"""Find the SiK radio's serial baud on COM13 by scanning for a MAVLink heartbeat."""
import sys
from pymavlink import mavutil

PORT = "COM13"
for baud in (57600, 115200, 38400, 230400, 19200, 9600):
    print(f"trying {PORT}@{baud} ...", end=" ", flush=True)
    try:
        m = mavutil.mavlink_connection(PORT, baud=baud)
    except Exception as e:
        print(f"open failed: {e}", flush=True)
        continue
    hb = m.wait_heartbeat(timeout=5)
    if hb is not None:
        ap = {3: "ArduPilot", 12: "PX4"}.get(hb.autopilot, f"ap={hb.autopilot}")
        print(f"HEARTBEAT  sys={m.target_system} comp={m.target_component} fw={ap}  <== BAUD = {baud}", flush=True)
        m.close()
        sys.exit(0)
    print("no heartbeat", flush=True)
    m.close()

print("No heartbeat at any baud — radio link down, drone powered off, or wrong port.", flush=True)
sys.exit(1)
