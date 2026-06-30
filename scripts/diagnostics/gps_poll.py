from pymavlink import mavutil
import time

m = mavutil.mavlink_connection("COM13", baud=57600)
print("connecting...", flush=True)
m.wait_heartbeat(timeout=15)
print("connected. polling GPS for 60s...", flush=True)

t0 = time.time()
while time.time() - t0 < 60:
    msg = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=5)
    if msg:
        print(f"fix={msg.fix_type} sats={msg.satellites_visible}  {'READY' if msg.fix_type >= 3 else ''}", flush=True)
        if msg.fix_type >= 3:
            break
print("done", flush=True)
