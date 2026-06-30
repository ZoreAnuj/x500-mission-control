from pymavlink import mavutil
import time
m = mavutil.mavlink_connection("COM13", baud=57600)
m.wait_heartbeat(timeout=10)
m.mav.param_set_send(m.target_system, m.target_component,
    b"RC5_OPTION", 31.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
time.sleep(0.5)
m.mav.param_request_read_send(m.target_system, m.target_component, b"RC5_OPTION", -1)
msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)
val = msg.param_value if msg else "no reply"
print("RC5_OPTION =", val, "  (31 = Motor Emergency Stop)")
