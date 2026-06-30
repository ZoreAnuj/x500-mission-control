"""Are ANY bytes arriving on COM13? Distinguishes 'link down' from 'parse issue'."""
import time
import serial

s = serial.Serial("COM13", 57600, timeout=0.5)
total = 0
sample = b""
t0 = time.time()
while time.time() - t0 < 6:
    d = s.read(512)
    if d:
        total += len(d)
        if not sample:
            sample = d[:48]
s.close()
print(f"bytes in 6s @57600: {total}")
print(f"sample: {sample.hex(' ') if sample else '(none)'}")
print("MAVLink magic 0xFD/0xFE present:" ,
      ("FD" in sample.hex(' ').upper() or "FE" in sample.hex(' ').upper()) if sample else False)
