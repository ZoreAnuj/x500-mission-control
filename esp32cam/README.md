# ESP32-CAM (OV2640, 160° lens) → Drone Hoop policy feed

Configures the onboard ESP32-CAM to stream **320×240 (QVGA) at ~30 fps** — the sim
front-camera resolution — and remaps its wide lens to the sim's **~80° rectilinear** FOV.

## Files
- `esp32cam_policy_stream.ino` — flash to the ESP32-CAM. QVGA / JPEG / 20 MHz / `fb_count=2`
  / `GRAB_LATEST`, SoftAP `dronecam`. Stream: `http://192.168.4.1:81/stream`.
- `calibrate_fisheye.py` — calibrate the fisheye → `calib.npz` (do this once).
- `esp32cam_capture.py` — grab newest frame + undistort to 80° pinhole (feeds the policy).

## 1. Flash
Arduino IDE → Board **"AI Thinker ESP32-CAM"**, PSRAM **enabled**. Open the `.ino`, set
`AP_SSID`/`AP_PASS` if you like, upload (GPIO0→GND to enter flash mode, then reset).
Serial @115200 prints the stream URL.

## 2. Verify
Join WiFi `dronecam`, open `http://192.168.4.1:81/stream` in a browser — should be ~30 fps.
If frames stripe or you see *"Failed to get frame on time"*, set `xclk_freq_hz = 10000000`.

## 3. Capture into Python
```
pip install opencv-python numpy
python esp32cam_capture.py --crop        # quick, no calibration (approx 80°)
```
Uses a drop-old-frames grab thread so you always read the freshest frame (~100–250 ms
glass-to-Python over MJPEG). If that latency blocks closed-loop control later, switch to a
UDP/packet-injection firmware (`esp32-cam-fpv`, ~20–50 ms).

## 4. FOV match — the part that actually matters for sim2real
The policy was trained on a **~80° rectilinear** front camera. Your lens is **wide** — and
the "160°" is almost certainly the *diagonal*; the real **horizontal** FOV on a 4:3 OV2640
is usually ~120–135°. Wrong FOV/distortion changes the apparent size & bearing of the hoop,
so the policy mis-judges range and steering (worst at frame edges, exactly where the hoop
sits during turns).

Do this once:
1. Print a 9×6 checkerboard, run `python calibrate_fisheye.py`, grab ≥15 views filling the
   corners, press **C**. It writes `calib.npz` and prints the measured HFOV.
2. Run `python esp32cam_capture.py --undistort` → rectilinear 80° 320×240 = genuine sim match.

`--crop` is a first pass only: the cropped image is still barrel-distorted, matching the sim
just near center. Prefer `--undistort`. Residual gap → hedge with FOV/distortion domain
randomization in sim and/or DAgger on the real undistorted frames.

## Gotchas (from field reports)
- **Brownout:** camera+WiFi peaks ~500 mA. Feed 5 V from a solid ≥1 A BEC with a **330 µF cap
  across 5V/GND** at the module. The sketch masks the detector, but *fix the supply* on a
  flying vehicle — don't rely on the mask.
- **RF interference:** the camera flex radiates broadband noise and can **jam GPS** within
  ~7–15 cm. Keep the module/flex ≥15 cm from GPS and away from the Pixhawk/compass.
- **WiFi:** ESP32-CAM is 2.4 GHz, weak antenna — fine for close hoop work, marginal at range.
  SiK telemetry (433/915 MHz) doesn't clash; 2.4 GHz RC would.
- **Orientation:** mount forward-facing, rigid (JPEG smears on vibration). Match training
  orientation with `set_vflip`/`set_hmirror` in the sketch (live gRPC frames were upside-down).
