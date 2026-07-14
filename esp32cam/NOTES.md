# ESP32-S3 CAM — as-flashed config & field notes

Everything learned bringing up the **Freenove ESP32-S3-WROOM CAM** (OV2640, wide lens) as
the drone's policy camera. Companion to [`README.md`](README.md) (how-to) — this file is
the *why* and the exact as-flashed state.

## As-flashed configuration (2026-07-08)

| Item | Value |
|---|---|
| Board | Freenove ESP32-S3-WROOM CAM — chip reads **N16R8**: 16 MB flash, **8 MB OPI PSRAM** |
| USB | CH343 UART bridge (`USB-Enhanced-SERIAL CH343`, was COM12). Auto-reset works, no BOOT hold |
| Firmware | `esp32cam_policy_stream/` — QVGA 320×240 JPEG, 20 MHz XCLK, `fb_count=2`, `CAMERA_GRAB_LATEST` |
| WiFi | SoftAP **`Ketu`** / **`12345678`**, stream `http://192.168.4.1:81/stream` (~30 fps MJPEG) |
| Sensor | `set_vflip(1)`, `set_hmirror(0)` — **verify orientation in the field** (hand-wave test) |
| FQBN | `esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,FlashMode=qio,PartitionScheme=huge_app,CDCOnBoot=default,USBMode=hwcdc,UploadMode=default,UploadSpeed=921600` |
| Toolchain | `arduino-cli` + esp32 core 3.3.10 (compile+upload headless; Serial @115200 on the CH343 port) |
| Pin map | **ESP32S3_EYE** (XCLK 15, SIOD 4, SIOC 5, D0-D7 = 11,9,8,10,12,18,17,16, VSYNC 6, HREF 7, PCLK 13) — NOT AI-Thinker |

Boot log on success ends with `stream: http://192.168.4.1:81/stream` — that line prints
only after `esp_camera_init()` succeeds.

## Calibration & geometry (why the numbers are what they are)

- **Lens**: marketed "160°" is the *diagonal*; `calib.npz` (cv2.fisheye, checkerboard) measured
  K≈[[112.9,0,150],[0,108.3,138]] with strong distortion. Equidistant reading ≈160° H / ≈127° V usable.
- **Sim camera truth**: Hazel `PerspectiveFOV: 80` is **VERTICAL** (`SetDegPerspectiveVerticalFOV`)
  → the sim front camera is **VFOV 80° / HFOV 96.3°** at 320×240. An earlier kit version wrongly
  targeted 80° *horizontal* (f=191, ~25 % over-zoom); fixed to **f=143 px**.
- **Policy feed**: one `cv2.remap` folds undistort + FOV match + the training 4:3→1:1 squash
  → 128×128 (fx=57.3, fy=76.3). Remap coverage verified 100 % on a real frame (no corner blowup).
- **CV feed** (`cv_hoop_pass.py`): same remap at 320×240, f=143 — pixel error ≈ angle error.

## Gotchas (each one cost real time)

1. **SSID collision**: first flashed as `zero` — same name as a nearby home network. Devices
   silently joined the home net (it was 5 GHz; the ESP is **2.4 GHz only**) and `192.168.4.1`
   timed out. Unique SSID (`Ketu`) fixed it. If the stream times out, first check the client's
   IP is `192.168.4.x`.
2. **SoftAP means no internet** on the client while connected — expected; laptops love to
   auto-switch back to a network with internet and silently drop the camera.
3. **Serial reads reset the board**: toggling DTR/RTS to read the boot log resets the ESP —
   the AP drops for ~3 s and kicks clients. Don't "verify" the AP while someone's streaming.
4. **MJPEG latency** is ~100–250 ms glass-to-Python. Fine for the 10 Hz control loop; if a
   future loop needs less, `esp32-cam-fpv` (raw WiFi injection) does 20–50 ms.
5. **Power/RF on the drone** (from vendor/field reports, not yet re-measured on ours): feed
   5 V ≥1 A with a 330 µF cap at the module; keep the camera flex ≥15 cm from GPS/compass.
6. **Hoop color thresholds** (`cv_hoop_pass.py` defaults) were measured on the real hoop photo:
   paint = H≈178 / S≈172; brown wood floor false-reds = H≈15 / S≤104 → bands H 0-10 ∪ 168-180,
   S≥90, V≥25 (V floor keeps the darker sim hoop detectable). Deep-shadow ring segments are
   inseparable from brown floor in HSV — a lit outdoor scene doesn't have this collision, and
   `--tune` exists for site conditions.

## Outdoor exposure blowout (2026-07-09)

First outdoor feed was massively blown out (white bloom across the frame). Two causes stack:
1. **Physical — check FIRST**: fingerprint/oil or leftover protective film on the lens produces
   exactly that center veiling glare; sun in/near the wide FOV adds flare. Clean the lens
   (isopropyl + microfiber), peel any film, and shade it (small hood / mount angle slightly down).
2. **Sensor AE aimed too bright for full sun.** Firmware now defaults to `ae_level=-2`,
   `aec2=1`, `gainceiling=2x`, `lenc/bpc/wpc on`, and adds a live-tuning endpoint:
   `http://192.168.4.1:81/ctrl?var=<name>&val=<n>` — vars: ae_level(-2..2), aec_value(0..1200,
   switches to manual exposure), aec(0/1), gainceiling(0..6), agc(0/1), wb_mode(0-4),
   brightness/contrast(-2..2), vflip/hmirror(0/1). Tune on site from any browser on `Ketu`;
   settings reset on reboot (defaults live in the sketch).

Note for CV: blowout kills the red-hoop mask (saturation collapses in clipped regions) — fix
exposure BEFORE running `cv_hoop_pass.py --tune`.

## Missing color = NO IR-CUT FILTER on the wide lens (2026-07-09, field video)

`cam_20260709_201423.mp4` (real field, hoop installed): not overexposed (V~127, 2% clipped)
but saturation dead flat (scene S mean 14-17 vs a normal outdoor 60-150). Signature is
textbook near-infrared contamination: foliage renders pale purple-white (chlorophyll is
bright in NIR), grass bone-gray, the red hoop washed PINK. Aftermarket 160-deg lenses
usually omit the IR-cut filter that stock OV2640 lenses have. **Software cannot remove
mixed-in IR.** Confirm with a TV remote: its LED will glow bright white on the feed.

**Measured hoop under IR contamination:** H 168-178 (hue survives), S only 25-53 (median 33),
V 66-225. `cv_hoop_pass` defaults (S>=90) DO NOT detect it; IR-adapted bands
(`--hsv 15,160,25,60`) detect cleanly on the field video. Until the lens is fixed, fly with
that fallback + on-site `--tune`; firmware now also defaults `saturation=+2` (tunable live:
`/ctrl?var=saturation&val=2`) to claw back chroma margin.

**Real fix: replace the M12 lens with one that HAS an IR-cut filter ("650nm IR filter" in
the listing), >=100-deg HFOV — we only need 96.3-deg horizontal for the sim match, so a
quality ~110-120-deg IR-cut lens beats the filterless 160-deg on every axis (~$8-15).**
Re-run calibrate_fisheye.py after any lens change (new K/D).
