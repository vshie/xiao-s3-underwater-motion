# XIAO ESP32-S3 Underwater Motion Detector

Firmware for a [Seeed Studio XIAO ESP32-S3 Sense](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/)
that uses its onboard OV2640 camera as a **motion detector** and drives two
outputs when motion is seen. It also hosts a WiFi access point with a web page
for live camera preview and motion-detection tuning.

## What it does

- **Motion detection** by grayscale frame differencing on the camera stream
  (no cloud, no training data, runs entirely on-device).
- **Two outputs** that assert while *active* (motion + a tunable hysteresis
  hold time) and release when idle:
  | Output | Pin | Active state | Idle state |
  |--------|-----|--------------|------------|
  | Digital trigger | `D0` (GPIO1) | `HIGH` | `LOW` |
  | Servo-style PWM | `D1` (GPIO2) | `2000 µs` pulse @ 50 Hz | `1000 µs` pulse @ 50 Hz |
- **WiFi SoftAP + web UI** for an MJPEG preview and live tuning of:
  - per-pixel change threshold,
  - minimum changed-area percentage,
  - hysteresis hold time,
  - detection frame interval.
- **Settings persist to flash** (NVS) and survive reboots.

## Hardware notes

- Outputs `D0`/`D1` are 3.3 V logic. Buffer/level-shift if your downstream
  device needs more current or 5 V.
- ⚠️ **Underwater range:** 2.4 GHz WiFi is absorbed within centimeters of
  water. The web UI is intended for **bench/surface tuning** (or a dry housing
  with a surface antenna). The motion→output logic runs fully standalone and
  needs no WiFi connection.

## Build & flash (no IDE required)

This is a [PlatformIO](https://platformio.org/) project. With the board
connected over USB:

```bash
pio run                 # compile
pio run -t upload       # flash the connected board
pio device monitor      # serial monitor @ 115200
```

The XIAO ESP32-S3 exposes a native USB-Serial/JTAG port, so no manual
boot/reset button presses are normally needed.

## Using it

1. Flash the board and power it.
2. On a phone/laptop, join the WiFi network:
   - **SSID:** `XIAO-Motion`
   - **Password:** `underwater`
3. Open **http://192.168.4.1/** in a browser.
4. Watch the preview, tune the sliders, then **Save to flash**.

Change the SSID/password and output pins near the top of
[`src/main.cpp`](src/main.cpp).

## Project layout

```
platformio.ini          # board + build config
include/camera_pins.h    # OV2640 pin map for the XIAO ESP32-S3 Sense
include/web_index.h      # embedded tuning web page
src/main.cpp             # camera, motion detection, outputs, web server
```

## License

MIT
