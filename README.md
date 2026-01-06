# AeroTrackNow CanSat (Teensy 4.0)

Flight code for a guided CanSat with parafoil steering.

## Features
- Predictive landing guidance (steers to make predicted landing converge to target)
- GNSS (ZED-X20P) with RTK passthrough (Teensy forwards RTCM bytes to GNSS)
- LoRa (SX1262) bidirectional:
  - receive TARGET once
  - receive RTK continuously
  - send telemetry @ 1 Hz
- BMP280: pressure + temperature (and sink-rate estimate)
- BNO085: accelerometer (drop detection)
- Dual-servo differential steering
- No buzzer

## Folder structure
- `firmware/` – Teensy Arduino sketch
- `docs/` – pinout, telemetry, state machine docs
- `hardware/` – put KiCad files/netlists here

## Build
1. Install required Arduino libraries:
   - RadioLib
   - SparkFun u-blox GNSS Arduino Library
   - Adafruit BMP280 Library
   - Adafruit BNO08x
2. Open `firmware/cansat_flight.ino` in Arduino IDE.
3. Select board: **Teensy 4.0**, then upload.

## First tuning parameters
Inside `firmware/cansat_flight.ino`:
- `RANGE_US` (servo throw) – start small, increase carefully
- `KP_TURN`, `MAX_TURN`
- `TERMINAL_HEIGHT_M`, `CAPTURE_RADIUS_M`
- `SERVO_TRIM` (parafoil trim) - change in steps of 0.01..0.02

