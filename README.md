# AeroTrackNow CanSat (Teensy 4.0)

## Project overview
This repo holds the flight and ground code for a guided CanSat with parafoil steering. It’s built around a Teensy 4.0 and a LoRa link, with a simple Python ground station on the PC.

# ChatGPT Chat Link

https://chatgpt.com/share/695c7dbe-eb48-800c-9de2-c052dbff324f

## Flight system
The flight stack runs on a Teensy 4.0 and includes:

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

## Ground station
The ground station has two pieces that work together:

- **Ground radio node (Teensy + SX1262):** a USB-to-LoRa bridge.
- **PC ground station (Python):** reads telemetry, validates CRC, and shows the key numbers.

A Teensy is used on the ground because the SX1262 is an SPI radio that needs tight timing and interrupt handling. The Teensy handles those hardware details and exposes a simple USB serial link to the PC.

## Radio architecture
The ground radio node is intentionally “transparent.” It just moves bytes and doesn’t care about packet formats.

**What the firmware does:**
- Initialize SX1262 via RadioLib with the same pin mapping as flight hardware.
- Forward any USB serial bytes to LoRa (transparent transmit).
- Forward any LoRa packet bytes to USB (transparent receive).
- Operate non-blocking so serial and radio remain responsive.

**What the firmware does not do:**
- Parse packets or enforce application formats.
- Add CRC or message framing.
- Buffer or reorder payloads beyond RadioLib’s receive buffer.

**Data Flow Diagram (textual):**
PC (Python) ⇄ USB ⇄ Teensy 4.0 ⇄ SPI ⇄ SX1262 ⇄ Air ⇄ CanSat

## Folder structure
- `firmware/` – Teensy Arduino sketch
- `docs/` – pinout, telemetry, state machine docs
- `hardware/` – put KiCad files/netlists here

## How to get started
1. Skim `docs/TELEMETRY.md`, `docs/PINOUT.md`, and `docs/STATE_MACHINE.md` to understand the data formats and wiring.
2. Build and upload the flight firmware from `firmware/cansat_flight.ino`.
3. Build and upload the ground radio firmware from `firmware/ground_radio_teensy.ino`.
4. Run the Python ground station (`ground_station.py`) on your PC and point it at the Teensy USB port.

## Build instructions
1. Install required Arduino libraries:
   - RadioLib
   - SparkFun u-blox GNSS Arduino Library
   - Adafruit BMP280 Library
   - Adafruit BNO08x
2. Open `firmware/cansat_flight.ino` in Arduino IDE.
3. Select board: **Teensy 4.0**, then upload.

## Tuning parameters
Inside `firmware/cansat_flight.ino`:
- `RANGE_US` (servo throw) – start small, increase carefully
- `KP_TURN`, `MAX_TURN`
- `TERMINAL_HEIGHT_M`, `CAPTURE_RADIUS_M`
- `SERVO_TRIM` (parafoil trim) – change in steps of 0.01..0.02

**Why trim exists and how to tune it:**
Parafoils usually fly with a slight built-in bias. `SERVO_TRIM` lets you offset the neutral command so the canopy flies straight. Start at 0.0, do a short test glide, and nudge the trim by 0.01–0.02 until it tracks straight with neutral steering.

**How to disable guidance for testing:**
If you want to fly with no steering, set the guidance gains to zero (`KP_TURN = 0` and `MAX_TURN = 0`). The servos will stay neutral and the system will still log and transmit telemetry.
