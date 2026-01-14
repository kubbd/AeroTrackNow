# AeroTrackNow Flight Firmware Test Plan

**Document purpose:** Provide a structured, practical test plan for engineering validation and competition documentation of the Teensy 4.0 flight firmware in `firmware/cansat_flight.ino`. The plan is organized by test level and includes state machine, sensor validation, guidance logic, and failsafe coverage. It also defines optional compile-time test hooks and ground procedures. This test plan **must not alter flight behavior or telemetry packet formats** in production builds.

## 1) Scope & Constraints

- **System under test:** CanSat flight firmware (Teensy 4.0), state machine, guidance, sensors, LoRa telemetry, and servo actuation.
- **Sensors:** GNSS (ZED-X20P), BMP280 barometer, BNO085 IMU.
- **Actuators:** Two servos for parafoil steering.
- **Comms:** LoRa telemetry + RTK passthrough.
- **Safety constraint:** Tests must not change flight behavior or packet formats in non-test builds.

## 2) Test Levels

| Level | Goal | Typical Tools | Output Artifacts |
| --- | --- | --- | --- |
| **Unit** | Validate logic-level functions and math | Host build w/ test hooks, scripted input injection | CSV of expected vs actual values, unit log | 
| **Integration** | Validate sensor+logic interactions | Bench rig, sensor simulators, test hooks | Telemetry logs, plots | 
| **Hardware-in-the-loop (HIL)** | Validate flight hardware response | Flight stack, real servos, GNSS/IMU simulators | Telemetry logs, video | 
| **End-to-end** | Validate mission flow | Field tests (drop/glide/full) | Telemetry + GNSS tracks + plots | 

## 3) Test Data & Logging Requirements

**Log sources:**
- Flight SD card (`flight.csv`) at 1 Hz.
- Ground telemetry receiver (LoRa capture).
- Optional high-rate IMU/pressure logs (bench/HIL).

**Minimum fields to capture:**
- Time, state, lat/lon, height AGL proxy, sink rate, ground speed, predicted landing, servo commands.

**Plot checklist:**
- State vs time.
- Height AGL vs time.
- Sink rate vs time.
- Servo commands vs time.
- Predicted landing vs target distance.

---

# PART A — Test Plan Document

## A1) State Machine Tests

**Transitions under test:**
- BOOT → WAIT_FOR_DROP
- WAIT_FOR_DROP → WAIT_FOR_STABLE_DESCENT
- WAIT_FOR_STABLE_DESCENT → GUIDED_DESCENT
- GUIDED_DESCENT → TERMINAL
- TERMINAL → LANDED

| Transition | Trigger Condition | How to Simulate | Expected Behavior | Failure Cases |
| --- | --- | --- | --- | --- |
| **BOOT → WAIT_FOR_DROP** | `targetReceived == true` | Inject a valid TARGET packet over LoRa or simulate `targetReceived` via test hook | State advances to WAIT_FOR_DROP, servos remain neutral | No transition if CRC invalid or target packet missing; servos should remain neutral |
| **WAIT_FOR_DROP → WAIT_FOR_STABLE_DESCENT** | `dropDetected() == true` with sink rate > 1 m/s and accel spike | Inject IMU accel spike + baro sink rate via test hook; or shake+drop test rig | State advances, servos neutral | False positives from noise; if IMU not reporting, remain in WAIT_FOR_DROP |
| **WAIT_FOR_STABLE_DESCENT → GUIDED_DESCENT** | `descentStable()` for `STABLE_HOLD_MS` within sink range | Inject stable sink rate via baro hook; or controlled drop/slide on bench rig | State advances, guidance active, servo commands respond to heading error | Oscillating sink rate prevents transition; transition too early indicates threshold errors |
| **GUIDED_DESCENT → TERMINAL** | height AGL < `TERMINAL_HEIGHT_M` **and** pred-to-target < `CAPTURE_RADIUS_M` | Feed GNSS + baro to force altitude/position near target | Servos neutral, state goes to TERMINAL | If thresholds wrong, may remain in GUIDED_DESCENT (late) or go TERMINAL too early |
| **TERMINAL → LANDED** | `landingDetected()` (sink rate < 0.3 m/s for >1s) | Simulate near-zero sink rate and stable state on bench | State advances to LANDED, servos neutral | False landing in gusty conditions; failure to land if baro drift high |

## A2) Sensor Validation Tests

| Test | Stimulus | How to Simulate | Expected Behavior | Failure Cases |
| --- | --- | --- | --- | --- |
| **GNSS loss/recovery** | Drop GNSS updates for N seconds then resume | Disconnect GNSS TX or inject no updates with test hook | Lat/lon/alt freeze during loss; on recovery, updates resume cleanly | State machine should not crash; guidance should tolerate stale data |
| **Barometer noise/drift** | Inject noise and slow drift | Test hook: vary `baro_pressure_hPa` and `baro_sink_rate` | Sink rate smoothing should not create false transitions | Large drift could trigger premature landing or block stable descent |
| **IMU acceleration spikes** | Simulate rocket launch vs actual drop | Apply IMU spikes with varying sink rates | Only triggers drop when sink rate indicates descent | False drop triggers if threshold too low; missed drop if too high |

## A3) Guidance Logic Tests

| Test | Setup | Expected Behavior | Failure Cases |
| --- | --- | --- | --- |
| **Zero error → straight flight** | Predicted landing equals target | Servo commands near 0 (trim-only) | Servo bias or oscillation indicates incorrect heading error |
| **Large heading error → saturated turn** | Set error > max turn limit | Servo commands clamp to ±`MAX_TURN` | Exceeding clamp may overstress parafoil |
| **Trim bias compensation** | Apply constant wind bias + trim | Commands reflect trim, no drift in heading | Trim insufficient or saturates over time |
| **RTK available vs unavailable** | Toggle RTK stream | Guidance should run regardless; improved position in RTK | If guidance depends on RTK, it may fail in standalone GNSS mode |

## A4) Failsafe Tests

| Test | Stimulus | Expected Behavior | Failure Cases |
| --- | --- | --- | --- |
| **Servo disconnected/jammed** | Unplug servo or lock linkage | Commands still issued; telemetry shows commanded values | Firmware resets, watchdog, or stalls are failures |
| **GNSS timeout** | Stop GNSS updates | Guidance should hold last values; state machine should not crash | NaNs or invalid values propagate to telemetry |
| **Radio loss** | Turn off ground station | No effect on guidance; telemetry attempts continue | Guidance should not stop without radio |
| **Power reset mid-flight** | Power cycle after drop | System restarts to BOOT; waits for target | If target not re-received, servos neutral and no guidance |

---

# PART B — Embedded Test Hooks (Code)

## B1) Overview

Optional compile-time hooks allow controlled injection of sensor data, forcing state transitions, and disabling guidance. These hooks are **disabled by default** and do **not** change flight behavior unless `AEROTRACK_TEST_HOOKS` is defined at compile time.

## B2) Example Hook Usage (C/C++)

```cpp
// Compile with -DAEROTRACK_TEST_HOOKS to enable
// Example: Inject GNSS + baro and force state to GUIDED_DESCENT

testHooks.use_gnss = true;
testHooks.gnss_lat = 34.1234567;
testHooks.gnss_lon = -117.1234567;
testHooks.gnss_alt_msl = 150.0;
testHooks.gnss_speed = 8.5;
testHooks.gnss_track_rad = 1.57; // east

testHooks.use_baro = true;
testHooks.baro_pressure_hPa = 1008.0;
testHooks.baro_temp_c = 22.5;
testHooks.baro_sink_rate = 5.0;

testHooks.force_state = true;
testHooks.forced_state = GUIDED_DESCENT;
```

**Hook capabilities (implemented in `firmware/cansat_flight.ino`):**
- `use_gnss`, `use_baro`, `use_imu`: inject synthetic sensor data.
- `force_state`: directly set the state machine.
- `disable_guidance`: hold servos neutral while keeping state transitions intact.

---

# PART C — Ground-Based Test Procedures

## C1) Bench Test (No Movement)

**Goal:** Validate sensor bring-up, telemetry, and safe neutral servos.

**Steps:**
1. Power flight stack on bench; confirm SD logging starts.
2. Verify GNSS has fix (or inject with test hooks).
3. Ensure servos stay neutral in BOOT and WAIT_FOR_DROP.
4. Record 5 minutes of telemetry.

**Log/Plot:**
- Telemetry CSV, state vs time, pressure/temp stability, servo outputs.

**Pass/Fail:**
- Pass if state stays BOOT/WAIT_FOR_DROP without false transitions; servos neutral; telemetry frames valid.

## C2) Drop Test (Short Drop)

**Goal:** Validate drop detection and stable descent transition.

**Steps:**
1. Use a short drop (3–5 m) with safe recovery.
2. Ensure target is received before drop.
3. Drop payload and record telemetry.

**Log/Plot:**
- Accel spike, sink rate, state transition timestamps.

**Pass/Fail:**
- Pass if WAIT_FOR_DROP → WAIT_FOR_STABLE_DESCENT → GUIDED_DESCENT occurs without false triggers.

## C3) Glide Test (Parafoil)

**Goal:** Validate guidance response and servo behavior in air.

**Steps:**
1. Perform parafoil glide from known point with target loaded.
2. Monitor servo commands and predicted landing point.
3. Recover payload and retrieve logs.

**Log/Plot:**
- Predicted vs target distance, servo commands, track vs target bearing.

**Pass/Fail:**
- Pass if commands saturate appropriately for large errors and converge when near target.

## C4) Full Mission Rehearsal

**Goal:** End-to-end mission validation.

**Steps:**
1. Full system integration with LoRa, GNSS, IMU, baro, servos.
2. Conduct a full drop and guided descent.
3. Collect SD + ground logs.

**Log/Plot:**
- State timeline, AGL vs time, sink rate, servo commands, predicted landing vs target.

**Pass/Fail:**
- Pass if state machine progresses cleanly to LANDED and telemetry remains valid throughout.

---

## Output Requirements

- **Test report** (PDF or Markdown) with:
  - Test description and configuration
  - Log file references
  - Plots and pass/fail verdicts
- **Data retention:** raw logs and plots stored under a dated test folder.
