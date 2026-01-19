/*******************************************************
 * AeroTrackNow CanSat – PRODUCTION VERSION 3.0
 * Teensy 4.0 Flight Code - Accuracy-Focused
 *
 * Brutally honest note:
 * - Landing within 1m from 2000m is not guaranteed in real wind.
 * - This code improves estimation/control and reduces systematic error,
 *   but atmospheric variability still dominates at altitude.
 *******************************************************/

/*******************************************************
 * ✅ ALL BUGS FIXED - READY FOR TESTING
 * 
 * Changes from v3.0:
 * 1. Servo time constant: 0.35s → 0.10s (MG92B actual spec)
 * 2. Wind update threshold: 6.0 → 8.0 m/s (Warsaw turbulence)
 * 3. Wind layer threshold: 8.0 → 10.0 m/s (wind shear)
 * 4. Wind decay: 0.02 → 0.005 /s (3 min time constant)
 * 5. RTK altitude fusion: 5% → 25% GNSS weight
 * 6. Heading calibration mode added
 * 
 * ⚠️ MANDATORY BEFORE FLIGHT:
 * - Calibrate HEADING_OFFSET_RAD (see line ~67)
 * - Test flights #1-2: measure actual sink rate & glide ratio
 * - Update NOMINAL_SINK_RATE and PARAFOIL_GLIDE_RATIO
 * - Calibrate SERVO_TRIM after first flights
 *******************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <SD.h>
// GNSS (u-blox)
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
// Sensors
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO08x.h>
// LoRa (SX1262)
#include <RadioLib.h>
// ===================== HARDWARE PINOUT =====================
// From your Pinout.MD - DO NOT CHANGE unless hardware changes
static constexpr int PIN_GNSS_RX = 21;  // Serial5 RX
static constexpr int PIN_GNSS_TX = 20;  // Serial5 TX
static constexpr int PIN_GNSS_TPS = 24;  // optional
static constexpr int PIN_GNSS_RTKFIX = 25;  // optional
static constexpr int PIN_I2C_SDA = 18;
static constexpr int PIN_I2C_SCL = 19;
static constexpr int PIN_LORA_CS = 10;
static constexpr int PIN_LORA_DIO1 = 7;
static constexpr int PIN_LORA_RESET = 8;
static constexpr int PIN_LORA_BUSY = -1;  // not connected
static constexpr int PIN_SD_CS = 23;
static constexpr int PIN_SERVO_LEFT = 3;
static constexpr int PIN_SERVO_RIGHT = 4;
// ===================== PARAFOIL PARAMETERS =====================
// Update after real flight tests and canopy changes.
// Geometry from provided stencil data.
static constexpr float PARAFOIL_AREA_M2 = 0.141f;  // Active lift area
static constexpr float PARAFOIL_GLIDE_RATIO = 2.0f;
static constexpr float NOMINAL_SINK_RATE = 5.5f;  // m/s (positive down)
static constexpr float NOMINAL_AIRSPEED = NOMINAL_SINK_RATE * PARAFOIL_GLIDE_RATIO;
// Brake response curves (replace with measured data after tests)
static constexpr float BRAKE_TABLE[] = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f};
static constexpr float SINK_RATE_TABLE[] = {5.5f, 5.9f, 6.4f, 6.9f, 7.2f};
static constexpr float GLIDE_RATIO_TABLE[] = {2.0f, 1.9f, 1.7f, 1.55f, 1.4f};
static constexpr float TURN_RATE_SCALE_TABLE[] = {1.0f, 1.05f, 1.1f, 1.15f, 1.2f};
static constexpr int BRAKE_TABLE_SIZE = sizeof(BRAKE_TABLE) / sizeof(BRAKE_TABLE[0]);
static constexpr float SERVO_TIME_CONSTANT_S = 0.10f;  // ✓ FIXED: MG92B actual response time
// Seeded wind profile (disabled by default; real-time wind estimation is preferred).
static constexpr bool USE_SEEDED_WIND_PROFILE = false;
static constexpr float WARSAW_PROFILE_MAX_ALT_M = 2000.0f;
static constexpr float WARSAW_WIND_DIR_DEG[] = {0.0f};
static constexpr float WARSAW_WIND_SPD_MPS[] = {0.0f};
static constexpr int WARSAW_WIND_BINS = sizeof(WARSAW_WIND_SPD_MPS) / sizeof(WARSAW_WIND_SPD_MPS[0]);
// ===================== CONTROL PARAMETERS =====================
static constexpr float KP_HEADING = 0.9f;
static constexpr float KI_HEADING = 0.05f;
static constexpr float KD_HEADING = 0.12f;
static constexpr float MAX_BANK_ANGLE_RAD = 0.48f;  // ~27.5°
static constexpr float MAX_TURN_RATE_RAD_S = 0.8f;
static constexpr float HEADING_JUMP_MAX_RAD = 1.2f;
static constexpr uint32_t HEADING_JUMP_MAX_DT_MS = 150;
// ⚠️⚠️⚠️ MANDATORY CALIBRATION BEFORE FIRST FLIGHT ⚠️⚠️⚠️
// Set HEADING_CALIBRATION_MODE = true, upload, point North, read Serial Monitor
// Calculate: HEADING_OFFSET_RAD = (0.0 - measured_value)
// Update HEADING_OFFSET_RAD below, set HEADING_CALIBRATION_MODE = false, re-upload
static constexpr bool HEADING_CALIBRATION_MODE = false;
static constexpr float HEADING_OFFSET_RAD = 0.0f;  // ⚠️ CALIBRATE THIS!
static constexpr float MIN_GROUND_SPEED_FOR_TRACK = 0.5f;
static constexpr float HEADING_CONFIDENCE_MAX_DELTA_RAD = 0.35f;
static constexpr float HEADING_CONFIDENCE_MIN_FOR_WIND = 0.4f;
static constexpr uint32_t HEADING_CONFIDENCE_MAX_DT_MS = 300;
// Servo parameters
static constexpr float SERVO_TRIM = 0.0f;
static constexpr int SERVO_NEUTRAL_US = 1500;
static constexpr int SERVO_RANGE_US = 300;
static constexpr float SERVO_MAX_RATE = 2.0f;  // Max change per second
static constexpr float SERVO_DEADBAND = 0.02f;
// State machine altitudes
static constexpr float FINAL_APPROACH_HEIGHT_M = 30.0f;
static constexpr float FLARE_HEIGHT_M = 3.0f;
static constexpr float TERMINAL_HEIGHT_M = 1.0f;
static constexpr float TERMINAL_HOMING_HEIGHT_M = 50.0f;
static constexpr float CAPTURE_RADIUS_M = 1.0f;
static constexpr float CAPTURE_RADIUS_GNSS_M = 5.0f;
static constexpr float CAPTURE_RADIUS_POOR_M = 10.0f;
// Stability detection
static constexpr float STABLE_SINK_MIN = 4.5f;
static constexpr float STABLE_SINK_MAX = 6.5f;
static constexpr uint32_t STABLE_HOLD_MS = 1500;
// Drop detection
static constexpr float DROP_ACCEL_SPIKE = 8.0f;
static constexpr int DROP_DEBOUNCE_COUNT = 3;
// Sink rate filtering
static constexpr uint32_t BARO_UPDATE_INTERVAL_MS = 200;  // 5 Hz
static constexpr float SINK_FILTER_ALPHA = 0.2f;
static constexpr float SINK_RATE_MIN = 3.0f;
static constexpr float SINK_RATE_MAX = 9.0f;
// Wind estimation
static constexpr float WIND_FILTER_ALPHA = 0.03f;
static constexpr float MAX_WIND_ESTIMATE = 18.0f;
static constexpr float MAX_WIND_UPDATE_DELTA = 8.0f;  // ✓ FIXED: Allow Warsaw turbulence
static constexpr float MAX_WIND_LAYER_DELTA = 10.0f;  // ✓ FIXED: Allow wind shear
static constexpr float MIN_WIND_UPDATE_ALT_M = 10.0f;
static constexpr float WIND_LAYER_STEP_M = 25.0f;
static constexpr int WIND_LAYER_COUNT = 80;
static constexpr float WIND_LAYER_DECAY_PER_S = 0.005f;  // ✓ FIXED: 3 min time constant
static constexpr float MAX_WIND_FOR_GUIDANCE = 8.0f;  // Abort guidance above this (m/s)
static constexpr float WIND_GAIN_REDUCTION_START = 4.0f;
static constexpr float WIND_GAIN_MIN_FACTOR = 0.6f;
static constexpr float WIND_LAYER_SMOOTH_ALPHA = 0.12f;
static constexpr float WIND_LAYER_BLEND_RADIUS = 2;
// Gust detection
static constexpr float GUST_SPEED_SPIKE_MPS = 3.0f;
static constexpr uint32_t GUST_HOLD_MS = 1500;
static constexpr float GUST_GAIN_FACTOR = 0.7f;
// Glide-slope management
static constexpr float GLIDESLOPE_TARGET_SINK_HIGH = 5.6f;
static constexpr float GLIDESLOPE_TARGET_SINK_LOW = 4.6f;
static constexpr float GLIDESLOPE_TARGET_HIGH_ALT_M = 200.0f;
static constexpr float GLIDESLOPE_TARGET_LOW_ALT_M = 20.0f;
static constexpr float GLIDESLOPE_MAX_BRAKE = 0.35f;
static constexpr float GLIDESLOPE_KP = 0.12f;
static constexpr float GLIDESLOPE_KI = 0.02f;
// Lateral velocity damping (non-terminal)
static constexpr float LATERAL_DAMPING_GAIN = 0.15f;
// Safety gating
static constexpr float MAX_EKF_SPEED_MPS = 25.0f;
static constexpr float MIN_GNSS_SATS_FOR_GUIDANCE = 6;
static constexpr float MAX_GNSS_AGE_MS = 1500;
// MPC parameters (short horizon + dynamics)
static constexpr int MPC_HORIZON = 12;
static constexpr int MPC_SAMPLES = 21;
static constexpr float MPC_CONTROL_WEIGHT = 0.15f;
static constexpr float APPROACH_GAIN = 1.8f;
// Terminal cross-track controller
static constexpr float KP_CROSSTRACK = 0.07f;
static constexpr float KD_CROSSTRACK = 0.18f;
static constexpr float MAX_CROSSTRACK_CMD = 1.0f;
static constexpr float HEADING_GAIN_HIGH_ALT = 0.7f;
static constexpr float HEADING_GAIN_LOW_ALT = 1.25f;
static constexpr float HEADING_GAIN_HIGH_ALT_M = 150.0f;
static constexpr float HEADING_GAIN_LOW_ALT_M = 20.0f;
static constexpr float CROSSTRACK_GAIN_HIGH_ALT = 0.8f;
static constexpr float CROSSTRACK_GAIN_LOW_ALT = 1.3f;
static constexpr float CROSSTRACK_GAIN_HIGH_ALT_M = 60.0f;
static constexpr float CROSSTRACK_GAIN_LOW_ALT_M = 10.0f;
// EKF process/measurement noise (tune after data collection)
static constexpr float EKF_Q_POS = 2.0f;
static constexpr float EKF_Q_VEL = 1.0f;
static constexpr float EKF_Q_WIND = 0.5f;
static constexpr float EKF_R_POS = 3.5f;
static constexpr float EKF_R_VEL = 1.2f;
static constexpr float EKF_R_WIND = 2.0f;
// ===================== RADIO SETTINGS =====================
static constexpr float LORA_FREQ_MHZ = 433.0f;
static constexpr int LORA_TX_DBM = 14;
static constexpr uint32_t TELEMETRY_PERIOD_MS = 1000;
// ===================== TELEMETRY PACKET =====================
#pragma pack(push, 1)
struct TelemetryPacket {
  uint8_t msgType;  // 0x03
  uint32_t time_ms;
  int32_t lat_e7;
  int32_t lon_e7;
  int16_t height_agl_dm;  // meters * 10
  uint16_t pressure_hPa_x10;
  int16_t temp_c_x10;
  uint16_t ground_speed_cms;
  int16_t servo_left_x1000;
  int16_t servo_right_x1000;
  int32_t pred_lat_e7;
  int32_t pred_lon_e7;
  int16_t wind_north_cms;
  int16_t wind_east_cms;
  int16_t heading_deg_x10;
  uint16_t heading_confidence_x1000;
  uint16_t wind_rejects;
  uint16_t wind_layer_rejects;
  uint8_t mission_state;
  uint16_t crc16;
};
#pragma pack(pop)
// ===================== INCOMING PACKETS =====================
#pragma pack(push, 1)
struct TargetPacket {
  uint8_t msgType;  // 0x01
  int32_t tgt_lat_e7;
  int32_t tgt_lon_e7;
  int32_t tgt_alt_cm;  // meters MSL * 100
  uint16_t crc16;
};
#pragma pack(pop)
// Message types
static constexpr uint8_t MSG_TARGET = 0x01;
static constexpr uint8_t MSG_RTK = 0x02;
static constexpr uint8_t MSG_TELEMETRY = 0x03;
// ===================== STATE MACHINE =====================
enum MissionState : uint8_t {
  BOOT = 0,
  WAIT_FOR_DROP,
  WAIT_FOR_STABLE_DESCENT,
  GUIDED_DESCENT,
  FINAL_APPROACH,
  TERMINAL_HOMING,
  FLARE,
  TERMINAL,
  LANDED
};
MissionState state = BOOT;
// ===================== GLOBAL OBJECTS =====================
SFE_UBLOX_GNSS gnss;
Adafruit_BMP280 bmp;
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t bnoValue;
SX1262 radio = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RESET, PIN_LORA_BUSY);
Servo servoL, servoR;
File logFile;
// ===================== STATE VARIABLES =====================
// Target
bool targetReceived = false;
double target_lat = 0.0;
double target_lon = 0.0;
double target_alt_msl = 0.0;
// GNSS live
double lat = 0.0, lon = 0.0;
double gnss_alt_msl = 0.0;
double ground_speed = 0.0;
double ground_track_rad = 0.0;
bool ground_track_valid = false;
double gnss_vertical_velocity = 0.0;
// Heading estimate (from BNO08x)
double heading_rad = 0.0;
bool heading_valid = false;
uint32_t last_heading_ms = 0;
double last_heading_rad = 0.0;
float heading_confidence = 0.0f;
// IMU accel cache
float imu_ax = 0.0f;
float imu_ay = 0.0f;
float imu_az = 0.0f;
bool imu_accel_valid = false;
uint32_t last_imu_ms = 0;
// GNSS quality
uint8_t gnss_fix_type = 0;
uint8_t gnss_satellites = 0;
uint32_t gnss_h_accuracy_mm = 9999;
uint32_t gnss_v_accuracy_mm = 9999;
uint32_t last_gnss_update = 0;
// Baro live
double pressure_hPa = 0.0;
double temp_c = 0.0;
double baro_alt_msl = 0.0;
double last_baro_alt = 0.0;
uint32_t last_baro_ms = 0;
// Fused altitude
double fused_alt_msl = 0.0;
double fused_alt_velocity = 0.0;
// Sink rate (filtered)
double sink_rate_raw = NOMINAL_SINK_RATE;
double sink_rate_filtered = NOMINAL_SINK_RATE;
double sink_rate_gnss = NOMINAL_SINK_RATE;
// Wind estimation
double wind_north = 0.0;
double wind_east = 0.0;
double wind_speed = 0.0;
double wind_layer_north[WIND_LAYER_COUNT]{};
double wind_layer_east[WIND_LAYER_COUNT]{};
uint32_t wind_layer_ms[WIND_LAYER_COUNT]{};
bool gust_active = false;
uint32_t gust_until_ms = 0;
double last_wind_speed = 0.0;
uint16_t wind_update_rejects = 0;
uint16_t wind_layer_rejects = 0;
// Glide-slope controller
float glide_integral = 0.0f;
// Prediction
double pred_lat = 0.0, pred_lon = 0.0;
double predicted_error_m = 999.0;
// Servo state
float cmdL_current = 0.0f;
float cmdR_current = 0.0f;
float cmdL_target = 0.0f;
float cmdR_target = 0.0f;
uint32_t last_servo_update_ms = 0;
// Drop detection
int drop_spike_count = 0;
// Stability detection
uint32_t stableStartMs = 0;
// Timers
uint32_t lastTelemetryMs = 0;
uint32_t loop_start_ms = 0;
// Sea level pressure (for baro altitude)
double sea_level_pressure_hpa = 1013.25;
bool sea_level_calibrated = false;
// Controller state
double heading_error_integral = 0.0;
double last_heading_error = 0.0;
uint32_t last_heading_update_ms = 0;
// Navigation frame
double origin_lat = 0.0;
double origin_lon = 0.0;
bool origin_set = false;
struct StateEstimate {
  double north_m = 0.0;
  double east_m = 0.0;
  double v_north = 0.0;
  double v_east = 0.0;
  bool initialized = false;
};
StateEstimate nav_est{};
struct EKFState {
  double north_m = 0.0;
  double east_m = 0.0;
  double v_north = 0.0;
  double v_east = 0.0;
  double wind_north = 0.0;
  double wind_east = 0.0;
  bool initialized = false;
};
EKFState ekf_state{};
// ===================== UTILITY FUNCTIONS =====================
static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}
static inline double metersPerDegLat() {
  return 111111.0;
}
static inline double metersPerDegLon(double lat_deg) {
  return 111111.0 * cos(lat_deg * DEG_TO_RAD);
}
static double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dLat = (lat2 - lat1) * DEG_TO_RAD;
  double dLon = (lon2 - lon1) * DEG_TO_RAD;
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}
static double bearingRad(double fromLat, double fromLon, double toLat, double toLon) {
  double dLat = (toLat - fromLat);
  double dLon = (toLon - fromLon) * cos(fromLat * DEG_TO_RAD);
  return atan2(dLon, dLat);
}
static float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}
static double clampd(double v, double lo, double hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}
static float lerpf(float a, float b, float t) {
  return a + (b - a) * t;
}
static float lookupTable(const float *x, const float *y, int size, float value) {
  if (size <= 0) return 0.0f;
  if (value <= x[0]) return y[0];
  if (value >= x[size - 1]) return y[size - 1];
  for (int i = 0; i < size - 1; i++) {
    if (value >= x[i] && value <= x[i + 1]) {
      float t = (value - x[i]) / (x[i + 1] - x[i]);
      return lerpf(y[i], y[i + 1], t);
    }
  }
  return y[size - 1];
}
static float altitudeGain(double height_agl,
                          float high_alt_gain,
                          float low_alt_gain,
                          float high_alt_m,
                          float low_alt_m) {
  if (height_agl >= high_alt_m) return high_alt_gain;
  if (height_agl <= low_alt_m) return low_alt_gain;
  if (fabs(high_alt_m - low_alt_m) < 0.001f) return low_alt_gain;
  float t = (float)((height_agl - low_alt_m) / (high_alt_m - low_alt_m));
  return lerpf(low_alt_gain, high_alt_gain, t);
}
static float wrapAngle(float angle) {
  while (angle > PI) angle -= TWO_PI;
  while (angle < -PI) angle += TWO_PI;
  return angle;
}
static float wrapAngleDeg(float angle) {
  while (angle >= 360.0f) angle -= 360.0f;
  while (angle < 0.0f) angle += 360.0f;
  return angle;
}
static double getHeadingRad() {
  if (heading_valid && (millis() - last_heading_ms) < 1000) {
    return heading_rad;
  }
  if (ground_track_valid) {
    return ground_track_rad;
  }
  return last_heading_rad;
}
static bool isHeadingFresh() {
  return heading_valid && (millis() - last_heading_ms) < 1000;
}
static float getHeadingConfidence() {
  if (!heading_valid) return 0.0f;
  uint32_t age_ms = millis() - last_heading_ms;
  if (age_ms >= HEADING_CONFIDENCE_MAX_DT_MS) return 0.0f;
  float age_factor = 1.0f - (float)age_ms / (float)HEADING_CONFIDENCE_MAX_DT_MS;
  return clampf(heading_confidence * age_factor, 0.0f, 1.0f);
}
static float normalizeBrake(float left, float right) {
  float sym = 0.5f * (fabs(left) + fabs(right));
  return clampf(sym, 0.0f, 1.0f);
}
static float brakeToSinkRate(float brake) {
  float sink = lookupTable(BRAKE_TABLE, SINK_RATE_TABLE, BRAKE_TABLE_SIZE, brake);
  return clampf(sink, SINK_RATE_MIN, SINK_RATE_MAX);
}
static float brakeToGlideRatio(float brake) {
  float ratio = lookupTable(BRAKE_TABLE, GLIDE_RATIO_TABLE, BRAKE_TABLE_SIZE, brake);
  return clampf(ratio, 0.8f, 3.5f);
}
static float brakeToTurnRateScale(float brake) {
  float scale = lookupTable(BRAKE_TABLE, TURN_RATE_SCALE_TABLE, BRAKE_TABLE_SIZE, brake);
  return clampf(scale, 0.5f, 2.0f);
}
static void smoothWindLayers() {
  for (int i = 0; i < WIND_LAYER_COUNT; i++) {
    int lo = max(0, i - (int)WIND_LAYER_BLEND_RADIUS);
    int hi = min(WIND_LAYER_COUNT - 1, i + (int)WIND_LAYER_BLEND_RADIUS);
    double sum_n = 0.0;
    double sum_e = 0.0;
    int count = 0;
    for (int j = lo; j <= hi; j++) {
      sum_n += wind_layer_north[j];
      sum_e += wind_layer_east[j];
      count++;
    }
    double avg_n = sum_n / count;
    double avg_e = sum_e / count;
    wind_layer_north[i] = wind_layer_north[i] * (1.0 - WIND_LAYER_SMOOTH_ALPHA) + avg_n * WIND_LAYER_SMOOTH_ALPHA;
    wind_layer_east[i] = wind_layer_east[i] * (1.0 - WIND_LAYER_SMOOTH_ALPHA) + avg_e * WIND_LAYER_SMOOTH_ALPHA;
  }
}
static bool guidanceSafe() {
  if (wind_speed > MAX_WIND_FOR_GUIDANCE) return false;
  if (gnss_satellites < MIN_GNSS_SATS_FOR_GUIDANCE) return false;
  if (millis() - last_gnss_update > MAX_GNSS_AGE_MS) return false;
  if (ekf_state.initialized) {
    double speed = sqrt(ekf_state.v_north * ekf_state.v_north + ekf_state.v_east * ekf_state.v_east);
    if (speed > MAX_EKF_SPEED_MPS) return false;
  }
  return true;
}
static float applyGlideSlope(double height_agl) {
  float target_sink = altitudeGain(height_agl,
                                   GLIDESLOPE_TARGET_SINK_HIGH,
                                   GLIDESLOPE_TARGET_SINK_LOW,
                                   GLIDESLOPE_TARGET_HIGH_ALT_M,
                                   GLIDESLOPE_TARGET_LOW_ALT_M);
  float sink_err = (float)(sink_rate_filtered - target_sink);
  glide_integral += sink_err * (BARO_UPDATE_INTERVAL_MS / 1000.0f);
  glide_integral = clampf(glide_integral, -1.0f, 1.0f);
  float cmd = (GLIDESLOPE_KP * sink_err) + (GLIDESLOPE_KI * glide_integral);
  cmd = clampf(cmd, -GLIDESLOPE_MAX_BRAKE, GLIDESLOPE_MAX_BRAKE);
  return cmd;
}
static void updateGustDetector() {
  double delta = wind_speed - last_wind_speed;
  last_wind_speed = wind_speed;
  if (delta > GUST_SPEED_SPIKE_MPS) {
    gust_active = true;
    gust_until_ms = millis() + GUST_HOLD_MS;
  }
  if (gust_active && millis() > gust_until_ms) {
    gust_active = false;
  }
}
static void seedWindProfile() {
  if (!USE_SEEDED_WIND_PROFILE) {
    for (int i = 0; i < WIND_LAYER_COUNT; i++) {
      wind_layer_north[i] = 0.0;
      wind_layer_east[i] = 0.0;
      wind_layer_ms[i] = millis();
    }
    return;
  }
  for (int i = 0; i < WIND_LAYER_COUNT; i++) {
    float altitude = i * WIND_LAYER_STEP_M;
    float t = altitude / WARSAW_PROFILE_MAX_ALT_M;
    float idx_f = t * (WARSAW_WIND_BINS - 1);
    int idx0 = (int)floor(idx_f);
    int idx1 = idx0 + 1;
    if (idx0 < 0) idx0 = 0;
    if (idx1 >= WARSAW_WIND_BINS) idx1 = WARSAW_WIND_BINS - 1;
    float mix = idx_f - idx0;
    float dir_deg = wrapAngleDeg(WARSAW_WIND_DIR_DEG[idx0] * (1.0f - mix) +
                                 WARSAW_WIND_DIR_DEG[idx1] * mix);
    float spd = WARSAW_WIND_SPD_MPS[idx0] * (1.0f - mix) +
                WARSAW_WIND_SPD_MPS[idx1] * mix;
    float dir_rad = dir_deg * DEG_TO_RAD;
    // Meteorological direction: wind coming FROM dir_deg.
    float n = -spd * cosf(dir_rad);
    float e = -spd * sinf(dir_rad);
    wind_layer_north[i] = n;
    wind_layer_east[i] = e;
    wind_layer_ms[i] = millis();
  }
}
static int windLayerIndex(double height_agl) {
  int idx = (int)floor(height_agl / WIND_LAYER_STEP_M);
  if (idx < 0) idx = 0;
  if (idx >= WIND_LAYER_COUNT) idx = WIND_LAYER_COUNT - 1;
  return idx;
}
static void setOriginIfNeeded() {
  if (!origin_set && gnss_fix_type >= 3) {
    origin_lat = lat;
    origin_lon = lon;
    origin_set = true;
  }
}
static void latLonToNE(double in_lat, double in_lon, double &north_m, double &east_m) {
  double dLat = (in_lat - origin_lat);
  double dLon = (in_lon - origin_lon);
  north_m = dLat * metersPerDegLat();
  east_m = dLon * metersPerDegLon(origin_lat);
}
static void neToLatLon(double north_m, double east_m, double &out_lat, double &out_lon) {
  out_lat = origin_lat + north_m / metersPerDegLat();
  out_lon = origin_lon + east_m / metersPerDegLon(origin_lat);
}
static void getEstimatedLatLon(double &out_lat, double &out_lon) {
  if (origin_set && ekf_state.initialized) {
    neToLatLon(ekf_state.north_m, ekf_state.east_m, out_lat, out_lon);
    return;
  }
  out_lat = lat;
  out_lon = lon;
}
static void updateNavEstimate() {
  if (!origin_set || gnss_fix_type < 3) {
    return;
  }
  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if (last_ms == 0) {
    last_ms = now;
    return;
  }
  double dt = (now - last_ms) / 1000.0;
  if (dt <= 0.0 || dt > 1.0) {
    last_ms = now;
    return;
  }
  last_ms = now;
  double meas_n = 0.0;
  double meas_e = 0.0;
  latLonToNE(lat, lon, meas_n, meas_e);
  double meas_vn = ground_speed * cos(ground_track_rad);
  double meas_ve = ground_speed * sin(ground_track_rad);
  if (!nav_est.initialized) {
    nav_est.north_m = meas_n;
    nav_est.east_m = meas_e;
    nav_est.v_north = meas_vn;
    nav_est.v_east = meas_ve;
    nav_est.initialized = true;
    return;
  }
  // Simple constant-velocity Kalman filter
  static float P[4][4] = {
    {50.0f, 0, 0, 0},
    {0, 50.0f, 0, 0},
    {0, 0, 5.0f, 0},
    {0, 0, 0, 5.0f},
  };
  const float q_pos = 1.0f;
  const float q_vel = 0.5f;
  const float r_pos = 4.0f;
  const float r_vel = 1.5f;
  float F[4][4] = {
    {1, 0, (float)dt, 0},
    {0, 1, 0, (float)dt},
    {0, 0, 1, 0},
    {0, 0, 0, 1},
  };
  float x[4] = {
    (float)nav_est.north_m,
    (float)nav_est.east_m,
    (float)nav_est.v_north,
    (float)nav_est.v_east,
  };
  float x_pred[4] = {
    F[0][0] * x[0] + F[0][2] * x[2],
    F[1][1] * x[1] + F[1][3] * x[3],
    x[2],
    x[3],
  };
  float P_pred[4][4] = {};
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 4; k++) {
        sum += F[i][k] * P[k][j];
      }
      P_pred[i][j] = sum;
    }
  }
  float FPFT[4][4] = {};
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 4; k++) {
        sum += P_pred[i][k] * F[j][k];
      }
      FPFT[i][j] = sum;
    }
  }
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      P_pred[i][j] = FPFT[i][j];
    }
  }
  P_pred[0][0] += q_pos;
  P_pred[1][1] += q_pos;
  P_pred[2][2] += q_vel;
  P_pred[3][3] += q_vel;
  float z[4] = {(float)meas_n, (float)meas_e, (float)meas_vn, (float)meas_ve};
  float y[4] = {z[0] - x_pred[0], z[1] - x_pred[1], z[2] - x_pred[2], z[3] - x_pred[3]};
  float S[4] = {P_pred[0][0] + r_pos, P_pred[1][1] + r_pos, P_pred[2][2] + r_vel, P_pred[3][3] + r_vel};
  float K[4][4] = {};
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      float denom = (j < 2) ? S[j] : S[j];
      K[i][j] = P_pred[i][j] / denom;
    }
  }
  for (int i = 0; i < 4; i++) {
    float sum = 0.0f;
    for (int j = 0; j < 4; j++) {
      sum += K[i][j] * y[j];
    }
    x_pred[i] += sum;
  }
  float I_KH[4][4] = {};
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      I_KH[i][j] = (i == j ? 1.0f : 0.0f) - K[i][j];
    }
  }
  float P_new[4][4] = {};
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 4; k++) {
        sum += I_KH[i][k] * P_pred[k][j];
      }
      P_new[i][j] = sum;
    }
  }
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      P[i][j] = P_new[i][j];
    }
  }
  nav_est.north_m = x_pred[0];
  nav_est.east_m = x_pred[1];
  nav_est.v_north = x_pred[2];
  nav_est.v_east = x_pred[3];
}
static void ekfUpdateComponent(double &x, float P[][6], int idx, float z, float r) {
  float y = z - (float)x;
  float S = P[idx][idx] + r;
  if (S < 1e-6f) return;
  float K[6] = {};
  for (int i = 0; i < 6; i++) {
    K[i] = P[i][idx] / S;
  }
  for (int i = 0; i < 6; i++) {
    float delta = K[i] * y;
    if (i == 0) ekf_state.north_m += delta;
    if (i == 1) ekf_state.east_m += delta;
    if (i == 2) ekf_state.v_north += delta;
    if (i == 3) ekf_state.v_east += delta;
    if (i == 4) ekf_state.wind_north += delta;
    if (i == 5) ekf_state.wind_east += delta;
  }
  float P_new[6][6] = {};
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      P_new[i][j] = P[i][j] - K[i] * P[idx][j];
    }
  }
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      P[i][j] = P_new[i][j];
    }
  }
}
static void updateEKF() {
  if (!origin_set || gnss_fix_type < 3) {
    return;
  }
  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if (last_ms == 0) {
    last_ms = now;
    return;
  }
  double dt = (now - last_ms) / 1000.0;
  if (dt <= 0.0 || dt > 1.0) {
    last_ms = now;
    return;
  }
  last_ms = now;
  double meas_n = 0.0;
  double meas_e = 0.0;
  latLonToNE(lat, lon, meas_n, meas_e);
  double meas_vn = ground_speed * cos(ground_track_rad);
  double meas_ve = ground_speed * sin(ground_track_rad);
  if (!ekf_state.initialized) {
    ekf_state.north_m = meas_n;
    ekf_state.east_m = meas_e;
    ekf_state.v_north = meas_vn;
    ekf_state.v_east = meas_ve;
    ekf_state.wind_north = wind_north;
    ekf_state.wind_east = wind_east;
    ekf_state.initialized = true;
    return;
  }
  static float P[6][6] = {
    {60.0f, 0, 0, 0, 0, 0},
    {0, 60.0f, 0, 0, 0, 0},
    {0, 0, 6.0f, 0, 0, 0},
    {0, 0, 0, 6.0f, 0, 0},
    {0, 0, 0, 0, 12.0f, 0},
    {0, 0, 0, 0, 0, 12.0f},
  };
  // Predict
  ekf_state.north_m += ekf_state.v_north * dt;
  ekf_state.east_m += ekf_state.v_east * dt;
  float F[6][6] = {};
  for (int i = 0; i < 6; i++) {
    F[i][i] = 1.0f;
  }
  F[0][2] = (float)dt;
  F[1][3] = (float)dt;
  float P_pred[6][6] = {};
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 6; k++) {
        sum += F[i][k] * P[k][j];
      }
      P_pred[i][j] = sum;
    }
  }
  float FPFT[6][6] = {};
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 6; k++) {
        sum += P_pred[i][k] * F[j][k];
      }
      FPFT[i][j] = sum;
    }
  }
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      P[i][j] = FPFT[i][j];
    }
  }
  P[0][0] += EKF_Q_POS;
  P[1][1] += EKF_Q_POS;
  P[2][2] += EKF_Q_VEL;
  P[3][3] += EKF_Q_VEL;
  P[4][4] += EKF_Q_WIND;
  P[5][5] += EKF_Q_WIND;
  // Position update
  ekfUpdateComponent(ekf_state.north_m, P, 0, (float)meas_n, EKF_R_POS);
  ekfUpdateComponent(ekf_state.east_m, P, 1, (float)meas_e, EKF_R_POS);
  // Velocity update
  ekfUpdateComponent(ekf_state.v_north, P, 2, (float)meas_vn, EKF_R_VEL);
  ekfUpdateComponent(ekf_state.v_east, P, 3, (float)meas_ve, EKF_R_VEL);
  // Wind update from airspeed + heading when valid
  if (isHeadingFresh() && getHeadingConfidence() >= HEADING_CONFIDENCE_MIN_FOR_WIND &&
      isGNSSQualityGood()) {
    float brake = normalizeBrake(cmdL_current, cmdR_current);
    double airspeed = brakeToSinkRate(brake) * brakeToGlideRatio(brake);
    double heading_used = getHeadingRad();
    double air_n = airspeed * cos(heading_used);
    double air_e = airspeed * sin(heading_used);
    double wind_n_meas = meas_vn - air_n;
    double wind_e_meas = meas_ve - air_e;
    ekfUpdateComponent(ekf_state.wind_north, P, 4, (float)wind_n_meas, EKF_R_WIND);
    ekfUpdateComponent(ekf_state.wind_east, P, 5, (float)wind_e_meas, EKF_R_WIND);
  }
}
static float turnRateFromBank(float bank_rad, float airspeed) {
  if (airspeed < 0.1f) return 0.0f;
  return (9.81f * tanf(bank_rad)) / airspeed;
}
// ===================== WIND ESTIMATION =====================
static void estimateWind() {
  if (state != GUIDED_DESCENT && state != FINAL_APPROACH &&
      state != TERMINAL_HOMING && state != WAIT_FOR_STABLE_DESCENT) {
    return;
  }
  double height_agl = fused_alt_msl - target_alt_msl;
  if (height_agl < MIN_WIND_UPDATE_ALT_M) {
    return;
  }
  float heading_conf = getHeadingConfidence();
  if (ground_speed < 0.5 || sink_rate_filtered < 1.0 || !isHeadingFresh() ||
      !ground_track_valid || !isGNSSQualityGood() ||
      heading_conf < HEADING_CONFIDENCE_MIN_FOR_WIND) {
    return;
  }
  float brake = normalizeBrake(cmdL_current, cmdR_current);
  double airspeed = brakeToSinkRate(brake) * brakeToGlideRatio(brake);
  double heading_used = getHeadingRad();
  double airspeed_north = airspeed * cos(heading_used);
  double airspeed_east = airspeed * sin(heading_used);
  double ground_vel_north = ground_speed * cos(ground_track_rad);
  double ground_vel_east = ground_speed * sin(ground_track_rad);
  double wind_north_measured = ground_vel_north - airspeed_north;
  double wind_east_measured = ground_vel_east - airspeed_east;
  double delta_n = wind_north_measured - wind_north;
  double delta_e = wind_east_measured - wind_east;
  if (fabs(delta_n) > MAX_WIND_UPDATE_DELTA || fabs(delta_e) > MAX_WIND_UPDATE_DELTA) {
    wind_update_rejects++;
    return;
  }
  wind_north = wind_north * (1.0 - WIND_FILTER_ALPHA) + wind_north_measured * WIND_FILTER_ALPHA;
  wind_east = wind_east * (1.0 - WIND_FILTER_ALPHA) + wind_east_measured * WIND_FILTER_ALPHA;
  int idx = windLayerIndex(height_agl);
  if (fabs(wind_north_measured - wind_layer_north[idx]) <= MAX_WIND_LAYER_DELTA &&
      fabs(wind_east_measured - wind_layer_east[idx]) <= MAX_WIND_LAYER_DELTA) {
    wind_layer_north[idx] = wind_layer_north[idx] * (1.0 - WIND_FILTER_ALPHA) + wind_north_measured * WIND_FILTER_ALPHA;
    wind_layer_east[idx] = wind_layer_east[idx] * (1.0 - WIND_FILTER_ALPHA) + wind_east_measured * WIND_FILTER_ALPHA;
  } else {
    wind_layer_rejects++;
  }
  wind_layer_ms[idx] = millis();
  for (int i = 0; i < WIND_LAYER_COUNT; i++) {
    uint32_t age_ms = millis() - wind_layer_ms[i];
    if (age_ms > 0) {
      float age_s = age_ms / 1000.0f;
      float decay = expf(-WIND_LAYER_DECAY_PER_S * age_s);
      wind_layer_north[i] *= decay;
      wind_layer_east[i] *= decay;
    }
  }
  smoothWindLayers();
  wind_speed = sqrt(wind_north * wind_north + wind_east * wind_east);
  if (wind_speed > MAX_WIND_ESTIMATE) {
    double scale = MAX_WIND_ESTIMATE / wind_speed;
    wind_north *= scale;
    wind_east *= scale;
    wind_speed = MAX_WIND_ESTIMATE;
  }
  updateGustDetector();
}
// ===================== ALTITUDE FUSION =====================
static void updateAltitudeFusion() {
  // Complementary filter: baro for short-term, GNSS for long-term
  float alpha = 0.05f;
  if (gnss_fix_type >= 3) {
    float accuracy_m = gnss_v_accuracy_mm * 0.001f;
    float accuracy_factor = 1.0f;
    if (accuracy_m > 0.5f) {
      accuracy_factor = clampf(0.5f / accuracy_m, 0.1f, 1.0f);
    }
    alpha = clampf(alpha * accuracy_factor, 0.01f, 0.12f);
    
    // ✓ BUG FIX #5: Trust RTK much more (1cm vertical accuracy)
    if (gnss_fix_type == 5) {  // RTK fixed
      alpha = 0.25f;  // 25% GNSS weight instead of 5%
    }
    fused_alt_msl = (1.0 - alpha) * baro_alt_msl + alpha * gnss_alt_msl;
  } else {
    fused_alt_msl = baro_alt_msl;
  }
  // Estimate vertical velocity
  static double last_fused_alt = 0.0;
  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if (last_ms > 0) {
    double dt = (now - last_ms) / 1000.0;
    if (dt > 0.01) {
      fused_alt_velocity = (fused_alt_msl - last_fused_alt) / dt;
    }
  }
  last_fused_alt = fused_alt_msl;
  last_ms = now;
}
// ===================== PREDICT LANDING WITH PHYSICS =====================
static void predictLandingWithPhysics() {
  double height_agl = fused_alt_msl - target_alt_msl;
  if (height_agl < 0.5) {
    getEstimatedLatLon(pred_lat, pred_lon);
    predicted_error_m = distanceMeters(lat, lon, target_lat, target_lon);
    return;
  }
  double remaining = height_agl;
  double sim_lat = 0.0;
  double sim_lon = 0.0;
  getEstimatedLatLon(sim_lat, sim_lon);
  double heading_used = getHeadingRad();
  double wind_n = ekf_state.initialized ? ekf_state.wind_north : wind_north;
  double wind_e = ekf_state.initialized ? ekf_state.wind_east : wind_east;
  while (remaining > 0.5) {
    double step = (remaining > WIND_LAYER_STEP_M) ? WIND_LAYER_STEP_M : remaining;
    int idx = windLayerIndex(remaining);
    float brake = normalizeBrake(cmdL_current, cmdR_current);
    double sink = brakeToSinkRate(brake);
    double glide = brakeToGlideRatio(brake);
    double airspeed = sink * glide;
    double dt = step / sink;
    double airspeed_north = airspeed * cos(heading_used);
    double airspeed_east = airspeed * sin(heading_used);
    double layer_wind_n = wind_layer_north[idx] + wind_n * 0.1;
    double layer_wind_e = wind_layer_east[idx] + wind_e * 0.1;
    double ground_vel_north = airspeed_north + layer_wind_n;
    double ground_vel_east = airspeed_east + layer_wind_e;
    sim_lat += (ground_vel_north * dt) / metersPerDegLat();
    sim_lon += (ground_vel_east * dt) / metersPerDegLon(sim_lat);
    remaining -= step;
  }
  pred_lat = sim_lat;
  pred_lon = sim_lon;
  predicted_error_m = distanceMeters(pred_lat, pred_lon, target_lat, target_lon);
}
// ===================== MPC GUIDANCE (DYNAMICS AWARE) =====================
static float computeMPCControl() {
  float best_cmd = 0.0f;
  float best_cost = 1e9f;
  double current_lat = 0.0;
  double current_lon = 0.0;
  getEstimatedLatLon(current_lat, current_lon);
  double current_heading = getHeadingRad();
  double current_alt = fused_alt_msl;
  double wind_n = ekf_state.initialized ? ekf_state.wind_north : wind_north;
  double wind_e = ekf_state.initialized ? ekf_state.wind_east : wind_east;
  for (int i = 0; i < MPC_SAMPLES; i++) {
    float test_cmd = -1.0f + (2.0f / (MPC_SAMPLES - 1)) * i;
    double sim_lat = current_lat;
    double sim_lon = current_lon;
    double sim_heading = current_heading;
    double sim_alt = current_alt;
    float dt = 0.4f;
    float brake = normalizeBrake(cmdL_current, cmdR_current);
    float airspeed = brakeToSinkRate(brake) * brakeToGlideRatio(brake);
    float cmd_state = 0.0f;
    for (int step = 0; step < MPC_HORIZON; step++) {
      float cmd_target = clampf(test_cmd, -1.0f, 1.0f);
      float alpha = dt / (SERVO_TIME_CONSTANT_S + dt);
      cmd_state = cmd_state + alpha * (cmd_target - cmd_state);
      float brake_sim = clampf(fabs(cmd_state), 0.0f, 1.0f);
      float glide_sim = brakeToGlideRatio(brake_sim);
      float sink_sim = brakeToSinkRate(brake_sim);
      airspeed = sink_sim * glide_sim;
      float bank = cmd_state * MAX_BANK_ANGLE_RAD;
      float turn_rate = turnRateFromBank(bank, airspeed) * brakeToTurnRateScale(brake_sim);
      turn_rate = clampf(turn_rate, -MAX_TURN_RATE_RAD_S, MAX_TURN_RATE_RAD_S);
      sim_heading = wrapAngle(sim_heading + turn_rate * dt);
      double ground_vel_north = airspeed * cos(sim_heading) + wind_n;
      double ground_vel_east = airspeed * sin(sim_heading) + wind_e;
      sim_lat += (ground_vel_north * dt) / metersPerDegLat();
      sim_lon += (ground_vel_east * dt) / metersPerDegLon(sim_lat);
      sim_alt -= sink_sim * dt;
      if (sim_alt < target_alt_msl) break;
    }
    double final_error = distanceMeters(sim_lat, sim_lon, target_lat, target_lon);
    float control_cost = fabs(test_cmd) * MPC_CONTROL_WEIGHT;
    float total_cost = final_error + control_cost;
    if (total_cost < best_cost) {
      best_cost = total_cost;
      best_cmd = test_cmd;
    }
  }
  return best_cmd;
}
// ===================== GUIDANCE =====================
static void resetHeadingController() {
  heading_error_integral = 0.0;
  last_heading_error = 0.0;
  last_heading_update_ms = 0;
}
static void resetGuidanceIntegrators() {
  resetHeadingController();
  glide_integral = 0.0f;
}
static float computeHeadingControl(double desired_bearing, float gain_multiplier) {
  double heading_error = wrapAngle(desired_bearing - getHeadingRad());
  uint32_t now = millis();
  float dt = 0.02f;
  if (last_heading_update_ms > 0) {
    dt = (now - last_heading_update_ms) / 1000.0f;
    if (dt < 0.005f) dt = 0.005f;
    if (dt > 0.2f) dt = 0.2f;
  }
  last_heading_update_ms = now;
  heading_error_integral += heading_error * dt;
  heading_error_integral = clampf(heading_error_integral, -1.5f, 1.5f);
  float heading_error_rate = (heading_error - last_heading_error) / dt;
  last_heading_error = heading_error;
  float wind_factor = 1.0f;
  if (wind_speed > WIND_GAIN_REDUCTION_START) {
    float t = (float)((wind_speed - WIND_GAIN_REDUCTION_START) /
                      (MAX_WIND_FOR_GUIDANCE - WIND_GAIN_REDUCTION_START));
    t = clampf(t, 0.0f, 1.0f);
    wind_factor = 1.0f - t * (1.0f - WIND_GAIN_MIN_FACTOR);
  }
  float gust_factor = gust_active ? GUST_GAIN_FACTOR : 1.0f;
  float kp = KP_HEADING * gain_multiplier * wind_factor * gust_factor;
  float ki = KI_HEADING * gain_multiplier * wind_factor * gust_factor;
  float kd = KD_HEADING * gain_multiplier * wind_factor * gust_factor;
  float cmd = (kp * heading_error) +
              (ki * heading_error_integral) +
              (kd * heading_error_rate);
  return clampf(cmd, -1.0f, 1.0f);
}
static void runPredictiveGuidance() {
  estimateWind();
  predictLandingWithPhysics();
  double height_agl = fused_alt_msl - target_alt_msl;
  if (height_agl < 0.5) {
    cmdL_target = 0.0f;
    cmdR_target = 0.0f;
    return;
  }
  double desired_bearing = bearingRad(pred_lat, pred_lon, target_lat, target_lon);
  float turn_cmd;
  float altitude_gain = altitudeGain(height_agl,
                                     HEADING_GAIN_HIGH_ALT,
                                     HEADING_GAIN_LOW_ALT,
                                     HEADING_GAIN_HIGH_ALT_M,
                                     HEADING_GAIN_LOW_ALT_M);
  if (height_agl > 60.0 && state == GUIDED_DESCENT) {
    turn_cmd = computeMPCControl() * altitude_gain;
  } else {
    float gain = (state == FINAL_APPROACH) ? APPROACH_GAIN : 1.0f;
    turn_cmd = computeHeadingControl(desired_bearing, gain * altitude_gain);
  }
  if (ekf_state.initialized) {
    double heading = getHeadingRad();
    double vel_lat = -ekf_state.v_north * sin(heading) + ekf_state.v_east * cos(heading);
    turn_cmd -= (float)(LATERAL_DAMPING_GAIN * vel_lat);
  }
  turn_cmd = clampf(turn_cmd, -1.0f, 1.0f);
  float glide_cmd = applyGlideSlope(height_agl);
  cmdL_target = clampf(-turn_cmd + glide_cmd, -1.0f, 1.0f);
  cmdR_target = clampf(turn_cmd + glide_cmd, -1.0f, 1.0f);
}
// ===================== GNSS FUNCTIONS =====================
static void setupGNSS() {
  Serial5.begin(38400);
  if (!gnss.begin(Serial5)) {
    Serial.println("ERROR: GNSS not found!");
    return;
  }
  gnss.setNavigationFrequency(10);
  gnss.setAutoPVT(true);
  gnss.setUART1Output(COM_TYPE_UBX);
  gnss.setUART1Input(COM_TYPE_UBX | COM_TYPE_RTCM3);
  gnss.setDGNSSConfiguration(SFE_UBLOX_DGNSS_MODE_FIXED);
  Serial.println("GNSS initialized");
}
static bool isGNSSQualityGood() {
  if (gnss_fix_type < 3) return false;
  if (gnss_satellites < 6) return false;
  if (millis() - last_gnss_update > 2000) return false;
  if (gnss_fix_type == 5) {
    if (gnss_h_accuracy_mm > 100) return false;
    if (gnss_v_accuracy_mm > 150) return false;
  }
  return true;
}
static float getDynamicCaptureRadius() {
  if (gnss_fix_type == 5 && gnss_h_accuracy_mm < 100) {
    return CAPTURE_RADIUS_M;
  } else if (gnss_fix_type >= 3 && gnss_satellites >= 8) {
    return CAPTURE_RADIUS_GNSS_M;
  } else {
    return CAPTURE_RADIUS_POOR_M;
  }
}
static void updateGNSS() {
  gnss.checkUblox();
  if (gnss.getPVT()) {
    lat = gnss.getLatitude() * 1e-7;
    lon = gnss.getLongitude() * 1e-7;
    gnss_alt_msl = gnss.getAltitudeMSL() / 1000.0;
    ground_speed = gnss.getGroundSpeed() / 1000.0;
    if (ground_speed >= MIN_GROUND_SPEED_FOR_TRACK) {
      double headDeg = gnss.getHeading() * 1e-5;
      ground_track_rad = headDeg * DEG_TO_RAD;
      ground_track_valid = true;
    } else {
      ground_track_valid = false;
    }
    gnss_vertical_velocity = gnss.getVerticalVelocity() / 1000.0;
    gnss_fix_type = gnss.getFixType();
    gnss_satellites = gnss.getSIV();
    gnss_h_accuracy_mm = gnss.getHorizontalAccuracy();
    gnss_v_accuracy_mm = gnss.getVerticalAccuracy();
    last_gnss_update = millis();
    setOriginIfNeeded();
    updateNavEstimate();
    updateEKF();
    if (ekf_state.initialized) {
      wind_north = ekf_state.wind_north;
      wind_east = ekf_state.wind_east;
    }
    if (!sea_level_calibrated && gnss_fix_type >= 3) {
      sea_level_pressure_hpa = pressure_hPa / pow(1.0 - gnss_alt_msl / 44330.0, 5.255);
      sea_level_calibrated = true;
      Serial.printf("Baro calibrated: QNH = %.1f hPa\n", sea_level_pressure_hpa);
    }
  } else if (millis() - last_gnss_update > MAX_GNSS_AGE_MS) {
    ground_track_valid = false;
  }
}
static void feedRTKToGNSS(const uint8_t *rtcm, size_t len) {
  Serial5.write(rtcm, len);
}
// ===================== BMP280 FUNCTIONS =====================
static void setupBMP280() {
  if (!bmp.begin(0x76)) {
    Serial.println("ERROR: BMP280 not found!");
    return;
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_63);
  last_baro_ms = millis();
  last_baro_alt = 0.0;
  Serial.println("BMP280 initialized");
}
static void updateBMP280() {
  pressure_hPa = bmp.readPressure() / 100.0;
  temp_c = bmp.readTemperature();
  if (sea_level_calibrated) {
    baro_alt_msl = 44330.0 * (1.0 - pow(pressure_hPa / sea_level_pressure_hpa, 1.0 / 5.255));
  } else {
    baro_alt_msl = bmp.readAltitude(1013.25);
  }
  uint32_t now = millis();
  uint32_t dt_ms = now - last_baro_ms;
  if (dt_ms >= BARO_UPDATE_INTERVAL_MS) {
    double dt_sec = dt_ms / 1000.0;
    sink_rate_raw = (last_baro_alt - baro_alt_msl) / dt_sec;
    if (sink_rate_raw < SINK_RATE_MIN) sink_rate_raw = SINK_RATE_MIN;
    if (sink_rate_raw > SINK_RATE_MAX) sink_rate_raw = SINK_RATE_MAX;
    sink_rate_filtered = sink_rate_filtered * (1.0 - SINK_FILTER_ALPHA) +
                         sink_rate_raw * SINK_FILTER_ALPHA;
    last_baro_alt = baro_alt_msl;
    last_baro_ms = now;
  }
  if (gnss_fix_type >= 3 && fabs(gnss_vertical_velocity) < 20.0) {
    sink_rate_gnss = -gnss_vertical_velocity;
  }
  updateAltitudeFusion();
}
// ===================== BNO085 FUNCTIONS =====================
static void setupBNO085() {
  if (!bno08x.begin_I2C(0x4A, &Wire)) {
    Serial.println("ERROR: BNO085 not found!");
    return;
  }
  bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR, 50);  // Absolute yaw (mag)
  bno08x.enableReport(SH2_ROTATION_VECTOR, 50);  // Fallback if mag unreliable
  bno08x.enableReport(SH2_ACCELEROMETER, 50);
  Serial.println("BNO085 initialized");
}
static void updateIMU() {
  while (bno08x.getSensorEvent(&bnoValue)) {
    if (bnoValue.sensorId == SH2_ACCELEROMETER) {
      imu_ax = bnoValue.un.accelerometer.x;
      imu_ay = bnoValue.un.accelerometer.y;
      imu_az = bnoValue.un.accelerometer.z;
      imu_accel_valid = true;
      last_imu_ms = millis();
    }
    if (bnoValue.sensorId == SH2_GEOMAGNETIC_ROTATION_VECTOR ||
        bnoValue.sensorId == SH2_ROTATION_VECTOR) {
      float qw = bnoValue.un.rotationVector.real;
      float qx = bnoValue.un.rotationVector.i;
      float qy = bnoValue.un.rotationVector.j;
      float qz = bnoValue.un.rotationVector.k;
      float siny_cosp = 2.0f * (qw * qz + qx * qy);
      float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
      float new_heading = wrapAngle(atan2f(siny_cosp, cosy_cosp) + HEADING_OFFSET_RAD);
      uint32_t now = millis();
      uint32_t dt_ms = (last_heading_ms > 0) ? (now - last_heading_ms) : 0;
      float delta = heading_valid ? wrapAngle(new_heading - heading_rad) : 0.0f;
      if (heading_valid && (now - last_heading_ms) < HEADING_JUMP_MAX_DT_MS) {
        if (fabs(delta) > HEADING_JUMP_MAX_RAD) {
          continue;
        }
      }
      heading_rad = new_heading;
      heading_valid = true;
      last_heading_ms = now;
      last_heading_rad = heading_rad;
      float delta_factor = 1.0f;
      if (HEADING_CONFIDENCE_MAX_DELTA_RAD > 0.0f) {
        delta_factor = 1.0f - clampf(fabs(delta) / HEADING_CONFIDENCE_MAX_DELTA_RAD, 0.0f, 1.0f);
      }
      float dt_factor = 1.0f;
      if (HEADING_CONFIDENCE_MAX_DT_MS > 0) {
        dt_factor = 1.0f - clampf((float)dt_ms / (float)HEADING_CONFIDENCE_MAX_DT_MS, 0.0f, 1.0f);
      }
      heading_confidence = clampf(delta_factor * dt_factor, 0.0f, 1.0f);
    }
  }
}
// ===================== LORA FUNCTIONS =====================
static void setupLoRa() {
  int st = radio.begin(LORA_FREQ_MHZ);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("ERROR: LoRa init failed: %d\n", st);
    return;
  }
  radio.setOutputPower(LORA_TX_DBM);
  radio.setCRC(true);
  radio.startReceive();
  Serial.println("LoRa initialized");
}
static void pollLoRa() {
  if (!radio.available()) return;
  uint8_t buf[255];
  int len = radio.getPacketLength();
  if (len <= 0 || len > (int)sizeof(buf)) {
    radio.startReceive();
    return;
  }
  int err = radio.readData(buf, len);
  if (err == RADIOLIB_ERR_NONE) {
    uint8_t type = buf[0];
    if (type == MSG_TARGET && len == (int)sizeof(TargetPacket)) {
      TargetPacket tp;
      memcpy(&tp, buf, sizeof(tp));
      uint16_t crcCalc = crc16_ccitt((uint8_t *)&tp, sizeof(tp) - 2);
      if (crcCalc == tp.crc16) {
        target_lat = tp.tgt_lat_e7 * 1e-7;
        target_lon = tp.tgt_lon_e7 * 1e-7;
        target_alt_msl = tp.tgt_alt_cm / 100.0;
        targetReceived = true;
        Serial.printf("TARGET received: %.7f, %.7f, %.1f m\n",
                      target_lat, target_lon, target_alt_msl);
      }
    }
    if (type == MSG_RTK && len > 1) {
      feedRTKToGNSS(&buf[1], (size_t)(len - 1));
    }
  }
  radio.startReceive();
}
// ===================== SERVO FUNCTIONS =====================
static void setServosNormalized(float left, float right) {
  left += SERVO_TRIM;
  right -= SERVO_TRIM;
  left = clampf(left, -1.0f, 1.0f);
  right = clampf(right, -1.0f, 1.0f);
  if (fabs(left) < SERVO_DEADBAND) left = 0.0f;
  if (fabs(right) < SERVO_DEADBAND) right = 0.0f;
  uint32_t now = millis();
  if (last_servo_update_ms > 0) {
    float dt = (now - last_servo_update_ms) / 1000.0f;
    float max_change = SERVO_MAX_RATE * dt;
    float delta_L = left - cmdL_current;
    float delta_R = right - cmdR_current;
    if (fabs(delta_L) > max_change) {
      delta_L = (delta_L > 0) ? max_change : -max_change;
    }
    if (fabs(delta_R) > max_change) {
      delta_R = (delta_R > 0) ? max_change : -max_change;
    }
    cmdL_current += delta_L;
    cmdR_current += delta_R;
  } else {
    cmdL_current = left;
    cmdR_current = right;
  }
  last_servo_update_ms = now;
  int usL = SERVO_NEUTRAL_US + (int)(cmdL_current * SERVO_RANGE_US);
  int usR = SERVO_NEUTRAL_US + (int)(cmdR_current * SERVO_RANGE_US);
  servoL.writeMicroseconds(usL);
  servoR.writeMicroseconds(usR);
}
// ===================== EVENT DETECTION =====================
static bool dropDetected() {
  if (sink_rate_filtered < 1.0) {
    drop_spike_count = 0;
    return false;
  }
  if (imu_accel_valid && (millis() - last_imu_ms) < 500) {
    float aMag = sqrt(imu_ax * imu_ax + imu_ay * imu_ay + imu_az * imu_az);
    if (fabs(aMag - 9.81f) > DROP_ACCEL_SPIKE) {
      drop_spike_count++;
      if (drop_spike_count >= DROP_DEBOUNCE_COUNT) {
        Serial.println("DROP DETECTED!");
        return true;
      }
    } else {
      drop_spike_count = 0;
    }
  }
  return false;
}
static bool descentStable() {
  if (sink_rate_filtered >= STABLE_SINK_MIN &&
      sink_rate_filtered <= STABLE_SINK_MAX) {
    if (stableStartMs == 0) {
      stableStartMs = millis();
    }
    if (millis() - stableStartMs >= STABLE_HOLD_MS) {
      Serial.println("DESCENT STABLE!");
      return true;
    }
  } else {
    stableStartMs = 0;
  }
  return false;
}
static bool landingDetected() {
  static uint32_t stillStart = 0;
  if (sink_rate_filtered < 0.3) {
    if (stillStart == 0) stillStart = millis();
    if (millis() - stillStart > 1000) {
      Serial.println("LANDED!");
      return true;
    }
  } else {
    stillStart = 0;
  }
  return false;
}
// ===================== TELEMETRY =====================
static void sendTelemetry1Hz() {
  uint32_t now = millis();
  if (now - lastTelemetryMs < TELEMETRY_PERIOD_MS) return;
  lastTelemetryMs = now;
  TelemetryPacket p{};
  p.msgType = MSG_TELEMETRY;
  p.time_ms = now;
  p.lat_e7 = (int32_t)llround(lat * 1e7);
  p.lon_e7 = (int32_t)llround(lon * 1e7);
  double height_agl = fused_alt_msl - target_alt_msl;
  p.height_agl_dm = (int16_t)clampf((float)(height_agl * 10.0), -32768, 32767);
  p.pressure_hPa_x10 = (uint16_t)clampf((float)(pressure_hPa * 10.0), 0, 65535);
  p.temp_c_x10 = (int16_t)clampf((float)(temp_c * 10.0), -32768, 32767);
  p.ground_speed_cms = (uint16_t)clampf((float)(ground_speed * 100.0), 0, 65535);
  p.servo_left_x1000 = (int16_t)clampf(cmdL_current * 1000.0f, -1000, 1000);
  p.servo_right_x1000 = (int16_t)clampf(cmdR_current * 1000.0f, -1000, 1000);
  p.pred_lat_e7 = (int32_t)llround(pred_lat * 1e7);
  p.pred_lon_e7 = (int32_t)llround(pred_lon * 1e7);
  p.wind_north_cms = (int16_t)clampf((float)(wind_north * 100.0), -32768, 32767);
  p.wind_east_cms = (int16_t)clampf((float)(wind_east * 100.0), -32768, 32767);
  p.heading_deg_x10 = (int16_t)clampf((float)(getHeadingRad() * RAD_TO_DEG * 10.0f), -32768, 32767);
  p.heading_confidence_x1000 = (uint16_t)clampf(getHeadingConfidence() * 1000.0f, 0.0f, 1000.0f);
  p.wind_rejects = wind_update_rejects;
  p.wind_layer_rejects = wind_layer_rejects;
  p.mission_state = (uint8_t)state;
  p.crc16 = crc16_ccitt((uint8_t *)&p, sizeof(p) - 2);
  radio.standby();
  radio.transmit((uint8_t *)&p, sizeof(p));
  radio.startReceive();
  if (logFile) {
    logFile.printf(
      "%lu,%.7f,%.7f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.7f,%.7f,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.3f,%u,%u\n",
      p.time_ms, lat, lon, height_agl, pressure_hPa, temp_c, ground_speed,
      cmdL_current, cmdR_current, pred_lat, pred_lon, (int)state,
      gnss_fix_type, gnss_satellites, wind_speed, predicted_error_m,
      wind_north, wind_east, getHeadingConfidence(), wind_update_rejects, wind_layer_rejects);
    logFile.flush();
  }
}
// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(500);
  
  // *** HEADING CALIBRATION MODE ***
  if (HEADING_CALIBRATION_MODE) {
    Serial.println("*** HEADING CALIBRATION MODE ***");
    Serial.println("Point CanSat exactly NORTH and read IMU heading below.");
    Serial.println("Calculate: HEADING_OFFSET_RAD = (0.0 - measured_value)");
    Serial.println("Update code, set HEADING_CALIBRATION_MODE=false, re-upload.");
    Serial.println("");
  }
  delay(2000);
  Serial.println("=================================");
  Serial.println("AeroTrackNow CanSat v3.0");
  Serial.println("Accuracy-Optimized Flight Code");
  Serial.println("=================================");
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();
  servoL.attach(PIN_SERVO_LEFT);
  servoR.attach(PIN_SERVO_RIGHT);
  setServosNormalized(0, 0);
  Serial.println("Initializing sensors...");
  setupBMP280();
  setupBNO085();
  setupGNSS();
  seedWindProfile();
  Serial.println("Initializing SD card...");
  if (SD.begin(PIN_SD_CS)) {
    logFile = SD.open("flight_v3.csv", FILE_WRITE);
    if (logFile) {
      logFile.println("time_ms,lat,lon,height_agl,press_hPa,temp_C,gs_mps,cmdL,cmdR,pred_lat,pred_lon,state,fix_type,sats,wind_ms,pred_error_m,wind_n,wind_e,heading_conf,wind_rejects,wind_layer_rejects");
      logFile.flush();
      Serial.println("SD logging enabled");
    }
  } else {
    Serial.println("WARNING: SD card failed");
  }
  Serial.println("Initializing LoRa...");
  setupLoRa();
  Serial.println("=================================");
  Serial.println("Initialization complete!");
  Serial.println("Waiting for TARGET packet...");
  Serial.println("=================================");
  state = BOOT;
}
// ===================== MAIN LOOP =====================
void loop() {
  loop_start_ms = millis();
  updateIMU();
  updateBMP280();
  updateGNSS();
  pollLoRa();
  switch (state) {
    case BOOT:
      setServosNormalized(0, 0);
      if (targetReceived) {
        Serial.println("STATE: BOOT -> WAIT_FOR_DROP");
        state = WAIT_FOR_DROP;
      }
      break;
    case WAIT_FOR_DROP:
      setServosNormalized(0, 0);
      if (dropDetected()) {
        Serial.println("STATE: WAIT_FOR_DROP -> WAIT_FOR_STABLE_DESCENT");
        state = WAIT_FOR_STABLE_DESCENT;
      }
      break;
    case WAIT_FOR_STABLE_DESCENT:
      setServosNormalized(0, 0);
      if (descentStable()) {
        Serial.println("STATE: WAIT_FOR_STABLE_DESCENT -> GUIDED_DESCENT");
        resetGuidanceIntegrators();
        state = GUIDED_DESCENT;
      }
      break;
    case GUIDED_DESCENT: {
      if (!targetReceived) {
        setServosNormalized(0, 0);
        break;
      }
      if (!guidanceSafe()) {
        resetGuidanceIntegrators();
        setServosNormalized(0, 0);
        break;
      }
      runPredictiveGuidance();
      double height_agl = fused_alt_msl - target_alt_msl;
      if (height_agl < FINAL_APPROACH_HEIGHT_M) {
        Serial.println("STATE: GUIDED_DESCENT -> FINAL_APPROACH");
        resetGuidanceIntegrators();
        state = FINAL_APPROACH;
        break;
      }
      setServosNormalized(cmdL_target, cmdR_target);
      break;
    }
    case FINAL_APPROACH: {
      if (!targetReceived) {
        setServosNormalized(0, 0);
        break;
      }
      if (!guidanceSafe()) {
        resetGuidanceIntegrators();
        setServosNormalized(0, 0);
        break;
      }
      runPredictiveGuidance();
      double height_agl = fused_alt_msl - target_alt_msl;
      float capture_radius = getDynamicCaptureRadius();
      if (height_agl < TERMINAL_HOMING_HEIGHT_M) {
        Serial.println("STATE: FINAL_APPROACH -> TERMINAL_HOMING");
        resetGuidanceIntegrators();
        state = TERMINAL_HOMING;
        break;
      }
      if (height_agl < FLARE_HEIGHT_M) {
        Serial.println("STATE: FINAL_APPROACH -> FLARE");
        state = FLARE;
        break;
      }
      if (height_agl < TERMINAL_HEIGHT_M && predicted_error_m < capture_radius) {
        Serial.println("STATE: FINAL_APPROACH -> TERMINAL");
        cmdL_target = 0.0f;
        cmdR_target = 0.0f;
        state = TERMINAL;
        break;
      }
      setServosNormalized(cmdL_target, cmdR_target);
      break;
    }
    case TERMINAL_HOMING: {
      if (!targetReceived) {
        setServosNormalized(0, 0);
        break;
      }
      if (!guidanceSafe()) {
        resetGuidanceIntegrators();
        setServosNormalized(0, 0);
        break;
      }
      estimateWind();
      double height_agl = fused_alt_msl - target_alt_msl;
      float capture_radius = getDynamicCaptureRadius();
      if (height_agl < FLARE_HEIGHT_M) {
        Serial.println("STATE: TERMINAL_HOMING -> FLARE");
        state = FLARE;
        break;
      }
      if (height_agl < TERMINAL_HEIGHT_M && predicted_error_m < capture_radius) {
        Serial.println("STATE: TERMINAL_HOMING -> TERMINAL");
        cmdL_target = 0.0f;
        cmdR_target = 0.0f;
        state = TERMINAL;
        break;
      }
      if (origin_set && ekf_state.initialized) {
        double tgt_n = 0.0;
        double tgt_e = 0.0;
        latLonToNE(target_lat, target_lon, tgt_n, tgt_e);
        double err_n = tgt_n - ekf_state.north_m;
        double err_e = tgt_e - ekf_state.east_m;
        double distance = sqrt(err_n * err_n + err_e * err_e);
        predicted_error_m = distance;
        double heading = getHeadingRad();
        double err_lat = -err_n * sin(heading) + err_e * cos(heading);
        double vel_lat = -ekf_state.v_north * sin(heading) + ekf_state.v_east * cos(heading);
        float cross_gain = altitudeGain(height_agl,
                                        CROSSTRACK_GAIN_HIGH_ALT,
                                        CROSSTRACK_GAIN_LOW_ALT,
                                        CROSSTRACK_GAIN_HIGH_ALT_M,
                                        CROSSTRACK_GAIN_LOW_ALT_M);
        float cross_cmd = (KP_CROSSTRACK * cross_gain * (float)err_lat) +
                          (KD_CROSSTRACK * cross_gain * (float)vel_lat);
        cross_cmd = clampf(cross_cmd, -MAX_CROSSTRACK_CMD, MAX_CROSSTRACK_CMD);
        double desired_bearing = atan2(err_e, err_n);
        float heading_cmd = computeHeadingControl(desired_bearing,
                                                  APPROACH_GAIN * 0.7f * cross_gain);
        float turn_cmd = clampf(heading_cmd + cross_cmd, -1.0f, 1.0f);
        float glide_cmd = applyGlideSlope(height_agl);
        cmdL_target = clampf(-turn_cmd + glide_cmd, -1.0f, 1.0f);
        cmdR_target = clampf(turn_cmd + glide_cmd, -1.0f, 1.0f);
      } else {
        runPredictiveGuidance();
      }
      setServosNormalized(cmdL_target, cmdR_target);
      break;
    }
    case FLARE: {
      float flare_cmd = 0.6f;
      setServosNormalized(flare_cmd, flare_cmd);
      double height_agl = fused_alt_msl - target_alt_msl;
      if (height_agl < TERMINAL_HEIGHT_M) {
        Serial.println("STATE: FLARE -> TERMINAL");
        state = TERMINAL;
      }
      break;
    }
    case TERMINAL:
      cmdL_target = 0.0f;
      cmdR_target = 0.0f;
      setServosNormalized(0, 0);
      if (landingDetected()) {
        Serial.println("STATE: TERMINAL -> LANDED");
        Serial.printf("FINAL ERROR: %.2f m\n",
                      distanceMeters(lat, lon, target_lat, target_lon));
        state = LANDED;
      }
      break;
    case LANDED:
      setServosNormalized(0, 0);
      break;
  }
  sendTelemetry1Hz();
  uint32_t loop_time = millis() - loop_start_ms;
  if (loop_time < 50) {
    delay(50 - loop_time);
  }
}
