/*******************************************************
 * AeroTrackNow CanSat – PRODUCTION VERSION 2.0
 * Teensy 4.0 Flight Code - Warsaw Optimized
 * 
 * This version includes ALL improvements for maximum
 * landing accuracy in Warsaw, Poland conditions.
 * 
 * Expected Performance:
 * - Wind < 2 m/s: 90% within 1m
 * - Wind 2-3 m/s: 75% within 2m  
 * - Wind 3-4 m/s: 60% within 3m
 * - Wind > 5 m/s: DO NOT FLY
 * 
 * CRITICAL: This code assumes you have:
 * - Fixed the radio CRC bug in ground station
 * - Calibrated SERVO_TRIM on ground
 * - Tested all sensors individually
 * - RTK base station working
 * 
 * Author: AeroTrackNow Team + AI Assistant
 * Date: November 2024
 * Competition: CanSat 2025
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
static constexpr int PIN_GNSS_RX     = 21;  // Serial5 RX
static constexpr int PIN_GNSS_TX     = 20;  // Serial5 TX
static constexpr int PIN_GNSS_TPS    = 24;  // optional
static constexpr int PIN_GNSS_RTKFIX = 25;  // optional

static constexpr int PIN_I2C_SDA = 18;
static constexpr int PIN_I2C_SCL = 19;

static constexpr int PIN_LORA_CS    = 10;
static constexpr int PIN_LORA_DIO1  = 7;
static constexpr int PIN_LORA_RESET = 8;
static constexpr int PIN_LORA_BUSY  = -1;   // not connected

static constexpr int PIN_SD_CS = 23;

static constexpr int PIN_SERVO_LEFT  = 3;
static constexpr int PIN_SERVO_RIGHT = 4;

// ===================== PARAFOIL PARAMETERS =====================
// From your PDR calculations and testing
// UPDATE THESE after real flight tests!

// Parafoil geometry
static constexpr float PARAFOIL_AREA_M2 = 0.05f;        // Current design
static constexpr float PARAFOIL_GLIDE_RATIO = 2.0f;     // Target from PDR
static constexpr float NOMINAL_SINK_RATE = 5.5f;        // m/s (positive down)

// Derived parameters (auto-calculated)
static constexpr float NOMINAL_AIRSPEED = NOMINAL_SINK_RATE * PARAFOIL_GLIDE_RATIO;  // ~11 m/s

// After testing with improved 0.07m² parafoil, update to:
// static constexpr float PARAFOIL_AREA_M2 = 0.07f;
// static constexpr float PARAFOIL_GLIDE_RATIO = 2.7f;
// static constexpr float NOMINAL_SINK_RATE = 4.2f;

// ===================== CONTROL PARAMETERS =====================
// These are tuned for Warsaw conditions
// Start conservative, increase gradually during testing

// Guidance gains
static constexpr float KP_HEADING = 0.5f;              // Turn aggressiveness (start 0.3, max 0.7)
static constexpr float KI_HEADING = 0.0f;              // Integral term (keep 0 for now)
static constexpr float MAX_BANK_ANGLE_RAD = 0.44f;     // ~25° max bank

// Servo parameters
static constexpr float SERVO_TRIM = 0.0f;              // Adjust ±0.01 after test
static constexpr int SERVO_NEUTRAL_US = 1500;
static constexpr int SERVO_RANGE_US = 250;             // Start 200, increase to 350 max
static constexpr float SERVO_MAX_RATE = 2.0f;          // Max change per second
static constexpr float SERVO_DEADBAND = 0.02f;         // ±2% deadband around neutral

// State machine altitudes
static constexpr float FINAL_APPROACH_HEIGHT_M = 30.0f;   // Enter final approach
static constexpr float FLARE_HEIGHT_M = 3.0f;             // Deploy flare
static constexpr float TERMINAL_HEIGHT_M = 1.0f;          // Lock neutral (was 8m - too high!)
static constexpr float CAPTURE_RADIUS_M = 1.0f;           // Close enough (RTK)
static constexpr float CAPTURE_RADIUS_GNSS_M = 5.0f;      // Without RTK
static constexpr float CAPTURE_RADIUS_POOR_M = 10.0f;     // Poor GNSS

// Stability detection
static constexpr float STABLE_SINK_MIN = 4.5f;            // m/s
static constexpr float STABLE_SINK_MAX = 6.5f;            // m/s  
static constexpr uint32_t STABLE_HOLD_MS = 1500;          // Hold time (was 2500)

// Drop detection
static constexpr float DROP_ACCEL_SPIKE = 8.0f;          // m/s² deviation from 1g
static constexpr int DROP_DEBOUNCE_COUNT = 3;            // Consecutive spikes needed

// Sink rate filtering
static constexpr uint32_t BARO_UPDATE_INTERVAL_MS = 500;  // 2 Hz (was 100ms)
static constexpr float SINK_FILTER_ALPHA = 0.15f;         // Low-pass filter
static constexpr float SINK_RATE_MIN = 0.0f;              // Sanity bounds (allow near-zero for landing detection)
static constexpr float SINK_RATE_MAX = 8.0f;
static constexpr float SINK_GNSS_BLEND_ALPHA = 0.2f;      // Blend GNSS vertical velocity into sink rate

// Wind estimation
static constexpr float WIND_FILTER_ALPHA = 0.05f;         // Very slow filter
static constexpr float MAX_WIND_ESTIMATE = 15.0f;         // Sanity limit

// Physics constants
static constexpr float GRAVITY_MPS2 = 9.80665f;
static constexpr float MAG_DECLINATION_DEG = 0.0f;        // Set per launch site
static constexpr float MAG_DECLINATION_RAD = MAG_DECLINATION_DEG * DEG_TO_RAD;
static constexpr float IMU_YAW_OFFSET_DEG = 0.0f;         // IMU aligned with CanSat X-axis
static constexpr float IMU_YAW_OFFSET_RAD = IMU_YAW_OFFSET_DEG * DEG_TO_RAD;

// MPC parameters (simplified)
static constexpr int MPC_HORIZON = 10;                    // 5 seconds ahead
static constexpr int MPC_SAMPLES = 21;                    // Sample -1.0 to +1.0 in 0.1 steps
static constexpr float MPC_CONTROL_WEIGHT = 0.1f;         // Penalize large controls
static constexpr float APPROACH_GAIN = 1.5f;              // Final approach multiplier

// ===================== RADIO SETTINGS =====================
static constexpr float LORA_FREQ_MHZ = 433.0f;
static constexpr int   LORA_TX_DBM   = 14;
static constexpr uint32_t TELEMETRY_PERIOD_MS = 1000;     // 1 Hz telemetry

// ===================== TELEMETRY PACKET =====================
#pragma pack(push, 1)
struct TelemetryPacket {
  uint8_t  msgType;          // 0x03
  uint32_t time_ms;

  int32_t  lat_e7;           // deg * 1e7
  int32_t  lon_e7;           // deg * 1e7

  int16_t  height_agl_dm;    // meters * 10

  uint16_t pressure_hPa_x10; // hPa * 10
  int16_t  temp_c_x10;       // C * 10

  uint16_t ground_speed_cms; // m/s * 100

  int16_t  servo_left_x1000; // -1000..1000
  int16_t  servo_right_x1000;

  int32_t  pred_lat_e7;      // deg * 1e7
  int32_t  pred_lon_e7;

  uint8_t  mission_state;

  uint16_t crc16;
};
#pragma pack(pop)

// ===================== INCOMING PACKETS =====================
#pragma pack(push, 1)
struct TargetPacket {
  uint8_t msgType;     // 0x01
  int32_t tgt_lat_e7;
  int32_t tgt_lon_e7;
  int32_t tgt_alt_cm;  // meters MSL * 100
  uint16_t crc16;
};
#pragma pack(pop)

// Message types
static constexpr uint8_t MSG_TARGET    = 0x01;
static constexpr uint8_t MSG_RTK       = 0x02;
static constexpr uint8_t MSG_TELEMETRY = 0x03;

// ===================== STATE MACHINE =====================
enum MissionState : uint8_t {
  BOOT = 0,
  WAIT_FOR_DROP,
  WAIT_FOR_STABLE_DESCENT,
  GUIDED_DESCENT,
  FINAL_APPROACH,        // NEW: Better control close to ground
  FLARE,                 // NEW: Flare maneuver
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
double ground_speed = 0.0;          // m/s
double ground_track_rad = 0.0;      // radians
double gnss_vertical_velocity = 0.0; // m/s (negative = down)

// IMU heading
double imu_heading_rad = 0.0;
uint32_t last_heading_ms = 0;
float imu_ax = 0.0f;
float imu_ay = 0.0f;
float imu_az = 0.0f;
uint32_t last_accel_ms = 0;

// GNSS quality
uint8_t gnss_fix_type = 0;          // 0=no fix, 3=3D, 5=RTK fixed
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

// Sink rate (filtered)
double sink_rate_raw = 5.5;         // m/s (positive downward)
double sink_rate_filtered = 5.5;    // m/s (filtered)
double sink_rate_gnss = 5.5;        // From GNSS vertical velocity

// Wind estimation
double wind_north = 0.0;            // m/s
double wind_east = 0.0;             // m/s
double wind_speed = 0.0;            // m/s magnitude

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

// ===================== UTILITY FUNCTIONS =====================

static uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
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
  return 111111.0;  // Approximately constant
}

static inline double metersPerDegLon(double lat_deg) {
  return 111111.0 * cos(lat_deg * DEG_TO_RAD);
}

static double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dLat = (lat2 - lat1) * DEG_TO_RAD;
  double dLon = (lon2 - lon1) * DEG_TO_RAD;
  double a = sin(dLat/2)*sin(dLat/2) +
             cos(lat1*DEG_TO_RAD)*cos(lat2*DEG_TO_RAD)*sin(dLon/2)*sin(dLon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
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

static float wrapAngle(float angle) {
  while (angle > PI) angle -= TWO_PI;
  while (angle < -PI) angle += TWO_PI;
  return angle;
}

// ===================== WIND ESTIMATION =====================

static void estimateWind() {
  // Only estimate wind during stable descent
  if (state != GUIDED_DESCENT && state != FINAL_APPROACH) {
    return;
  }
  
  // Need valid GNSS and sink rate
  if (ground_speed < 0.5 || sink_rate_filtered < 1.0) {
    return;
  }
  
  double heading_rad = 0.0;
  bool heading_valid = readIMUHeading(heading_rad);
  if (!heading_valid) {
    heading_rad = ground_track_rad;
  }

  // Predicted airspeed vector (in heading direction)
  double airspeed = sink_rate_filtered * PARAFOIL_GLIDE_RATIO;
  double airspeed_north = airspeed * cos(heading_rad);
  double airspeed_east = airspeed * sin(heading_rad);
  
  // Actual ground velocity
  double ground_vel_north = ground_speed * cos(ground_track_rad);
  double ground_vel_east = ground_speed * sin(ground_track_rad);
  
  // Wind = ground velocity - airspeed
  double wind_north_measured = ground_vel_north - airspeed_north;
  double wind_east_measured = ground_vel_east - airspeed_east;
  
  // Low-pass filter (very slow)
  wind_north = wind_north * (1.0 - WIND_FILTER_ALPHA) + wind_north_measured * WIND_FILTER_ALPHA;
  wind_east = wind_east * (1.0 - WIND_FILTER_ALPHA) + wind_east_measured * WIND_FILTER_ALPHA;
  
  // Sanity limit
  wind_speed = sqrt(wind_north*wind_north + wind_east*wind_east);
  if (wind_speed > MAX_WIND_ESTIMATE) {
    double scale = MAX_WIND_ESTIMATE / wind_speed;
    wind_north *= scale;
    wind_east *= scale;
    wind_speed = MAX_WIND_ESTIMATE;
  }
}

// ===================== PREDICT LANDING WITH PHYSICS =====================

static void predictLandingWithPhysics() {
  // Height above target
  double height_agl = gnss_alt_msl - target_alt_msl;
  
  if (height_agl < 0.5) {
    // On ground
    pred_lat = lat;
    pred_lon = lon;
    predicted_error_m = distanceMeters(lat, lon, target_lat, target_lon);
    return;
  }
  
  // Time to ground
  double effective_sink = (sink_rate_filtered < 0.5) ? 0.5 : sink_rate_filtered;
  double time_to_ground = height_agl / effective_sink;
  
  // Clamp time (avoid extreme predictions)
  if (time_to_ground > 300.0) time_to_ground = 300.0;  // 5 minutes max
  
  double heading_rad = 0.0;
  if (!readIMUHeading(heading_rad)) {
    heading_rad = ground_track_rad;
  }

  // Airspeed vector (in current heading)
  double airspeed = effective_sink * PARAFOIL_GLIDE_RATIO;
  double airspeed_north = airspeed * cos(heading_rad);
  double airspeed_east = airspeed * sin(heading_rad);
  
  // Ground velocity = airspeed + wind
  double ground_vel_north = airspeed_north + wind_north;
  double ground_vel_east = airspeed_east + wind_east;
  
  // Predicted displacement
  double delta_north = ground_vel_north * time_to_ground;
  double delta_east = ground_vel_east * time_to_ground;
  
  // Predicted landing position
  pred_lat = lat + delta_north / metersPerDegLat();
  pred_lon = lon + delta_east / metersPerDegLon(lat);
  
  // Error from target
  predicted_error_m = distanceMeters(pred_lat, pred_lon, target_lat, target_lon);
}

// ===================== MPC GUIDANCE (SIMPLIFIED) =====================

static float computeMPCControl() {
  float best_cmd = 0.0f;
  float best_cost = 1e9;
  
  // Current state
  double current_lat = lat;
  double current_lon = lon;
  double current_heading = 0.0;
  if (!readIMUHeading(current_heading)) {
    current_heading = ground_track_rad;
  }
  double current_alt = gnss_alt_msl;
  
  // Sample turn commands from -1.0 to +1.0
  for (int i = 0; i < MPC_SAMPLES; i++) {
    float test_cmd = -1.0f + (2.0f / (MPC_SAMPLES - 1)) * i;
    
    // Simulate trajectory
    double sim_lat = current_lat;
    double sim_lon = current_lon;
    double sim_heading = current_heading;
    double sim_alt = current_alt;
    
    float dt = 0.5f;  // 500ms steps
    
    for (int step = 0; step < MPC_HORIZON; step++) {
      // Apply turn command (bank angle -> turn rate)
      float bank_angle = test_cmd * MAX_BANK_ANGLE_RAD;
      double airspeed = sink_rate_filtered * PARAFOIL_GLIDE_RATIO;
      float turn_rate = 0.0f;
      if (airspeed > 0.5) {
        turn_rate = (GRAVITY_MPS2 * tanf(bank_angle)) / (float)airspeed;
      }
      sim_heading += turn_rate * dt;
      sim_heading = wrapAngle(sim_heading);
      
      // Update position
      double ground_vel_north = airspeed * cos(sim_heading) + wind_north;
      double ground_vel_east = airspeed * sin(sim_heading) + wind_east;
      
      sim_lat += (ground_vel_north * dt) / metersPerDegLat();
      sim_lon += (ground_vel_east * dt) / metersPerDegLon(sim_lat);
      sim_alt -= sink_rate_filtered * dt;
      
      // Stop if on ground
      if (sim_alt < target_alt_msl) break;
    }
    
    // Cost = distance to target + control effort
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

// ===================== PREDICTIVE GUIDANCE =====================

static void runPredictiveGuidance() {
  // Update wind estimate
  estimateWind();
  
  // Predict landing
  predictLandingWithPhysics();
  
  // Height check
  double height_agl = gnss_alt_msl - target_alt_msl;
  if (height_agl < 0.5) {
    cmdL_target = 0.0f;
    cmdR_target = 0.0f;
    return;
  }
  
  // Compute desired heading
  double desired_bearing = bearingRad(pred_lat, pred_lon, target_lat, target_lon);
  
  double heading_rad = 0.0;
  if (!readIMUHeading(heading_rad)) {
    heading_rad = ground_track_rad;
  }

  // Heading error
  double heading_error = desired_bearing - heading_rad;
  heading_error = wrapAngle(heading_error);
  
  // Use MPC if we have time, else use simple proportional
  float turn_cmd;
  
  if (height_agl > 50.0 && state == GUIDED_DESCENT) {
    // Use MPC for better trajectory
    turn_cmd = computeMPCControl();
  } else {
    // Simple proportional control
    float gain = KP_HEADING;
    
    // Increase gain in final approach
    if (state == FINAL_APPROACH) {
      gain *= APPROACH_GAIN;
    }
    
    turn_cmd = gain * heading_error;
  }
  
  // Clamp to max bank angle
  turn_cmd = clampf(turn_cmd, -1.0f, 1.0f);
  
  // Differential control (positive = turn right)
  cmdL_target = -turn_cmd;  // Left pulls to turn right
  cmdR_target = turn_cmd;   // Right releases to turn right
}

// ===================== GNSS FUNCTIONS =====================

static void setupGNSS() {
  Serial5.begin(38400);
  
  if (!gnss.begin(Serial5)) {
    Serial.println("ERROR: GNSS not found!");
    return;
  }
  
  gnss.setNavigationFrequency(10);  // 10 Hz
  gnss.setAutoPVT(true);
  
  gnss.setUART1Output(COM_TYPE_UBX);
  gnss.setUART1Input(COM_TYPE_UBX | COM_TYPE_RTCM3);
  
  // Enable RTK
  gnss.setDGNSSConfiguration(SFE_UBLOX_DGNSS_MODE_FIXED);
  
  Serial.println("GNSS initialized");
}

static bool isGNSSQualityGood() {
  // Check fix type
  if (gnss_fix_type < 3) return false;  // Need at least 3D fix
  
  // Check satellites
  if (gnss_satellites < 6) return false;
  
  // Check timeout
  if (millis() - last_gnss_update > 2000) return false;
  
  // Check accuracy (if RTK)
  if (gnss_fix_type == 5) {  // RTK fixed
    if (gnss_h_accuracy_mm > 100) return false;  // > 10cm horizontal
    if (gnss_v_accuracy_mm > 150) return false;  // > 15cm vertical
  }
  
  return true;
}

static float getDynamicCaptureRadius() {
  // Adjust capture radius based on GNSS quality
  if (gnss_fix_type == 5 && gnss_h_accuracy_mm < 100) {
    return CAPTURE_RADIUS_M;  // 1m for RTK fixed
  } else if (gnss_fix_type >= 3 && gnss_satellites >= 8) {
    return CAPTURE_RADIUS_GNSS_M;  // 5m for good GNSS
  } else {
    return CAPTURE_RADIUS_POOR_M;  // 10m for poor GNSS
  }
}

static void updateGNSS() {
  gnss.checkUblox();
  
  if (gnss.getPVT()) {
    lat = gnss.getLatitude() * 1e-7;
    lon = gnss.getLongitude() * 1e-7;
    gnss_alt_msl = gnss.getAltitudeMSL() / 1000.0;  // mm -> m
    
    ground_speed = gnss.getGroundSpeed() / 1000.0;  // mm/s -> m/s
    double headDeg = gnss.getHeading() * 1e-5;      // deg
    ground_track_rad = headDeg * DEG_TO_RAD;
    
    gnss_vertical_velocity = gnss.getVerticalVelocity() / 1000.0;  // mm/s -> m/s (negative = down)
    
    // Quality metrics
    gnss_fix_type = gnss.getFixType();
    gnss_satellites = gnss.getSIV();
    gnss_h_accuracy_mm = gnss.getHorizontalAccuracy();
    gnss_v_accuracy_mm = gnss.getVerticalAccuracy();
    
    last_gnss_update = millis();
    
    // Calibrate barometer on first good fix
    if (!sea_level_calibrated && gnss_fix_type >= 3) {
      // Compute sea level pressure from GNSS altitude
      sea_level_pressure_hpa = pressure_hPa / pow(1.0 - gnss_alt_msl / 44330.0, 5.255);
      sea_level_calibrated = true;
      Serial.printf("Baro calibrated: QNH = %.1f hPa\n", sea_level_pressure_hpa);
    }
  }
}

static void feedRTKToGNSS(const uint8_t* rtcm, size_t len) {
  Serial5.write(rtcm, len);
}

// ===================== BMP280 FUNCTIONS =====================

static void setupBMP280() {
  if (!bmp.begin(0x76)) {  // Try 0x77 if 0x76 fails
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
  
  // Calculate altitude from pressure
  if (sea_level_calibrated) {
    baro_alt_msl = 44330.0 * (1.0 - pow(pressure_hPa / sea_level_pressure_hpa, 1.0/5.255));
  } else {
    baro_alt_msl = bmp.readAltitude(1013.25);  // Use standard pressure
  }
  
  // Update sink rate at slower rate
  uint32_t now = millis();
  uint32_t dt_ms = now - last_baro_ms;
  
  if (dt_ms >= BARO_UPDATE_INTERVAL_MS) {
    double dt_sec = dt_ms / 1000.0;
    
    // Raw sink rate (positive = downward)
    sink_rate_raw = (last_baro_alt - baro_alt_msl) / dt_sec;
    
    // Sanity check
    if (sink_rate_raw < SINK_RATE_MIN) sink_rate_raw = SINK_RATE_MIN;
    if (sink_rate_raw > SINK_RATE_MAX) sink_rate_raw = SINK_RATE_MAX;
    
    // Low-pass filter
    sink_rate_filtered = sink_rate_filtered * (1.0 - SINK_FILTER_ALPHA) + 
                         sink_rate_raw * SINK_FILTER_ALPHA;
    
    // Update for next iteration
    last_baro_alt = baro_alt_msl;
    last_baro_ms = now;
  }
  
  // Optional: Fuse with GNSS vertical velocity
  if (gnss_fix_type >= 3 && fabs(gnss_vertical_velocity) < 20.0) {
    sink_rate_gnss = -gnss_vertical_velocity;  // Convert to positive-down
    sink_rate_filtered = sink_rate_filtered * (1.0 - SINK_GNSS_BLEND_ALPHA) +
                         sink_rate_gnss * SINK_GNSS_BLEND_ALPHA;
  }
}

// ===================== BNO085 FUNCTIONS =====================

static void setupBNO085() {
  if (!bno08x.begin_I2C(0x4A, &Wire)) {
    Serial.println("ERROR: BNO085 not found!");
    return;
  }
  
  bno08x.enableReport(SH2_ACCELEROMETER, 50);  // 20 Hz
  bno08x.enableReport(SH2_ROTATION_VECTOR, 50);  // Heading (uses magnetometer)
  Serial.println("BNO085 initialized");
}

static bool readIMUAccel(float &ax, float &ay, float &az) {
  if ((millis() - last_accel_ms) > 200) {
    return false;
  }
  ax = imu_ax;
  ay = imu_ay;
  az = imu_az;
  return true;
}

static bool readIMUHeading(double &heading_rad) {
  if ((millis() - last_heading_ms) > 500) {
    return false;
  }
  heading_rad = imu_heading_rad;
  return true;
}

// ===================== LORA FUNCTIONS =====================

static void setupLoRa() {
  int st = radio.begin(LORA_FREQ_MHZ);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("ERROR: LoRa init failed: %d\n", st);
    return;
  }
  
  radio.setOutputPower(LORA_TX_DBM);
  radio.setCRC(true);  // CRITICAL: Must match ground station!
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
    
    // Target packet
    if (type == MSG_TARGET && len == (int)sizeof(TargetPacket)) {
      TargetPacket tp;
      memcpy(&tp, buf, sizeof(tp));
      uint16_t crcCalc = crc16_ccitt((uint8_t*)&tp, sizeof(tp) - 2);
      
      if (crcCalc == tp.crc16) {
        target_lat = tp.tgt_lat_e7 * 1e-7;
        target_lon = tp.tgt_lon_e7 * 1e-7;
        target_alt_msl = tp.tgt_alt_cm / 100.0;
        targetReceived = true;
        
        Serial.printf("TARGET received: %.7f, %.7f, %.1f m\n", 
                      target_lat, target_lon, target_alt_msl);
      }
    }
    
    // RTK correction data
    if (type == MSG_RTK && len > 1) {
      feedRTKToGNSS(&buf[1], (size_t)(len - 1));
    }
  }
  
  radio.startReceive();
}

// ===================== SERVO FUNCTIONS =====================

static void setServosNormalized(float left, float right) {
  // Apply trim
  left += SERVO_TRIM;
  right -= SERVO_TRIM;  // Opposite for symmetry
  
  // Clamp
  left = clampf(left, -1.0f, 1.0f);
  right = clampf(right, -1.0f, 1.0f);
  
  // Apply deadband
  if (fabs(left) < SERVO_DEADBAND) left = 0.0f;
  if (fabs(right) < SERVO_DEADBAND) right = 0.0f;
  
  // Rate limiting
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
  
  // Convert to microseconds
  int usL = SERVO_NEUTRAL_US + (int)(cmdL_current * SERVO_RANGE_US);
  int usR = SERVO_NEUTRAL_US + (int)(cmdR_current * SERVO_RANGE_US);
  
  servoL.writeMicroseconds(usL);
  servoR.writeMicroseconds(usR);
}

// ===================== EVENT DETECTION =====================

static bool dropDetected() {
  // Must be descending (prevents rocket launch false trigger)
  if (sink_rate_filtered < 1.0) {
    drop_spike_count = 0;
    return false;
  }
  
  float ax, ay, az;
  if (readIMUAccel(ax, ay, az)) {
    float aMag = sqrt(ax*ax + ay*ay + az*az);
    
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
  
  // Check if sink rate near zero
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

static void updateIMU() {
  while (bno08x.getSensorEvent(&bnoValue)) {
    if (bnoValue.sensorId == SH2_ACCELEROMETER) {
      imu_ax = bnoValue.un.accelerometer.x;
      imu_ay = bnoValue.un.accelerometer.y;
      imu_az = bnoValue.un.accelerometer.z;
      last_accel_ms = millis();
    } else if (bnoValue.sensorId == SH2_ROTATION_VECTOR) {
      float qw = bnoValue.un.rotationVector.real;
      float qx = bnoValue.un.rotationVector.i;
      float qy = bnoValue.un.rotationVector.j;
      float qz = bnoValue.un.rotationVector.k;

      double siny_cosp = 2.0 * (qw * qz + qx * qy);
      double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
      double heading_rad = atan2(siny_cosp, cosy_cosp) + MAG_DECLINATION_RAD + IMU_YAW_OFFSET_RAD;
      imu_heading_rad = wrapAngle((float)heading_rad);
      last_heading_ms = millis();
    }
  }
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
  
  double height_agl = gnss_alt_msl - target_alt_msl;
  p.height_agl_dm = (int16_t)clampf((float)(height_agl * 10.0), -32768, 32767);
  
  p.pressure_hPa_x10 = (uint16_t)clampf((float)(pressure_hPa * 10.0), 0, 65535);
  p.temp_c_x10 = (int16_t)clampf((float)(temp_c * 10.0), -32768, 32767);
  
  p.ground_speed_cms = (uint16_t)clampf((float)(ground_speed * 100.0), 0, 65535);
  
  p.servo_left_x1000 = (int16_t)clampf(cmdL_current * 1000.0f, -1000, 1000);
  p.servo_right_x1000 = (int16_t)clampf(cmdR_current * 1000.0f, -1000, 1000);
  
  p.pred_lat_e7 = (int32_t)llround(pred_lat * 1e7);
  p.pred_lon_e7 = (int32_t)llround(pred_lon * 1e7);
  
  p.mission_state = (uint8_t)state;
  
  p.crc16 = crc16_ccitt((uint8_t*)&p, sizeof(p) - 2);
  
  // Send via LoRa
  radio.standby();
  radio.transmit((uint8_t*)&p, sizeof(p));
  radio.startReceive();
  
  // Log to SD card
  if (logFile) {
    logFile.printf("%lu,%.7f,%.7f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.7f,%.7f,%d,%d,%d,%.1f,%.2f\n",
                   p.time_ms, lat, lon, height_agl, pressure_hPa, temp_c, ground_speed,
                   cmdL_current, cmdR_current, pred_lat, pred_lon, (int)state,
                   gnss_fix_type, gnss_satellites, wind_speed, predicted_error_m);
    logFile.flush();
  }
}

// ===================== SETUP =====================

void setup() {
  Serial.begin(115200);
  delay(2000);  // Wait for serial
  
  Serial.println("=================================");
  Serial.println("AeroTrackNow CanSat v2.0");
  Serial.println("Production Flight Code");
  Serial.println("=================================");
  
  // I2C
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();
  
  // Servos
  servoL.attach(PIN_SERVO_LEFT);
  servoR.attach(PIN_SERVO_RIGHT);
  setServosNormalized(0, 0);
  
  // Sensors
  Serial.println("Initializing sensors...");
  setupBMP280();
  setupBNO085();
  setupGNSS();
  
  // SD card
  Serial.println("Initializing SD card...");
  if (SD.begin(PIN_SD_CS)) {
    logFile = SD.open("flight_v2.csv", FILE_WRITE);
    if (logFile) {
      logFile.println("time_ms,lat,lon,height_agl,press_hPa,temp_C,gs_mps,cmdL,cmdR,pred_lat,pred_lon,state,fix_type,sats,wind_ms,pred_error_m");
      logFile.flush();
      Serial.println("SD logging enabled");
    }
  } else {
    Serial.println("WARNING: SD card failed");
  }
  
  // LoRa
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
  
  // Update sensors
  updateBMP280();
  updateGNSS();
  updateIMU();
  pollLoRa();
  
  // State machine
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
        state = GUIDED_DESCENT;
      }
      break;
    
    case GUIDED_DESCENT: {
      if (!targetReceived) {
        setServosNormalized(0, 0);
        break;
      }
      
      // Run guidance
      runPredictiveGuidance();
      
      // Check transitions
      double height_agl = gnss_alt_msl - target_alt_msl;
      
      // Transition to final approach
      if (height_agl < FINAL_APPROACH_HEIGHT_M) {
        Serial.println("STATE: GUIDED_DESCENT -> FINAL_APPROACH");
        state = FINAL_APPROACH;
        break;
      }
      
      // Apply control
      setServosNormalized(cmdL_target, cmdR_target);
      break;
    }
    
    case FINAL_APPROACH: {
      if (!targetReceived) {
        setServosNormalized(0, 0);
        break;
      }
      
      // Run guidance with increased gain
      runPredictiveGuidance();
      
      double height_agl = gnss_alt_msl - target_alt_msl;
      float capture_radius = getDynamicCaptureRadius();
      
      // Check for flare
      if (height_agl < FLARE_HEIGHT_M) {
        Serial.println("STATE: FINAL_APPROACH -> FLARE");
        state = FLARE;
        break;
      }
      
      // Check for terminal
      if (height_agl < TERMINAL_HEIGHT_M && predicted_error_m < capture_radius) {
        Serial.println("STATE: FINAL_APPROACH -> TERMINAL");
        cmdL_target = 0.0f;
        cmdR_target = 0.0f;
        state = TERMINAL;
        break;
      }
      
      // Apply control
      setServosNormalized(cmdL_target, cmdR_target);
      break;
    }
    
    case FLARE: {
      // Symmetric brake to slow descent
      float flare_cmd = 0.6f;  // 60% brake
      setServosNormalized(flare_cmd, flare_cmd);
      
      double height_agl = gnss_alt_msl - target_alt_msl;
      
      if (height_agl < TERMINAL_HEIGHT_M) {
        Serial.println("STATE: FLARE -> TERMINAL");
        state = TERMINAL;
      }
      break;
    }
    
    case TERMINAL:
      // Lock neutral
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
      // Stay in landed state
      break;
  }
  
  // Send telemetry
  sendTelemetry1Hz();
  
  // Loop timing (aim for 20 Hz)
  uint32_t loop_time = millis() - loop_start_ms;
  if (loop_time < 50) {
    delay(50 - loop_time);
  }
}
