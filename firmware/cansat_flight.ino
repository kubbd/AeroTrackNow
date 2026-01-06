/*******************************************************
 * AeroTrackNow CanSat – Teensy 4.0 Flight Code
 * Copy-paste into Arduino IDE and upload to Teensy 4.0.
 *
 * Features:
 * - State machine: BOOT -> WAIT_FOR_DROP -> WAIT_FOR_STABLE_DESCENT
 *                  -> GUIDED_DESCENT -> TERMINAL -> LANDED
 * - Predictive guidance toward target (uses predicted landing point)
 * - LoRa bidirectional:
 *     * receives TARGET once
 *     * receives RTK (RTCM3) continuously and forwards to GNSS UART
 *     * sends telemetry at 1 Hz
 * - GNSS: u-blox ZED-X20P over UART (Serial5 on Teensy 4.0)
 * - IMU: BNO085 over I2C (for drop detection accel spike)
 * - Barometer: BMP280 over I2C (pressure/temp + sink-rate)
 * - Two servos for parafoil steering
 * - NO buzzer
 *
 * Required libraries (Arduino Library Manager):
 * - RadioLib
 * - SparkFun u-blox GNSS Arduino Library
 * - Adafruit BMP280 Library (+ Adafruit Unified Sensor)
 * - Adafruit BNO08x
 * Built-in: Wire, Servo, SD
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

// ===================== PIN MAP (FINAL) =====================
// GNSS – ZED-X20P (U1)
static constexpr int PIN_GNSS_RX     = 21; // Teensy RX (connected to GNSS TX1)
static constexpr int PIN_GNSS_TX     = 20; // Teensy TX (connected to GNSS RX1)
static constexpr int PIN_GNSS_TPS    = 24; // optional input
static constexpr int PIN_GNSS_RTKFIX = 25; // optional input

// I2C – IMU + BMP280
static constexpr int PIN_I2C_SDA = 18;
static constexpr int PIN_I2C_SCL = 19;

// LoRa – SX1262 (U5)
static constexpr int PIN_LORA_CS    = 10;
static constexpr int PIN_LORA_DIO1  = 7;
static constexpr int PIN_LORA_RESET = 8;
static constexpr int PIN_LORA_BUSY  = -1; // set if wired; -1 if not connected

// SD card
static constexpr int PIN_SD_CS = 23;

// Servos
static constexpr int PIN_SERVO_LEFT  = 3;
static constexpr int PIN_SERVO_RIGHT = 4;

// ===================== BUILD OPTIONS =====================
static constexpr bool ENABLE_SD_LOGGING = true;  // set false to disable SD

// ===================== RADIO SETTINGS =====================
static constexpr float LORA_FREQ_MHZ = 433.0f;
static constexpr int   LORA_TX_DBM   = 14;   // adjust to comply with your rules
static constexpr uint32_t TELEMETRY_PERIOD_MS = 1000;

// ===================== GUIDANCE TUNING =====================
// Normalized servo command range is -1..+1
static constexpr float KP_TURN = 0.9f;             // turning aggressiveness
static constexpr float MAX_TURN = 0.45f;           // clamp (avoid aggressive turns)
static constexpr float TERMINAL_HEIGHT_M = 8.0f;   // below this: servos neutral
static constexpr float CAPTURE_RADIUS_M = 1.0f;    // predicted landing close enough -> terminal

// Stabilization checks
static constexpr float STABLE_SINK_MIN = 3.5f; // m/s (positive downward)
static constexpr float STABLE_SINK_MAX = 7.5f; // m/s (positive downward)
static constexpr uint32_t STABLE_HOLD_MS = 2500;

// Drop detection (accel “event” threshold)
static constexpr float DROP_ACCEL_SPIKE = 8.0f; // m/s^2 deviation from 1g

// ===================== TELEMETRY PACKET =====================
#pragma pack(push, 1)
struct TelemetryPacket {
  uint8_t  msgType;          // 0x03
  uint32_t time_ms;

  int32_t  lat_e7;           // deg * 1e7
  int32_t  lon_e7;           // deg * 1e7

  int16_t  height_agl_dm;    // meters * 10 (AGL proxy = GNSS_alt_MSL - target_alt_MSL)

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

// msg types
static constexpr uint8_t MSG_TARGET    = 0x01;
static constexpr uint8_t MSG_RTK       = 0x02; // payload = RTCM bytes
static constexpr uint8_t MSG_TELEMETRY = 0x03;

// ===================== STATE MACHINE =====================
enum MissionState : uint8_t {
  BOOT = 0,
  WAIT_FOR_DROP,
  WAIT_FOR_STABLE_DESCENT,
  GUIDED_DESCENT,
  TERMINAL,
  LANDED
};

MissionState state = BOOT;

// ===================== GLOBALS =====================
SFE_UBLOX_GNSS gnss;

Adafruit_BMP280 bmp;
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t bnoValue;

// RadioLib Module(CS, DIO1, RST, BUSY)
SX1262 radio = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RESET, PIN_LORA_BUSY);

Servo servoL, servoR;

File logFile;

// Target
bool targetReceived = false;
double target_lat = 0.0;
double target_lon = 0.0;
double target_alt_msl = 0.0;

// GNSS live
double lat = 0.0, lon = 0.0;
double gnss_alt_msl = 0.0;
double ground_speed = 0.0;      // m/s
double ground_track_rad = 0.0;  // radians

// Baro live
double pressure_hPa = 0.0;
double temp_c = 0.0;
double sink_rate = 5.5; // m/s (positive downward)
double last_baro_alt = 0.0;
uint32_t last_baro_ms = 0;

// Prediction
double pred_lat = 0.0, pred_lon = 0.0;

// Servo commands (normalized -1..+1)
float cmdL = 0.0f, cmdR = 0.0f;

// Timers
uint32_t lastTelemetryMs = 0;
uint32_t stableStartMs = 0;

// ===================== UTILS =====================
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

// Servo mapping: -1..+1 -> microseconds
static void setServosNormalized(float left, float right) {
  left  = clampf(left,  -1.0f, 1.0f);
  right = clampf(right, -1.0f, 1.0f);

  const int NEUTRAL_US = 1500;
  const int RANGE_US   = 350; // start smaller (e.g., 200) and increase carefully
  int usL = NEUTRAL_US + (int)(left  * RANGE_US);
  int usR = NEUTRAL_US + (int)(right * RANGE_US);

  servoL.writeMicroseconds(usL);
  servoR.writeMicroseconds(usR);
}

// ===================== GNSS =====================
static void setupGNSS() {
  Serial5.begin(38400);              // Serial5 = pins 21(RX5)/20(TX5)
  (void)gnss.begin(Serial5);

  gnss.setNavigationFrequency(10);   // 10 Hz
  gnss.setAutoPVT(true);             // Position/Velocity/Time

  gnss.setUART1Output(COM_TYPE_UBX);
  gnss.setUART1Input(COM_TYPE_UBX | COM_TYPE_RTCM3);
}

static void updateGNSS() {
  gnss.checkUblox();
  if (gnss.getPVT()) {
    lat = gnss.getLatitude()  * 1e-7;
    lon = gnss.getLongitude() * 1e-7;

    gnss_alt_msl = gnss.getAltitudeMSL() / 1000.0;  // mm -> m

    ground_speed = gnss.getGroundSpeed() / 1000.0;  // mm/s -> m/s
    double headDeg = gnss.getHeading() * 1e-5;       // deg
    ground_track_rad = headDeg * DEG_TO_RAD;
  }
}

static void feedRTKToGNSS(const uint8_t* rtcm, size_t len) {
  Serial5.write(rtcm, len);
}

// ===================== BMP280 =====================
static void setupBMP280() {
  bmp.begin(0x76); // change to 0x77 if needed
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_63);

  last_baro_ms = millis();
  last_baro_alt = bmp.readAltitude(1013.25); // relative
}

static void updateBMP280() {
  pressure_hPa = bmp.readPressure() / 100.0;
  temp_c = bmp.readTemperature();

  uint32_t now = millis();
  double alt = bmp.readAltitude(1013.25); // relative
  uint32_t dt_ms = now - last_baro_ms;

  if (dt_ms >= 100) {
    double dt = dt_ms / 1000.0;
    sink_rate = (last_baro_alt - alt) / dt; // positive downward
    last_baro_alt = alt;
    last_baro_ms = now;
  }
}

// ===================== BNO085 =====================
static void setupBNO085() {
  if (!bno08x.begin_I2C(0x4A, &Wire)) return;
  bno08x.enableReport(SH2_ACCELEROMETER, 50);
}

static bool readIMUAccel(float &ax, float &ay, float &az) {
  if (bno08x.getSensorEvent(&bnoValue)) {
    if (bnoValue.sensorId == SH2_ACCELEROMETER) {
      ax = bnoValue.un.accelerometer.x;
      ay = bnoValue.un.accelerometer.y;
      az = bnoValue.un.accelerometer.z;
      return true;
    }
  }
  return false;
}

// ===================== LoRa =====================
static void setupLoRa() {
  int st = radio.begin(LORA_FREQ_MHZ);
  if (st != RADIOLIB_ERR_NONE) return;

  radio.setOutputPower(LORA_TX_DBM);
  radio.setCRC(true);
  radio.startReceive();
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
      uint16_t crcCalc = crc16_ccitt((uint8_t*)&tp, sizeof(tp) - 2);
      if (crcCalc == tp.crc16) {
        target_lat = tp.tgt_lat_e7 * 1e-7;
        target_lon = tp.tgt_lon_e7 * 1e-7;
        target_alt_msl = tp.tgt_alt_cm / 100.0;
        targetReceived = true;
      }
    }

    if (type == MSG_RTK && len > 1) {
      feedRTKToGNSS(&buf[1], (size_t)(len - 1));
    }
  }

  radio.startReceive();
}

// ===================== GUIDANCE =====================
static void runPredictiveGuidance() {
  double height_agl = gnss_alt_msl - target_alt_msl;
  if (height_agl <= 0.5) { cmdL = 0; cmdR = 0; return; }

  double sr = (sink_rate < 0.5) ? 0.5 : sink_rate;
  double t_ground = height_agl / sr;

  double dNorth = ground_speed * cos(ground_track_rad) * t_ground;
  double dEast  = ground_speed * sin(ground_track_rad) * t_ground;

  pred_lat = lat + dNorth / 111111.0;
  pred_lon = lon + dEast  / metersPerDegLon(lat);

  double errBear = bearingRad(pred_lat, pred_lon, target_lat, target_lon);
  double heading_error = errBear - ground_track_rad;

  while (heading_error > PI)  heading_error -= TWO_PI;
  while (heading_error < -PI) heading_error += TWO_PI;

  float turn = clampf((float)(KP_TURN * heading_error), -MAX_TURN, MAX_TURN);

  cmdL =  turn;
  cmdR = -turn;
}

// ===================== EVENT DETECTION =====================
static bool dropDetected() {
  // Must be descending (prevents rocket launch/drone lift false trigger)
  if (sink_rate < 1.0) return false;

  float ax, ay, az;
  if (readIMUAccel(ax, ay, az)) {
    float aMag = sqrt(ax*ax + ay*ay + az*az);
    if (fabs(aMag - 9.81f) > DROP_ACCEL_SPIKE) return true;
  }
  return false;
}

static bool descentStable() {
  if (sink_rate >= STABLE_SINK_MIN && sink_rate <= STABLE_SINK_MAX) {
    if (stableStartMs == 0) stableStartMs = millis();
    if (millis() - stableStartMs >= STABLE_HOLD_MS) return true;
  } else {
    stableStartMs = 0;
  }
  return false;
}

static bool landingDetected() {
  static uint32_t stillStart = 0;
  if (sink_rate < 0.3) {
    if (stillStart == 0) stillStart = millis();
    if (millis() - stillStart > 1000) return true;
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

  double height_agl = gnss_alt_msl - target_alt_msl;
  p.height_agl_dm = (int16_t)clampf((float)(height_agl * 10.0), -32768, 32767);

  p.pressure_hPa_x10 = (uint16_t)clampf((float)(pressure_hPa * 10.0), 0, 65535);
  p.temp_c_x10 = (int16_t)clampf((float)(temp_c * 10.0), -32768, 32767);

  p.ground_speed_cms = (uint16_t)clampf((float)(ground_speed * 100.0), 0, 65535);

  p.servo_left_x1000  = (int16_t)clampf(cmdL * 1000.0f, -1000, 1000);
  p.servo_right_x1000 = (int16_t)clampf(cmdR * 1000.0f, -1000, 1000);

  p.pred_lat_e7 = (int32_t)llround(pred_lat * 1e7);
  p.pred_lon_e7 = (int32_t)llround(pred_lon * 1e7);

  p.mission_state = (uint8_t)state;

  p.crc16 = crc16_ccitt((uint8_t*)&p, sizeof(p) - 2);

  radio.standby();
  (void)radio.transmit((uint8_t*)&p, sizeof(p));
  radio.startReceive();

  if (ENABLE_SD_LOGGING && logFile) {
    logFile.print(p.time_ms); logFile.print(',');
    logFile.print(lat, 7); logFile.print(',');
    logFile.print(lon, 7); logFile.print(',');
    logFile.print(height_agl, 2); logFile.print(',');
    logFile.print(pressure_hPa, 2); logFile.print(',');
    logFile.print(temp_c, 2); logFile.print(',');
    logFile.print(ground_speed, 2); logFile.print(',');
    logFile.print(cmdL, 3); logFile.print(',');
    logFile.print(cmdR, 3); logFile.print(',');
    logFile.print(pred_lat, 7); logFile.print(',');
    logFile.print(pred_lon, 7); logFile.print(',');
    logFile.println((int)state);
    logFile.flush();
  }
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);

  // I2C
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();

  // Servos
  servoL.attach(PIN_SERVO_LEFT);
  servoR.attach(PIN_SERVO_RIGHT);
  setServosNormalized(0, 0);

  // Sensors
  setupBMP280();
  setupBNO085();

  // GNSS
  setupGNSS();

  // SD (optional)
  if (ENABLE_SD_LOGGING) {
    if (SD.begin(PIN_SD_CS)) {
      logFile = SD.open("flight.csv", FILE_WRITE);
      if (logFile) {
        logFile.println("t_ms,lat,lon,height_agl,press_hPa,temp_C,gs_mps,cmdL,cmdR,pred_lat,pred_lon,state");
        logFile.flush();
      }
    }
  }

  // LoRa
  setupLoRa();

  state = BOOT;
}

void loop() {
  updateBMP280();
  updateGNSS();
  pollLoRa();

  switch (state) {
    case BOOT:
      setServosNormalized(0, 0);
      if (targetReceived) state = WAIT_FOR_DROP;
      break;

    case WAIT_FOR_DROP:
      setServosNormalized(0, 0);
      if (dropDetected()) state = WAIT_FOR_STABLE_DESCENT;
      break;

    case WAIT_FOR_STABLE_DESCENT:
      setServosNormalized(0, 0);
      if (descentStable()) state = GUIDED_DESCENT;
      break;

    case GUIDED_DESCENT: {
      if (!targetReceived) { setServosNormalized(0, 0); break; }

      runPredictiveGuidance();

      double height_agl = gnss_alt_msl - target_alt_msl;
      double dist_pred_to_target = distanceMeters(pred_lat, pred_lon, target_lat, target_lon);

      if (height_agl < TERMINAL_HEIGHT_M && dist_pred_to_target < CAPTURE_RADIUS_M) {
        cmdL = 0; cmdR = 0;
        setServosNormalized(0, 0);
        state = TERMINAL;
      } else {
        setServosNormalized(cmdL, cmdR);
      }
      break;
    }

    case TERMINAL:
      cmdL = 0; cmdR = 0;
      setServosNormalized(0, 0);
      if (landingDetected()) state = LANDED;
      break;

    case LANDED:
      cmdL = 0; cmdR = 0;
      setServosNormalized(0, 0);
      break;
  }

  sendTelemetry1Hz();
}
