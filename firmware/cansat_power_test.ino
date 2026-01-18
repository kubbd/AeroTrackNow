/*******************************************************
 * AeroTrackNow CanSat – Power Endurance Test
 *
 * Purpose:
 * - Pre-launch, constant-load soak test to estimate battery life.
 * - Keeps subsystems active (GNSS, IMU, barometer, LoRa, SD logging).
 *
 * Upload to Teensy 4.0 and leave running until battery depletion.
 *
 * Required libraries (Arduino Library Manager):
 * - RadioLib
 * - SparkFun u-blox GNSS Arduino Library
 * - Adafruit BMP280 Library (+ Adafruit Unified Sensor)
 * - Adafruit BNO08x
 * Built-in: Wire, SD
 *******************************************************/

#include <Arduino.h>
#include <Wire.h>
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

// ===================== RADIO SETTINGS =====================
static constexpr float LORA_FREQ_MHZ = 433.0f;
static constexpr int   LORA_TX_DBM   = 14;   // adjust to comply with your rules
static constexpr uint32_t TELEMETRY_PERIOD_MS = 1000;

// ===================== TELEMETRY PACKET =====================
#pragma pack(push, 1)
struct PowerTelemetry {
  uint8_t  msgType;          // 0x55
  uint32_t time_ms;

  int32_t  lat_e7;           // deg * 1e7
  int32_t  lon_e7;           // deg * 1e7
  int16_t  alt_m_x10;        // meters * 10

  uint16_t pressure_hPa_x10; // hPa * 10
  int16_t  temp_c_x10;       // C * 10

  uint16_t imu_rate_dps_x10; // deg/s * 10 (approx magnitude)

  uint16_t crc16;
};
#pragma pack(pop)

static constexpr uint8_t MSG_POWER = 0x55;

// ===================== GLOBALS =====================
SFE_UBLOX_GNSS gnss;
Adafruit_BMP280 bmp;
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t bnoValue;

SX1262 radio = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RESET, PIN_LORA_BUSY);

File logFile;

// ===================== UTILS =====================
static uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

static void logLine(const String& line) {
  if (!logFile) return;
  logFile.println(line);
  logFile.flush();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  delay(200);

  // I2C
  Wire.begin();
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.setClock(400000);

  // GNSS
  Serial5.begin(115200);
  if (!gnss.begin(Serial5)) {
    Serial.println("GNSS not detected.");
  } else {
    gnss.setUART1Output(COM_TYPE_UBX);
    gnss.setNavigationFrequency(1);
  }

  // BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 not detected.");
  } else {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
  }

  // BNO08x
  if (!bno08x.begin_I2C()) {
    Serial.println("BNO08x not detected.");
  } else {
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000); // 100 Hz
  }

  // SD card
  if (SD.begin(PIN_SD_CS)) {
    logFile = SD.open("POWERLOG.CSV", FILE_WRITE);
    if (logFile) {
      logLine("time_ms,lat_e7,lon_e7,alt_m_x10,pressure_hpa_x10,temp_c_x10,gyro_dps_x10");
    }
  } else {
    Serial.println("SD init failed.");
  }

  // LoRa
  if (radio.begin() != RADIOLIB_ERR_NONE) {
    Serial.println("LoRa init failed.");
  } else {
    radio.setFrequency(LORA_FREQ_MHZ);
    radio.setOutputPower(LORA_TX_DBM);
    radio.setBandwidth(125.0);
    radio.setSpreadingFactor(9);
    radio.setCodingRate(7);
  }

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  static uint32_t lastTelemetryMs = 0;

  uint32_t now = millis();
  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;

    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;

    if (gnss.getGnssFixOk()) {
      lat = gnss.getLatitude() * 1e-7;
      lon = gnss.getLongitude() * 1e-7;
      alt = gnss.getAltitudeMSL() / 1000.0;
    }

    float pressure = bmp.readPressure() / 100.0f;
    float temp = bmp.readTemperature();

    float gyroMag = 0.0f;
    if (bno08x.getSensorEvent(&bnoValue)) {
      if (bnoValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        float gx = bnoValue.un.gyroscope.x;
        float gy = bnoValue.un.gyroscope.y;
        float gz = bnoValue.un.gyroscope.z;
        gyroMag = sqrtf(gx * gx + gy * gy + gz * gz) * 57.2958f; // rad/s -> deg/s
      }
    }

    PowerTelemetry pkt{};
    pkt.msgType = MSG_POWER;
    pkt.time_ms = now;
    pkt.lat_e7 = (int32_t)lround(lat * 1e7);
    pkt.lon_e7 = (int32_t)lround(lon * 1e7);
    pkt.alt_m_x10 = (int16_t)lround(alt * 10.0);
    pkt.pressure_hPa_x10 = (uint16_t)lround(pressure * 10.0f);
    pkt.temp_c_x10 = (int16_t)lround(temp * 10.0f);
    pkt.imu_rate_dps_x10 = (uint16_t)lround(gyroMag * 10.0f);

    pkt.crc16 = 0;
    pkt.crc16 = crc16_ccitt(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt) - sizeof(pkt.crc16));

    radio.transmit(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));

    String line = String(now) + "," +
                  String(pkt.lat_e7) + "," +
                  String(pkt.lon_e7) + "," +
                  String(pkt.alt_m_x10) + "," +
                  String(pkt.pressure_hPa_x10) + "," +
                  String(pkt.temp_c_x10) + "," +
                  String(pkt.imu_rate_dps_x10);
    logLine(line);

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}
