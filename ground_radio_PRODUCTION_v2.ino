/*******************************************************
 * AeroTrackNow Ground Radio - PRODUCTION VERSION
 * Teensy 4.0 USB-to-LoRa Bridge
 * 
 * CRITICAL FIX: CRC must be enabled to match flight code!
 * 
 * This version has the CRC bug FIXED.
 * 
 * Upload this to your ground station Teensy.
 *******************************************************/

#include <Arduino.h>
#include <RadioLib.h>

// ===================== PINOUT (Same as flight) =====================
static constexpr int PIN_LORA_CS    = 10;
static constexpr int PIN_LORA_DIO1  = 7;
static constexpr int PIN_LORA_RESET = 8;
static constexpr int PIN_LORA_BUSY  = -1;

// ===================== LORA CONFIG =====================
static constexpr float LORA_FREQ_MHZ = 433.0f;
static constexpr int   LORA_TX_DBM   = 14;

// RadioLib Module(CS, DIO1, RST, BUSY)
SX1262 radio = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RESET, PIN_LORA_BUSY);

// ===================== STATE =====================
static volatile bool radioEvent = false;
static bool txInProgress = false;

static constexpr size_t LORA_BUF_LEN = 255;
static uint8_t loraBuf[LORA_BUF_LEN];

static void setRadioFlag() {
  radioEvent = true;
}

static void startReceive() {
  radio.startReceive();
  txInProgress = false;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("AeroTrackNow Ground Station v2.0");
  Serial.println("Initializing LoRa...");
  
  int st = radio.begin(LORA_FREQ_MHZ);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("ERROR: LoRa init failed: %d\n", st);
    while (1) delay(1000);
  }
  
  radio.setOutputPower(LORA_TX_DBM);
  
  // CRITICAL FIX: CRC must be TRUE to match flight code!
  // OLD (BROKEN): radio.setCRC(false);
  // NEW (FIXED):
  radio.setCRC(true);
  
  radio.setDio1Action(setRadioFlag);
  startReceive();
  
  Serial.println("Ground station ready!");
  Serial.println("CRC enabled (matches flight code)");
}

void loop() {
  // Handle radio events (RX done or TX done)
  if (radioEvent) {
    radioEvent = false;
    
    if (txInProgress) {
      // TX completed; return to RX
      radio.finishTransmit();
      startReceive();
    } else {
      // RX completed; forward raw bytes to USB
      int len = radio.getPacketLength();
      if (len > 0 && len <= (int)LORA_BUF_LEN) {
        int err = radio.readData(loraBuf, len);
        if (err == RADIOLIB_ERR_NONE) {
          Serial.write(loraBuf, len);
        }
      }
      startReceive();
    }
  }
  
  // Forward USB serial bytes to LoRa (when not transmitting)
  if (!txInProgress && Serial.available() > 0) {
    int toRead = Serial.available();
    if (toRead > (int)LORA_BUF_LEN) {
      toRead = (int)LORA_BUF_LEN;
    }
    
    int readCount = Serial.readBytes(loraBuf, toRead);
    if (readCount > 0) {
      int err = radio.startTransmit(loraBuf, (size_t)readCount);
      if (err == RADIOLIB_ERR_NONE) {
        txInProgress = true;
      } else {
        startReceive();
      }
    }
  }
}
