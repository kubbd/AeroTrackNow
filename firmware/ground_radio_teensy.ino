// ground_radio_teensy.ino
// Teensy 4.0 ground radio bridge: USB Serial <-> SX1262 LoRa
// Forwards raw bytes without parsing or app-level CRC logic.

#include <Arduino.h>
#include <RadioLib.h>

// ===================== PINOUT (matches flight system) =====================
static constexpr int PIN_LORA_CS    = 10;
static constexpr int PIN_LORA_DIO1  = 7;
static constexpr int PIN_LORA_RESET = 8;
static constexpr int PIN_LORA_BUSY  = -1; // set if wired; -1 if not connected

// ===================== LoRa CONFIG =====================
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

  int st = radio.begin(LORA_FREQ_MHZ);
  if (st != RADIOLIB_ERR_NONE) {
    // If radio init fails, keep running so USB serial is still usable.
    return;
  }

  radio.setOutputPower(LORA_TX_DBM);
  // Match flight radio settings so payload length matches on both ends.
  // The flight firmware uses radio.setCRC(true); mismatched CRC modes will
  // cause extra bytes in the payload and break the ground-side CRC check.
  radio.setCRC(true);

  radio.setDio1Action(setRadioFlag);
  startReceive();
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
