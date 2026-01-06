# PINOUT (Teensy 4.0)

These are the **actual Teensy pin numbers** (the leading number in the KiCad pin name).

## GNSS (ZED-X20P)
- GNSS UART: **Serial5**
- Teensy **RX** = **21** (GNSS TX1 → Teensy RX)
- Teensy **TX** = **20** (GNSS RX1 ← Teensy TX)
- TPS (optional) = **24**
- RTKFIX (optional) = **25**

## I2C (BNO085 + BMP280)
- SDA = **18**
- SCL = **19**

## LoRa (SX1262 / CORE1262-LF)
- SCK = **13**
- MOSI = **11**
- MISO = **12**
- NSS/CS = **10**
- DIO1 = **7**
- RESET = **8**
- BUSY = **(not connected in code, set if wired)**

## SD Card
- SPI shared with LoRa: **SCK/MOSI/MISO = 13 / 11 / 12**
- CS = **23**

## Servos
- Left servo PWM = **3**
- Right servo PWM = **4**
