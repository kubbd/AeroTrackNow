# PINOUT (Teensy 4.0)

These are the **actual Teensy pin numbers** (the leading number in the KiCad pin name).

## GNSS (ZED-X20P)
- GNSS UART: **Serial5**
- Teensy **RX** = **21** (GNSS TX1 → Teensy RX)
- Teensy **TX** = **20** (GNSS RX1 ← Teensy TX)
- TPS (optional) = **24** Currently unconnected
- RTKFIX (optional) = **25** Currently unconnected

## I2C (BNO085 + BMP280)
- SDA = **18**
- SCL = **19**

## LoRa (SX1262 / CORE1262-LF)
This module requires controlling the external RF switch using RXEN/TXEN (not DIO2/DIO3), and it may need a slow SPI clock (~500 kHz) for reliable operation.
- SCK = **13**
- MOSI = **11**
- MISO = **12**
- NSS/CS = **10**
- DIO1 = **7**
- RESET = **8**
- BUSY = **9**
- RXEN = **5**
- TXEN = **6**

## SD Card
- SPI shared with LoRa: **SCK/MOSI/MISO = 13 / 11 / 12**
- CS = **23**

## Servos
- Left servo PWM = **3**
- Right servo PWM = **4**
