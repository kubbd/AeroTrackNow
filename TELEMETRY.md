# TELEMETRY (CanSat → Ground)

**Rate:** 1 Hz  
**Transport:** LoRa (SX1262)  
**Encoding:** Binary struct + CRC16-CCITT

## Packet: TelemetryPacket (msgType = 0x03)

| Field | Type | Units / Scale |
|---|---|---|
| msgType | uint8 | 0x03 |
| time_ms | uint32 | milliseconds since boot |
| lat_e7 | int32 | latitude * 1e7 |
| lon_e7 | int32 | longitude * 1e7 |
| height_agl_dm | int16 | (GNSS_alt_MSL - target_alt_MSL) * 10 |
| pressure_hPa_x10 | uint16 | pressure (hPa) * 10 |
| temp_c_x10 | int16 | temperature (°C) * 10 |
| ground_speed_cms | uint16 | ground speed (m/s) * 100 |
| servo_left_x1000 | int16 | command -1000..1000 |
| servo_right_x1000 | int16 | command -1000..1000 |
| pred_lat_e7 | int32 | predicted landing latitude * 1e7 |
| pred_lon_e7 | int32 | predicted landing longitude * 1e7 |
| mission_state | uint8 | state enum |
| crc16 | uint16 | CRC16-CCITT over all prior bytes |

## Incoming packets (Ground → CanSat)

### TargetPacket (msgType = 0x01)
Sent once before drop.

| Field | Type | Units / Scale |
|---|---|---|
| msgType | uint8 | 0x01 |
| tgt_lat_e7 | int32 | latitude * 1e7 |
| tgt_lon_e7 | int32 | longitude * 1e7 |
| tgt_alt_cm | int32 | target altitude MSL (m) * 100 |
| crc16 | uint16 | CRC16-CCITT |

### RTK (msgType = 0x02)
- Byte stream of RTCM3 corrections.
- Teensy **forwards raw bytes** to the GNSS UART.
