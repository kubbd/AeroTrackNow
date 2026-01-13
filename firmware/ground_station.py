#!/usr/bin/env python3
"""CanSat ground station for AeroTrackNow.

Reads telemetry packets from a Teensy over USB serial, validates CRC16-CCITT,
decodes fields using the documented scaling, displays key fields, and sends
TARGET/RTK packets back to the Teensy.
"""

import argparse
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass

import serial

MSG_TARGET = 0x01
MSG_RTK = 0x02
MSG_TELEMETRY = 0x03

TELEMETRY_STRUCT = struct.Struct("<BIiihHhHhhi iBH".replace(" ", ""))
TARGET_STRUCT = struct.Struct("<BiiiH")

TELEMETRY_SIZE = TELEMETRY_STRUCT.size

MISSION_STATES = {
    0: "BOOT",
    1: "WAIT_FOR_DROP",
    2: "WAIT_FOR_STABLE_DESCENT",
    3: "GUIDED_DESCENT",
    4: "TERMINAL",
    5: "LANDED",
}

RTK_MAX_PAYLOAD = 240  # keep well under LoRa payload limits


@dataclass
class Telemetry:
    time_ms: int
    lat_deg: float
    lon_deg: float
    height_agl_m: float
    pressure_hpa: float
    temp_c: float
    ground_speed_mps: float
    servo_left: float
    servo_right: float
    pred_lat_deg: float
    pred_lon_deg: float
    mission_state: int


def crc16_ccitt(data: bytes) -> int:
    """Compute CRC16-CCITT (poly 0x1021, init 0xFFFF)"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def build_target_packet(lat_deg: float, lon_deg: float, alt_m_msl: float) -> bytes:
    """Build TARGET packet exactly per docs/TELEMETRY.md."""
    lat_e7 = int(round(lat_deg * 1e7))
    lon_e7 = int(round(lon_deg * 1e7))
    alt_cm = int(round(alt_m_msl * 100.0))
    payload = TARGET_STRUCT.pack(MSG_TARGET, lat_e7, lon_e7, alt_cm, 0)
    crc = crc16_ccitt(payload[:-2])
    return payload[:-2] + struct.pack("<H", crc)


def decode_telemetry(packet: bytes) -> Telemetry:
    """Decode a validated telemetry packet into engineering units."""
    (
        _msg_type,
        time_ms,
        lat_e7,
        lon_e7,
        height_agl_dm,
        pressure_hpa_x10,
        temp_c_x10,
        ground_speed_cms,
        servo_left_x1000,
        servo_right_x1000,
        pred_lat_e7,
        pred_lon_e7,
        mission_state,
        _crc16,
    ) = TELEMETRY_STRUCT.unpack(packet)

    return Telemetry(
        time_ms=time_ms,
        lat_deg=lat_e7 * 1e-7,
        lon_deg=lon_e7 * 1e-7,
        height_agl_m=height_agl_dm / 10.0,
        pressure_hpa=pressure_hpa_x10 / 10.0,
        temp_c=temp_c_x10 / 10.0,
        ground_speed_mps=ground_speed_cms / 100.0,
        servo_left=servo_left_x1000 / 1000.0,
        servo_right=servo_right_x1000 / 1000.0,
        pred_lat_deg=pred_lat_e7 * 1e-7,
        pred_lon_deg=pred_lon_e7 * 1e-7,
        mission_state=mission_state,
    )


def display_telemetry(tlm: Telemetry) -> None:
    """Print key telemetry fields to the CLI."""
    state_name = MISSION_STATES.get(tlm.mission_state, f"UNKNOWN({tlm.mission_state})")
    print(
        "\n".join(
            [
                f"Time: {tlm.time_ms} ms",
                f"Current Lat/Lon: {tlm.lat_deg:.7f}, {tlm.lon_deg:.7f}",
                f"Predicted Lat/Lon: {tlm.pred_lat_deg:.7f}, {tlm.pred_lon_deg:.7f}",
                f"Height AGL: {tlm.height_agl_m:.1f} m",
                f"Ground Speed: {tlm.ground_speed_mps:.2f} m/s",
                f"Mission State: {state_name}",
            ]
        )
        + "\n"
    )


def telemetry_reader(port: serial.Serial, stop_event: threading.Event) -> None:
    """Continuously read telemetry packets from the serial stream."""
    buffer = bytearray()
    while not stop_event.is_set():
        chunk = port.read(128)
        if not chunk:
            continue
        buffer.extend(chunk)

        while True:
            if len(buffer) < 1:
                break
            try:
                start = buffer.index(MSG_TELEMETRY)
            except ValueError:
                buffer.clear()
                break

            if len(buffer) - start < TELEMETRY_SIZE:
                if start > 0:
                    del buffer[:start]
                break

            packet = bytes(buffer[start : start + TELEMETRY_SIZE])
            crc_calc = crc16_ccitt(packet[:-2])
            crc_recv = struct.unpack("<H", packet[-2:])[0]

            if crc_calc == crc_recv:
                telemetry = decode_telemetry(packet)
                display_telemetry(telemetry)
                del buffer[: start + TELEMETRY_SIZE]
            else:
                del buffer[: start + 1]


def send_rtk_stream(
    port: serial.Serial,
    stop_event: threading.Event,
    source: str,
    lock: threading.Lock,
) -> None:
    """Read raw RTCM bytes from a file or socket and forward as MSG_RTK packets."""
    if source.startswith("file:"):
        path = source[len("file:") :]
        with open(path, "rb") as handle:
            while not stop_event.is_set():
                data = handle.read(RTK_MAX_PAYLOAD)
                if not data:
                    break
                packet = bytes([MSG_RTK]) + data
                with lock:
                    port.write(packet)
    elif source.startswith("tcp:"):
        host_port = source[len("tcp:") :]
        host, port_str = host_port.rsplit(":", 1)
        with socket.create_connection((host, int(port_str))) as sock:
            sock.settimeout(1.0)
            while not stop_event.is_set():
                try:
                    data = sock.recv(RTK_MAX_PAYLOAD)
                except socket.timeout:
                    continue
                if not data:
                    break
                packet = bytes([MSG_RTK]) + data
                with lock:
                    port.write(packet)
    else:
        raise ValueError(f"Unsupported RTK source: {source}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="AeroTrackNow CanSat ground station")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--target-lat", type=float, required=True, help="Target latitude (deg)")
    parser.add_argument("--target-lon", type=float, required=True, help="Target longitude (deg)")
    parser.add_argument("--target-alt", type=float, required=True, help="Target altitude MSL (m)")
    parser.add_argument(
        "--rtk-file",
        help="Path to RTCM3 file to forward (mutually exclusive with --rtk-host)",
    )
    parser.add_argument(
        "--rtk-host",
        help="RTK TCP host (mutually exclusive with --rtk-file)",
    )
    parser.add_argument("--rtk-port", type=int, help="RTK TCP port")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.rtk_file and args.rtk_host:
        print("--rtk-file and --rtk-host are mutually exclusive", file=sys.stderr)
        return 2
    if args.rtk_host and not args.rtk_port:
        print("--rtk-port is required when using --rtk-host", file=sys.stderr)
        return 2

    with serial.Serial(args.port, args.baud, timeout=0.5) as ser:
        time.sleep(1.0)  # allow port to settle
        target_packet = build_target_packet(args.target_lat, args.target_lon, args.target_alt)
        lock = threading.Lock()
        with lock:
            ser.write(target_packet)
        print(
            f"Sent TARGET packet: lat={args.target_lat:.7f}, lon={args.target_lon:.7f}, "
            f"alt={args.target_alt:.2f} m"
        )

        stop_event = threading.Event()
        threads = []

        if args.rtk_file:
            rtk_source = f"file:{args.rtk_file}"
            thread = threading.Thread(
                target=send_rtk_stream,
                args=(ser, stop_event, rtk_source, lock),
                daemon=True,
            )
            thread.start()
            threads.append(thread)
        elif args.rtk_host:
            rtk_source = f"tcp:{args.rtk_host}:{args.rtk_port}"
            thread = threading.Thread(
                target=send_rtk_stream,
                args=(ser, stop_event, rtk_source, lock),
                daemon=True,
            )
            thread.start()
            threads.append(thread)

        try:
            telemetry_reader(ser, stop_event)
        except KeyboardInterrupt:
            print("Shutting down...")
        finally:
            stop_event.set()
            for thread in threads:
                thread.join(timeout=1.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
