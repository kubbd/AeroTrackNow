"""
AeroTrackNow CanSat Ground Station (Advanced)
===========================================

How to run
----------
1) Install dependencies:
   pip install pyserial matplotlib numpy

2) Connect the Teensy ground radio node over USB.

3) Run the ground station:
   python3 ground_station_advanced.py \
     --port /dev/ttyACM0 \
     --target-lat 39.1234567 \
     --target-lon -104.9876543 \
     --target-alt 1850.0

Optional RTK passthrough:
   python3 ground_station_advanced.py --port /dev/ttyACM0 \
     --target-lat 39.1234567 --target-lon -104.9876543 --target-alt 1850.0 \
     --rtk-port /dev/ttyUSB0

Controls
--------
- Each plot/map is a separate window.
- Press 'l' in any window to jump back to live view (auto-scaling to latest data).
- Close any window to disable it; reopen via CLI commands (see console output).
"""

import argparse
import csv
import datetime
import os
import struct
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import serial


MSG_TARGET = 0x01
MSG_RTK = 0x02
MSG_TELEMETRY = 0x03

TELEMETRY_STRUCT = struct.Struct("<BIii h H h H h h ii B H")
TARGET_STRUCT_NO_CRC = struct.Struct("<Biii")
TARGET_STRUCT_WITH_CRC = struct.Struct("<BiiiH")

DIRECTION_RX = 0
DIRECTION_TX = 1

RAW_LOG_STRUCT = struct.Struct("<dBH")  # timestamp, direction, length


@dataclass
class TelemetrySample:
    recv_time: float
    time_s: float
    lat: float
    lon: float
    height_agl_m: float
    gnss_alt_msl: float
    pressure_hpa: float
    temp_c: float
    ground_speed_mps: float
    servo_left: int
    servo_right: int
    pred_lat: float
    pred_lon: float
    mission_state: int
    baro_alt_msl: float
    sink_rate_baro: float
    sink_rate_gnss: float


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) & 0xFFFF) ^ 0x1021
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


def pressure_to_altitude(pressure_hpa: float, sea_level_hpa: float) -> float:
    """
    Convert pressure to altitude using the barometric formula.

    Altitude = 44330 * (1 - (P / P0) ^ 0.1903)
    """
    return 44330.0 * (1.0 - (pressure_hpa / sea_level_hpa) ** 0.1903)


def sea_level_from_pressure(pressure_hpa: float, altitude_m: float) -> float:
    """Compute sea-level pressure based on a known altitude and pressure."""
    return pressure_hpa / (1.0 - altitude_m / 44330.0) ** 5.255


class TelemetryParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.packet_size = TELEMETRY_STRUCT.size

    def append(self, data: bytes) -> List[bytes]:
        self.buffer.extend(data)
        packets: List[bytes] = []
        while True:
            if len(self.buffer) < self.packet_size:
                break
            try:
                idx = self.buffer.index(MSG_TELEMETRY)
            except ValueError:
                self.buffer.clear()
                break
            if idx > 0:
                del self.buffer[:idx]
            if len(self.buffer) < self.packet_size:
                break
            candidate = bytes(self.buffer[:self.packet_size])
            calc = crc16_ccitt(candidate[:-2])
            recv_crc = struct.unpack_from("<H", candidate, self.packet_size - 2)[0]
            if calc == recv_crc:
                packets.append(candidate)
                del self.buffer[:self.packet_size]
            else:
                del self.buffer[0]
        return packets


class DataStore:
    def __init__(self, target_alt_msl: float) -> None:
        self.lock = threading.Lock()
        self.samples: List[TelemetrySample] = []
        self.target_alt_msl = target_alt_msl
        self.t0: Optional[float] = None
        self.sea_level_hpa: Optional[float] = None

    def add_sample(self, sample: TelemetrySample) -> None:
        with self.lock:
            self.samples.append(sample)

    def build_sample(self, recv_time: float, fields: Tuple) -> TelemetrySample:
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
            _crc,
        ) = fields

        time_s = time_ms / 1000.0
        if self.t0 is None:
            self.t0 = time_s
        time_rel = time_s - self.t0

        lat = lat_e7 * 1e-7
        lon = lon_e7 * 1e-7
        height_agl_m = height_agl_dm / 10.0
        gnss_alt_msl = self.target_alt_msl + height_agl_m
        pressure_hpa = pressure_hpa_x10 / 10.0
        temp_c = temp_c_x10 / 10.0
        ground_speed_mps = ground_speed_cms / 100.0
        pred_lat = pred_lat_e7 * 1e-7
        pred_lon = pred_lon_e7 * 1e-7

        if self.sea_level_hpa is None:
            # Calibrate baro altitude to the GNSS altitude for the first sample.
            self.sea_level_hpa = sea_level_from_pressure(pressure_hpa, gnss_alt_msl)

        baro_alt_msl = pressure_to_altitude(pressure_hpa, self.sea_level_hpa)

        sink_rate_baro = 0.0
        sink_rate_gnss = 0.0
        with self.lock:
            if self.samples:
                last = self.samples[-1]
                dt = time_rel - last.time_s
                if dt > 0:
                    # Sink rate is positive downward, so negate altitude rate.
                    sink_rate_baro = -(baro_alt_msl - last.baro_alt_msl) / dt
                    sink_rate_gnss = -(gnss_alt_msl - last.gnss_alt_msl) / dt

        return TelemetrySample(
            recv_time=recv_time,
            time_s=time_rel,
            lat=lat,
            lon=lon,
            height_agl_m=height_agl_m,
            gnss_alt_msl=gnss_alt_msl,
            pressure_hpa=pressure_hpa,
            temp_c=temp_c,
            ground_speed_mps=ground_speed_mps,
            servo_left=servo_left_x1000,
            servo_right=servo_right_x1000,
            pred_lat=pred_lat,
            pred_lon=pred_lon,
            mission_state=mission_state,
            baro_alt_msl=baro_alt_msl,
            sink_rate_baro=sink_rate_baro,
            sink_rate_gnss=sink_rate_gnss,
        )

    def snapshot(self) -> List[TelemetrySample]:
        with self.lock:
            return list(self.samples)


class RawLogger:
    def __init__(self, path: str) -> None:
        self.file = open(path, "ab")
        self.lock = threading.Lock()

    def log(self, direction: int, payload: bytes) -> None:
        with self.lock:
            header = RAW_LOG_STRUCT.pack(time.time(), direction, len(payload))
            self.file.write(header)
            self.file.write(payload)
            self.file.flush()

    def close(self) -> None:
        with self.lock:
            self.file.close()


class CsvLogger:
    def __init__(self, path: str) -> None:
        file_exists = os.path.exists(path)
        self.file = open(path, "a", newline="")
        self.writer = csv.writer(self.file)
        if not file_exists:
            self.writer.writerow(
                [
                    "recv_time",
                    "time_s",
                    "lat",
                    "lon",
                    "height_agl_m",
                    "gnss_alt_msl",
                    "baro_alt_msl",
                    "pressure_hpa",
                    "temp_c",
                    "ground_speed_mps",
                    "servo_left",
                    "servo_right",
                    "pred_lat",
                    "pred_lon",
                    "mission_state",
                    "sink_rate_baro",
                    "sink_rate_gnss",
                ]
            )
            self.file.flush()

    def log(self, sample: TelemetrySample) -> None:
        self.writer.writerow(
            [
                sample.recv_time,
                sample.time_s,
                sample.lat,
                sample.lon,
                sample.height_agl_m,
                sample.gnss_alt_msl,
                sample.baro_alt_msl,
                sample.pressure_hpa,
                sample.temp_c,
                sample.ground_speed_mps,
                sample.servo_left,
                sample.servo_right,
                sample.pred_lat,
                sample.pred_lon,
                sample.mission_state,
                sample.sink_rate_baro,
                sample.sink_rate_gnss,
            ]
        )
        self.file.flush()

    def close(self) -> None:
        self.file.close()


class PlotWindow:
    def __init__(self, title: str, time_window: float = 120.0) -> None:
        self.title = title
        self.time_window = time_window
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.manager.set_window_title(title)
        self.live_mode = True
        self._manual_interaction = False
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.canvas.mpl_connect("button_press_event", self._on_button)

    def _on_key(self, event) -> None:
        if event.key == "l":
            self.live_mode = True
            self._manual_interaction = False

    def _on_button(self, _event) -> None:
        self.live_mode = False
        self._manual_interaction = True

    def apply_live_xlim(self, times: List[float]) -> None:
        if not times:
            return
        tmax = times[-1]
        tmin = max(0.0, tmax - self.time_window)
        self.ax.set_xlim(tmin, tmax)


class Map3DWindow:
    def __init__(self, target: Tuple[float, float, float]) -> None:
        self.target_lat, self.target_lon, self.target_alt = target
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.fig.canvas.manager.set_window_title("3D Flight Map")
        self.live_mode = True
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.canvas.mpl_connect("button_press_event", self._on_button)

        self.trail_line, = self.ax.plot([], [], [], color="tab:blue", label="Flight trail")
        self.current_marker = self.ax.scatter([], [], [], color="tab:orange", label="Current")
        self.pred_marker = self.ax.scatter([], [], [], color="tab:green", label="Predicted landing")
        self.target_marker = self.ax.scatter(
            [self.target_lat],
            [self.target_lon],
            [self.target_alt],
            color="tab:red",
            label="Target",
        )
        self.pred_line, = self.ax.plot([], [], [], "k--", label="To predicted")
        self.ax.set_xlabel("Latitude")
        self.ax.set_ylabel("Longitude")
        self.ax.set_zlabel("Altitude (m)")
        self.ax.legend(loc="upper left")

    def _on_key(self, event) -> None:
        if event.key == "l":
            self.live_mode = True

    def _on_button(self, _event) -> None:
        self.live_mode = False

    def update(self, samples: List[TelemetrySample]) -> None:
        if not samples:
            return
        lats = [s.lat for s in samples]
        lons = [s.lon for s in samples]
        alts = [s.gnss_alt_msl for s in samples]
        self.trail_line.set_data(lats, lons)
        self.trail_line.set_3d_properties(alts)

        current = samples[-1]
        self.current_marker._offsets3d = ([current.lat], [current.lon], [current.gnss_alt_msl])

        pred_alt = self.target_alt
        self.pred_marker._offsets3d = ([current.pred_lat], [current.pred_lon], [pred_alt])

        self.pred_line.set_data([current.lat, current.pred_lat], [current.lon, current.pred_lon])
        self.pred_line.set_3d_properties([current.gnss_alt_msl, pred_alt])

        if self.live_mode:
            self.ax.relim()
            self.ax.autoscale_view()


class Map2DWindow:
    def __init__(self, target: Tuple[float, float]) -> None:
        self.target_lat, self.target_lon = target
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.manager.set_window_title("2D Flight Map")
        self.live_mode = True
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.canvas.mpl_connect("button_press_event", self._on_button)

        self.trail_line, = self.ax.plot([], [], color="tab:blue", label="Flight trail")
        self.current_marker = self.ax.scatter([], [], color="tab:orange", label="Current")
        self.pred_marker = self.ax.scatter([], [], color="tab:green", label="Predicted landing")
        self.target_marker = self.ax.scatter(
            [self.target_lon],
            [self.target_lat],
            color="tab:red",
            label="Target",
        )
        self.pred_line, = self.ax.plot([], [], "k--", label="To predicted")
        self.ax.set_xlabel("Longitude")
        self.ax.set_ylabel("Latitude")
        self.ax.legend(loc="upper left")

    def _on_key(self, event) -> None:
        if event.key == "l":
            self.live_mode = True

    def _on_button(self, _event) -> None:
        self.live_mode = False

    def update(self, samples: List[TelemetrySample]) -> None:
        if not samples:
            return
        lats = [s.lat for s in samples]
        lons = [s.lon for s in samples]
        self.trail_line.set_data(lons, lats)

        current = samples[-1]
        self.current_marker.set_offsets([[current.lon, current.lat]])
        self.pred_marker.set_offsets([[current.pred_lon, current.pred_lat]])
        self.pred_line.set_data([current.lon, current.pred_lon], [current.lat, current.pred_lat])

        if self.live_mode:
            self.ax.relim()
            self.ax.autoscale_view()


class PlotManager:
    def __init__(self, data_store: DataStore, target: Tuple[float, float, float]) -> None:
        self.data_store = data_store
        self.target = target
        self.animations: List[FuncAnimation] = []
        self.windows: Dict[str, object] = {}

    def _register_window(self, name: str, window: object, anim: FuncAnimation) -> None:
        self.animations.append(anim)
        self.windows[name] = window
        window.fig.canvas.mpl_connect(
            "close_event", lambda _event: self.windows.pop(name, None)
        )

    def open_sink_rate(self) -> None:
        if "sink" in self.windows:
            return
        window = PlotWindow("Sink Rate Comparison")
        line_baro, = window.ax.plot([], [], label="Baro sink rate", color="tab:blue")
        line_gnss, = window.ax.plot([], [], label="GNSS sink rate", color="tab:orange")
        window.ax.set_xlabel("Time (s)")
        window.ax.set_ylabel("Sink rate (m/s, +down)")
        window.ax.legend(loc="upper right")

        def update(_frame):
            samples = self.data_store.snapshot()
            if not samples:
                return line_baro, line_gnss
            times = [s.time_s for s in samples]
            baro = [s.sink_rate_baro for s in samples]
            gnss = [s.sink_rate_gnss for s in samples]
            line_baro.set_data(times, baro)
            line_gnss.set_data(times, gnss)
            if window.live_mode:
                window.apply_live_xlim(times)
                window.ax.relim()
                window.ax.autoscale_view()
            return line_baro, line_gnss

        anim = FuncAnimation(window.fig, update, interval=500)
        self._register_window("sink", window, anim)

    def open_temperature(self) -> None:
        if "temp" in self.windows:
            return
        window = PlotWindow("Temperature")
        line_temp, = window.ax.plot([], [], label="BMP280 temp", color="tab:red")
        window.ax.set_xlabel("Time (s)")
        window.ax.set_ylabel("Temperature (°C)")
        window.ax.legend(loc="upper right")

        def update(_frame):
            samples = self.data_store.snapshot()
            if not samples:
                return line_temp
            times = [s.time_s for s in samples]
            temps = [s.temp_c for s in samples]
            line_temp.set_data(times, temps)
            if window.live_mode:
                window.apply_live_xlim(times)
                window.ax.relim()
                window.ax.autoscale_view()
            return line_temp

        anim = FuncAnimation(window.fig, update, interval=500)
        self._register_window("temp", window, anim)

    def open_altitude(self) -> None:
        if "alt" in self.windows:
            return
        window = PlotWindow("Altitude Comparison")
        line_baro, = window.ax.plot([], [], label="Baro altitude", color="tab:blue")
        line_gnss, = window.ax.plot([], [], label="GNSS altitude", color="tab:green")
        window.ax.set_xlabel("Time (s)")
        window.ax.set_ylabel("Altitude (m MSL)")
        window.ax.legend(loc="upper right")

        def update(_frame):
            samples = self.data_store.snapshot()
            if not samples:
                return line_baro, line_gnss
            times = [s.time_s for s in samples]
            baro = [s.baro_alt_msl for s in samples]
            gnss = [s.gnss_alt_msl for s in samples]
            line_baro.set_data(times, baro)
            line_gnss.set_data(times, gnss)
            if window.live_mode:
                window.apply_live_xlim(times)
                window.ax.relim()
                window.ax.autoscale_view()
            return line_baro, line_gnss

        anim = FuncAnimation(window.fig, update, interval=500)
        self._register_window("alt", window, anim)

    def open_map3d(self) -> None:
        if "map3d" in self.windows:
            return
        window = Map3DWindow(self.target)

        def update(_frame):
            samples = self.data_store.snapshot()
            window.update(samples)

        anim = FuncAnimation(window.fig, update, interval=500)
        self._register_window("map3d", window, anim)

    def open_map2d(self) -> None:
        if "map2d" in self.windows:
            return
        window = Map2DWindow((self.target[0], self.target[1]))

        def update(_frame):
            samples = self.data_store.snapshot()
            window.update(samples)

        anim = FuncAnimation(window.fig, update, interval=500)
        self._register_window("map2d", window, anim)

    def close(self, name: str) -> None:
        window = self.windows.pop(name, None)
        if window:
            plt.close(window.fig)


def build_target_packet(lat: float, lon: float, alt_msl: float) -> bytes:
    lat_e7 = int(round(lat * 1e7))
    lon_e7 = int(round(lon * 1e7))
    alt_cm = int(round(alt_msl * 100))
    payload = TARGET_STRUCT_NO_CRC.pack(MSG_TARGET, lat_e7, lon_e7, alt_cm)
    crc = crc16_ccitt(payload)
    return TARGET_STRUCT_WITH_CRC.pack(MSG_TARGET, lat_e7, lon_e7, alt_cm, crc)


def command_loop(manager: PlotManager, stop_event: threading.Event) -> None:
    print(
        "Commands: open sink|temp|alt|map3d|map2d, close sink|temp|alt|map3d|map2d, live, quit"
    )
    while not stop_event.is_set():
        try:
            cmd = input("command> ").strip().lower()
        except EOFError:
            break
        if cmd in {"quit", "exit"}:
            stop_event.set()
            break
        if cmd == "live":
            for window in manager.windows.values():
                window.live_mode = True
            print("Live mode enabled on all windows.")
            continue
        if cmd.startswith("open "):
            name = cmd.split(" ", 1)[1]
            if name == "sink":
                manager.open_sink_rate()
            elif name == "temp":
                manager.open_temperature()
            elif name == "alt":
                manager.open_altitude()
            elif name == "map3d":
                manager.open_map3d()
            elif name == "map2d":
                manager.open_map2d()
            else:
                print("Unknown window.")
            continue
        if cmd.startswith("close "):
            name = cmd.split(" ", 1)[1]
            manager.close(name)
            continue
        print("Unknown command.")


def serial_reader(
    ser: serial.Serial,
    parser: TelemetryParser,
    data_store: DataStore,
    csv_logger: CsvLogger,
    raw_logger: RawLogger,
    stop_event: threading.Event,
) -> None:
    while not stop_event.is_set():
        data = ser.read(256)
        if not data:
            continue
        for packet in parser.append(data):
            raw_logger.log(DIRECTION_RX, packet)
            fields = TELEMETRY_STRUCT.unpack(packet)
            sample = data_store.build_sample(time.time(), fields)
            data_store.add_sample(sample)
            csv_logger.log(sample)


def rtk_reader(
    rtk_ser: serial.Serial,
    radio_ser: serial.Serial,
    raw_logger: RawLogger,
    stop_event: threading.Event,
    chunk_size: int = 180,
) -> None:
    while not stop_event.is_set():
        data = rtk_ser.read(chunk_size)
        if not data:
            continue
        payload = bytes([MSG_RTK]) + data
        radio_ser.write(payload)
        raw_logger.log(DIRECTION_TX, payload)


def main() -> None:
    parser = argparse.ArgumentParser(description="AeroTrackNow Advanced Ground Station")
    parser.add_argument("--port", required=True, help="USB serial port for the ground radio node")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default 115200)")
    parser.add_argument("--target-lat", type=float, required=True, help="Target latitude (deg)")
    parser.add_argument("--target-lon", type=float, required=True, help="Target longitude (deg)")
    parser.add_argument("--target-alt", type=float, required=True, help="Target altitude MSL (m)")
    parser.add_argument("--rtk-port", help="Optional RTK input serial port")
    parser.add_argument("--rtk-baud", type=int, default=115200, help="RTK serial baud (default 115200)")
    args = parser.parse_args()

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = os.path.join(os.getcwd(), "logs", timestamp)
    os.makedirs(log_dir, exist_ok=True)
    csv_path = os.path.join(log_dir, "telemetry.csv")
    raw_path = os.path.join(log_dir, "raw_packets.bin")

    csv_logger = CsvLogger(csv_path)
    raw_logger = RawLogger(raw_path)

    data_store = DataStore(target_alt_msl=args.target_alt)
    parser_state = TelemetryParser()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)

    target_packet = build_target_packet(args.target_lat, args.target_lon, args.target_alt)
    ser.write(target_packet)
    raw_logger.log(DIRECTION_TX, target_packet)
    print("Sent TARGET packet.")

    manager = PlotManager(data_store, (args.target_lat, args.target_lon, args.target_alt))
    manager.open_sink_rate()
    manager.open_temperature()
    manager.open_altitude()
    manager.open_map3d()

    stop_event = threading.Event()
    reader_thread = threading.Thread(
        target=serial_reader,
        args=(ser, parser_state, data_store, csv_logger, raw_logger, stop_event),
        daemon=True,
    )
    reader_thread.start()

    rtk_thread = None
    if args.rtk_port:
        rtk_ser = serial.Serial(args.rtk_port, args.rtk_baud, timeout=0.1)
        rtk_thread = threading.Thread(
            target=rtk_reader,
            args=(rtk_ser, ser, raw_logger, stop_event),
            daemon=True,
        )
        rtk_thread.start()
        print("RTK passthrough enabled.")

    cmd_thread = threading.Thread(target=command_loop, args=(manager, stop_event), daemon=True)
    cmd_thread.start()

    try:
        plt.show()
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        stop_event.set()
        csv_logger.close()
        raw_logger.close()
        ser.close()
        if args.rtk_port:
            rtk_ser.close()


if __name__ == "__main__":
    main()
