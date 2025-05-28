import serial
import struct
import time
import threading
import queue
from typing import Optional, Dict, Any, List, Tuple

class LidarCommunicationError(Exception):
    pass

class LidarTimeoutError(LidarCommunicationError):
    pass

class LidarPacketError(ValueError):
    pass


class LidarAPI:
    HEADER = 0x54
    POINTS_PER_PACKET = 12
    PACKET_SIZE = 47  # 1 byte header + 46 bytes payload

    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 230400,
                 serial_timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.serial_timeout = serial_timeout
        self.ser: Optional[serial.Serial] = None
        self._read_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._latest_packet_queue: "queue.Queue[Tuple[int, Dict[str, Any]]]" \
            = queue.Queue(maxsize=1)
        self._is_connected = False
        self._lock = threading.Lock()

    # ────────────────────────────────────────────────────────────── connection ──
    def connect(self) -> None:
        with self._lock:
            if self._is_connected:
                return
            try:
                self.ser = serial.Serial(self.port, self.baudrate,
                                         timeout=self.serial_timeout)
                time.sleep(0.1)          # let the device settle
                self.ser.reset_input_buffer()
                self._stop_event.clear()
                self._read_thread = threading.Thread(
                    target=self._read_loop, daemon=True
                )
                self._read_thread.start()
                self._is_connected = True
            except serial.SerialException as e:
                self.ser = None
                raise LidarCommunicationError(
                    f"Failed to open LiDAR serial port {self.port}: {e}"
                )

    def disconnect(self) -> None:
        with self._lock:
            if not self._is_connected:
                return
            self._stop_event.set()
            if self._read_thread and self._read_thread.is_alive():
                self._read_thread.join(timeout=self.serial_timeout + 0.5)
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                finally:
                    pass
            self.ser = None
            self._read_thread = None
            self._is_connected = False
            while not self._latest_packet_queue.empty():
                try:
                    self._latest_packet_queue.get_nowait()
                except queue.Empty:
                    break

    # ───────────────────────────────────────────────────────────── packet fetch ─
    def get_latest_packet(self) -> Optional[Tuple[int, Dict[str, Any]]]:
        """
        POP (not peek) the most recent packet from the queue.

        Returns (pi_timestamp_ns, parsed_packet_dict) or None if none available.
        """
        if not self._is_connected:
            return None
        try:
            return self._latest_packet_queue.get_nowait()
        except queue.Empty:
            return None

    # ──────────────────────────────────────────────────────────── reader thread ─
    def _read_loop(self) -> None:
        buffer = bytearray()
        while not self._stop_event.is_set():
            try:
                if not self.ser or not self.ser.is_open:
                    break
                data = self.ser.read(max(1, min(2048, self.ser.in_waiting)))
                if not data:
                    time.sleep(0.001)
                    continue
                buffer.extend(data)

                while len(buffer) >= self.PACKET_SIZE:
                    header_index = buffer.find(self.HEADER)
                    if header_index == -1:
                        buffer = buffer[-(self.PACKET_SIZE - 1):]
                        break
                    if header_index:
                        buffer = buffer[header_index:]

                    if len(buffer) < self.PACKET_SIZE:
                        break

                    packet = bytes(buffer[:self.PACKET_SIZE])
                    pi_ts = time.time_ns()
                    try:
                        parsed = self._parse_packet(packet)
                        if parsed:
                            if not self._latest_packet_queue.empty():
                                try:
                                    self._latest_packet_queue.get_nowait()
                                except queue.Empty:
                                    pass
                            self._latest_packet_queue.put((pi_ts, parsed))
                        buffer = buffer[self.PACKET_SIZE:]
                    except LidarPacketError:
                        buffer = buffer[1:]
            except serial.SerialException:
                break
            except Exception:
                time.sleep(0.1)
        with self._lock:
            self._is_connected = False

    # ────────────────────────────────────────────────────────────── packet parse ─
    @staticmethod
    def _parse_packet(packet: bytes) -> Dict[str, Any]:
        if len(packet) != LidarAPI.PACKET_SIZE or packet[0] != LidarAPI.HEADER:
            raise LidarPacketError("Bad packet header/length")

        speed = struct.unpack('<H', packet[2:4])[0] / 100.0
        start_angle = struct.unpack('<H', packet[4:6])[0] / 100.0
        end_angle = struct.unpack('<H', packet[42:44])[0] / 100.0
        sensor_ts_sec = struct.unpack('<H', packet[44:46])[0] / 1000.0

        points: List[Dict[str, Any]] = []
        for i in range(LidarAPI.POINTS_PER_PACKET):
            off = 6 + i * 3
            distance = struct.unpack('<H', packet[off:off + 2])[0] / 1000.0
            intensity = packet[off + 2]
            points.append({'distance': distance, 'intensity': intensity})

        if end_angle < start_angle:
            end_angle += 360.0
        angle_step = (
            (end_angle - start_angle) / (LidarAPI.POINTS_PER_PACKET - 1)
            if LidarAPI.POINTS_PER_PACKET > 1 else 0.0
        )
        for i, p in enumerate(points):
            p['angle'] = (start_angle + i * angle_step) % 360.0

        return {
            'speed': speed,
            'start_angle': start_angle,
            'end_angle': (end_angle if end_angle < 360.0 else end_angle - 360.0),
            'sensor_timestamp': sensor_ts_sec,
            'points': points
        }
