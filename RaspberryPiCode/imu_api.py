import serial
import threading
import time
import json
from typing import Optional, Dict, Any

class IMUAPI:
    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 115200, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self.thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._latest: Optional[Dict[str, Any]] = None
        self._lock = threading.Lock()

    def connect(self):
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        time.sleep(0.1)  # allow settle
        self.ser.reset_input_buffer()
        self._stop_event.clear()
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def disconnect(self):
        self._stop_event.set()
        if self.thread:
            self.thread.join()
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None
        self.thread = None

    def _read_loop(self):
        while not self._stop_event.is_set():
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line.startswith('{'):
                    continue  # Skip non-JSON lines

                parsed = json.loads(line)

                # Skip if it's just an error message
                if "error" in parsed:
                    continue

                with self._lock:
                    self._latest = parsed

            except json.JSONDecodeError:
                continue
            except Exception as e:
                print(f"IMU read error: {e}")
                time.sleep(0.1)

    def get_latest_packet(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            return self._latest.copy() if self._latest else None

if __name__ == '__main__':
    imu = IMUAPI('/dev/ttyACM0')
    imu.connect()

    try:
        print("Collecting IMU data...")
        for _ in range(10):
            pkt = imu.get_latest_packet()
            if pkt:
                print(json.dumps(pkt, indent=2))
            time.sleep(0.2)
    finally:
        imu.disconnect()
        print("Disconnected.")