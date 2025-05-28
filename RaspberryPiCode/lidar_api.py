import serial
import struct
import time
import threading
import queue
from typing import Optional, Dict, Any, List, Tuple

# Define custom exceptions for LiDAR API errors
class LidarCommunicationError(Exception):
    """Custom exception for general serial communication problems with LiDAR."""
    pass

class LidarTimeoutError(LidarCommunicationError):
    """Custom exception for timeouts during LiDAR communication."""
    pass

class LidarPacketError(ValueError):
    """Custom exception for errors parsing LiDAR packets."""
    pass


class LidarAPI:
    """
    API for interfacing with the 2D LiDAR sensor.

    Handles connection, data reading, packet parsing, and provides access
    to the latest data packet with associated host timestamp. Runs the
    serial reading in a background thread.
    """
    HEADER = 0x54
    POINTS_PER_PACKET = 12
    PACKET_SIZE = 47 # Header (1) + Data (46)

    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 230400, serial_timeout: float = 1.0):
        """
        Initialize LiDAR API.

        Args:
            port: Serial port where the LiDAR is connected.
            baudrate: Communication speed for the LiDAR.
            serial_timeout: Timeout in seconds for serial read operations.
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_timeout = serial_timeout
        self.ser: Optional[serial.Serial] = None
        self._read_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        # Use a thread-safe queue to store the latest packet info
        # Stores tuples: (pi_timestamp_ns, parsed_packet_dict)
        self._latest_packet_queue = queue.Queue(maxsize=1)
        self._is_connected = False
        self._lock = threading.Lock() # Lock for accessing shared resources if needed

        print(f"DEBUG: LidarAPI initialized for port {port} at {baudrate} baud.")

    def connect(self) -> None:
        """
        Establish serial connection and start the background reading thread.
        """
        with self._lock:
            if self._is_connected:
                print("DEBUG: LidarAPI already connected.")
                return

            print(f"DEBUG: LidarAPI attempting to connect to {self.port}...")
            try:
                self.ser = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=self.serial_timeout
                )
                # Short delay to allow device to settle? Depends on LiDAR model.
                time.sleep(0.1)
                self.ser.reset_input_buffer() # Clear any old data
                print(f"DEBUG: LidarAPI serial port {self.port} opened.")

                self._stop_event.clear()
                self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
                self._read_thread.start()
                self._is_connected = True
                print("DEBUG: LidarAPI background reading thread started.")

            except serial.SerialException as e:
                self.ser = None
                self._is_connected = False
                print(f"DEBUG: LidarAPI failed to open serial port {self.port}: {e}")
                raise LidarCommunicationError(f"Failed to open LiDAR serial port {self.port}: {e}")
            except Exception as e:
                self.ser = None
                self._is_connected = False
                print(f"DEBUG: LidarAPI unexpected error during connect: {e}")
                raise LidarCommunicationError(f"Unexpected error connecting to LiDAR: {e}")

    def disconnect(self) -> None:
        """
        Stop the background thread and close the serial connection.
        """
        with self._lock:
            if not self._is_connected:
                print("DEBUG: LidarAPI already disconnected.")
                return

            print("DEBUG: LidarAPI disconnecting...")
            self._stop_event.set() # Signal the thread to stop

            if self._read_thread and self._read_thread.is_alive():
                print("DEBUG: LidarAPI waiting for read thread to join...")
                self._read_thread.join(timeout=self.serial_timeout + 0.5) # Wait for thread
                if self._read_thread.is_alive():
                    print("DEBUG: LidarAPI read thread did not join cleanly.")

            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                    print(f"DEBUG: LidarAPI serial port {self.port} closed.")
                except Exception as e:
                    print(f"DEBUG: LidarAPI error closing serial port: {e}")

            self.ser = None
            self._read_thread = None
            self._is_connected = False
            # Clear the queue on disconnect
            while not self._latest_packet_queue.empty():
                try:
                    self._latest_packet_queue.get_nowait()
                except queue.Empty:
                    break
            print("DEBUG: LidarAPI disconnected.")

    def get_latest_packet(self) -> Optional[Tuple[int, Dict[str, Any]]]:
        """
        Get the most recently received and parsed LiDAR packet data.

        Returns:
            A tuple containing (pi_timestamp_ns, parsed_packet_dict) if a packet
            is available, otherwise None. The timestamp is captured when the
            packet is fully read.
            Returns None if not connected or no packet received yet.
        """
        if not self._is_connected:
            return None
        try:
            # Get the item from the queue without blocking
            # The queue stores the latest item, overwriting older ones if full (maxsize=1)
            # We use peek/get_nowait semantics here
            latest = self._latest_packet_queue.queue[0] # Peek if possible (not thread safe without lock, but queue handles it)
            return latest
        except IndexError: # Queue is empty
             return None
        except Exception as e:
             print(f"DEBUG: Error getting latest packet from queue: {e}")
             return None


    def _read_loop(self) -> None:
        """
        Background thread function to continuously read and parse LiDAR packets.
        """
        print("DEBUG: LidarAPI read loop started.")
        buffer = bytearray()
        while not self._stop_event.is_set():
            try:
                if not self.ser or not self.ser.is_open:
                    print("DEBUG: LidarAPI read loop: Serial port closed unexpectedly. Stopping.")
                    break # Exit loop if serial port closed

                # Read available bytes
                bytes_to_read = max(1, min(2048, self.ser.in_waiting))
                data = self.ser.read(bytes_to_read)

                if not data:
                    # Timeout occurred, or nothing read, continue loop
                    time.sleep(0.001) # Small sleep to prevent busy-waiting
                    continue

                buffer.extend(data)

                # Process buffer to find packets
                while len(buffer) >= self.PACKET_SIZE:
                    # Find the header byte
                    header_index = buffer.find(self.HEADER)
                    if header_index == -1:
                        # No header found, discard buffer (or keep partial end?)
                        # Discarding is simpler but might lose data if header is split
                        # Keep last PACKET_SIZE-1 bytes in case header is split
                        buffer = buffer[-(self.PACKET_SIZE - 1):]
                        break

                    if header_index > 0:
                        # Discard bytes before the header
                        print(f"DEBUG: LidarAPI discarding {header_index} bytes before header.")
                        buffer = buffer[header_index:]

                    # Check if we have a full packet starting from the header
                    if len(buffer) < self.PACKET_SIZE:
                        # Not enough data for a full packet yet, wait for more
                        break

                    # Potential packet found
                    potential_packet = bytes(buffer[:self.PACKET_SIZE])
                    pi_timestamp_ns = time.time_ns() # Timestamp when packet is fully formed in buffer

                    # Basic check (Header byte is already confirmed)
                    # Add CRC check here if needed for robustness

                    # Attempt to parse
                    try:
                        parsed_data = self._parse_packet(potential_packet)
                        if parsed_data:
                            # Successfully parsed, update the latest packet queue
                            # Put item, potentially blocking if queue was full (shouldn't happen with maxsize=1 and get logic)
                            # Use put_nowait or clear queue first for non-blocking update
                            if not self._latest_packet_queue.empty():
                                try:
                                    self._latest_packet_queue.get_nowait() # Clear previous item
                                except queue.Empty:
                                    pass # Already empty, race condition?
                            self._latest_packet_queue.put((pi_timestamp_ns, parsed_data))
                            # print(f"DEBUG: LidarAPI parsed packet. Queue size: {self._latest_packet_queue.qsize()}") # Debug queue

                        # Consume the processed packet from the buffer
                        buffer = buffer[self.PACKET_SIZE:]

                    except LidarPacketError as e:
                        print(f"DEBUG: LidarAPI packet error: {e}. Discarding header byte and retrying.")
                        # Packet was invalid, discard the header byte and try finding the next header
                        buffer = buffer[1:]
                    except Exception as e:
                         print(f"DEBUG: LidarAPI unexpected parsing error: {e}. Discarding header byte.")
                         buffer = buffer[1:] # Move past the problematic header

            except serial.SerialException as e:
                print(f"ERROR: LidarAPI serial error in read loop: {e}. Stopping thread.")
                self._stop_event.set() # Signal stop on serial error
                break
            except Exception as e:
                print(f"ERROR: LidarAPI unexpected error in read loop: {e}")
                # Decide whether to stop or continue
                time.sleep(0.1) # Pause briefly after unexpected error

        print("DEBUG: LidarAPI read loop finished.")
        # Ensure connection status is updated if loop exits unexpectedly
        with self._lock:
            self._is_connected = False


    @staticmethod
    def _parse_packet(packet: bytes) -> Optional[Dict[str, Any]]:
        """
        Parse the raw byte packet data. (Static method for potential reuse)

        Args:
            packet: The raw bytes of a potential LiDAR packet (47 bytes).

        Returns:
            A dictionary containing parsed data, or None if parsing fails.

        Raises:
            LidarPacketError: If the packet is invalid (e.g., wrong header, bad values).
        """
        if len(packet) != LidarAPI.PACKET_SIZE:
             raise LidarPacketError(f"Invalid packet size: {len(packet)}, expected {LidarAPI.PACKET_SIZE}")
        if packet[0] != LidarAPI.HEADER:
            raise LidarPacketError("Invalid header byte.")

        # Add CRC check here if the protocol includes it and it's needed

        try:
            # Unpack data according to the protocol structure
            # Example structure based on read_lidar.py (adjust if different)
            # ver_len = packet[1] # Often contains version/length info
            speed = struct.unpack('<H', packet[2:4])[0] / 100.0  # Speed in degrees/sec
            start_angle = struct.unpack('<H', packet[4:6])[0] / 100.0  # Start angle in degrees
            end_angle = struct.unpack('<H', packet[42:44])[0] / 100.0 # End angle in degrees
            # Timestamp from LiDAR packet (interpretation depends on device)
            # Assuming it's seconds relative to some epoch or power-on
            sensor_timestamp = struct.unpack('<H', packet[44:46])[0] # Original code divides by 1000.0 - keep raw or convert? Let's keep raw for now, convert later if needed.
                                                                       # Revisit: The original code divides by 1000.0, let's stick to that for consistency.
            sensor_timestamp_sec = struct.unpack('<H', packet[44:46])[0] / 1000.0 # Timestamp in seconds

            # Extract points
            points_data = []
            for i in range(LidarAPI.POINTS_PER_PACKET):
                offset = 6 + i * 3
                distance = struct.unpack('<H', packet[offset:offset + 2])[0] / 1000.0 # Distance in meters
                intensity = packet[offset + 2] # Signal intensity
                points_data.append({'distance': distance, 'intensity': intensity})

            # Interpolate angles
            start_angle_calc = start_angle
            end_angle_calc = end_angle
            if end_angle_calc < start_angle_calc:
                end_angle_calc += 360.0 # Handle wrap-around

            # Avoid division by zero if POINTS_PER_PACKET is 1
            if LidarAPI.POINTS_PER_PACKET > 1:
                 angle_step = (end_angle_calc - start_angle_calc) / (LidarAPI.POINTS_PER_PACKET - 1)
            else:
                 angle_step = 0 # Or handle as appropriate

            # Assign angles to points
            for i, point in enumerate(points_data):
                point['angle'] = (start_angle_calc + i * angle_step) % 360.0

            return {
                'speed': speed,
                'start_angle': start_angle, # Original start angle
                'end_angle': end_angle,     # Original end angle
                'sensor_timestamp': sensor_timestamp_sec, # Timestamp from packet (seconds)
                'points': points_data       # List of {'distance', 'intensity', 'angle'}
            }

        except struct.error as e:
            raise LidarPacketError(f"Error unpacking packet data: {e}")
        except IndexError as e:
             raise LidarPacketError(f"Packet index out of bounds during parsing: {e}")
        except Exception as e:
             # Catch any other unexpected errors during parsing
             raise LidarPacketError(f"Unexpected error parsing packet: {e}")


# --- Example Usage ---
if __name__ == '__main__':
    # Example: Connect, get a few packets, then disconnect.
    # Replace with your actual LiDAR port and baudrate if different
    lidar_port = '/dev/ttyUSB0' # CHANGE AS NEEDED
    lidar_baud = 230400         # CHANGE AS NEEDED

    print(f"Starting LiDAR API example for {lidar_port}...")
    lidar = LidarAPI(port=lidar_port, baudrate=lidar_baud)

    try:
        lidar.connect()
        print("LiDAR connected. Waiting for packets...")

        start_time = time.time()
        packets_received = 0
        while time.time() - start_time < 10.0: # Run for 10 seconds
            latest = lidar.get_latest_packet()
            if latest:
                pi_ts_ns, packet_data = latest
                pi_ts_sec = pi_ts_ns / 1e9
                print(f"--- Latest Packet (Pi Time: {pi_ts_sec:.3f}) ---")
                print(f"  Sensor Timestamp: {packet_data.get('sensor_timestamp', 'N/A'):.3f} s")
                print(f"  Speed: {packet_data.get('speed', 'N/A'):.2f} deg/s")
                print(f"  Start Angle: {packet_data.get('start_angle', 'N/A'):.2f} deg")
                print(f"  End Angle: {packet_data.get('end_angle', 'N/A'):.2f} deg")
                points = packet_data.get('points', [])
                print(f"  Points: {len(points)}")
                if points:
                     # Print first point details as example
                     print(f"    First Point: Angle={points[0].get('angle', 'N/A'):.2f}, Dist={points[0].get('distance', 'N/A'):.3f}m, Intensity={points[0].get('intensity', 'N/A')}")
                packets_received += 1
                # Add a small delay to avoid spamming the console too fast
                time.sleep(0.1)
            else:
                # print("No new packet yet...")
                time.sleep(0.05) # Wait a bit longer if no packet

        print(f"\nFinished example loop. Received indications for {packets_received} packets.")

    except LidarCommunicationError as e:
        print(f"ERROR: Failed to connect or communicate with LiDAR: {e}")
    except KeyboardInterrupt:
        print("\nStopping example...")
    finally:
        print("Disconnecting LiDAR...")
        lidar.disconnect()
        print("LiDAR disconnected.")

