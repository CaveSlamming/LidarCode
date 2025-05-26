from lidar_api import LidarAPI  # Change to your file name
import time

def print_packet(packet_info):
    pi_timestamp_ns, data = packet_info
    pi_time_s = pi_timestamp_ns / 1e9

    print(f"\n=== New LiDAR Packet ===")
    print(f"  Pi Timestamp     : {pi_time_s:.6f} s")
    print(f"  Sensor Timestamp : {data['sensor_timestamp']:.3f} s")
    print(f"  Speed            : {data['speed']:.2f} deg/s")
    print(f"  Start Angle      : {data['start_angle']:.2f}°")
    print(f"  End Angle        : {data['end_angle']:.2f}°")
    print(f"  Points           : {len(data['points'])}")

    for i, pt in enumerate(data['points'][:5]):  # Only show first 5 points
        print(f"    Point {i+1:2d}: "
              f"Angle = {pt['angle']:.2f}°, "
              f"Distance = {pt['distance']:.3f} m, "
              f"Intensity = {pt['intensity']}")
    if len(data['points']) > 5:
        print(f"    ... {len(data['points']) - 5} more points not shown")

# ---- Main Usage ----
if __name__ == '__main__':
    lidar = LidarAPI(port='/dev/ttyUSB0', baudrate=230400)  # Adjust if needed

    try:
        print("Connecting to LiDAR...")
        lidar.connect()
        print("Connected. Collecting data for 10 seconds...")

        start_time = time.time()
        while time.time() - start_time < 10.0:
            packet = lidar.get_latest_packet()
            if packet:
                print_packet(packet)
            time.sleep(0.1)  # Avoid printing too fast

    except Exception as e:
        print(f"ERROR: {e}")

    finally:
        print("Disconnecting...")
        lidar.disconnect()
        print("Done.")
