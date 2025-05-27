import serial
import json
import argparse
import sys

# Corrected argument parser
parser = argparse.ArgumentParser(description="Live IMU display")
parser.add_argument("--sensor", choices=["acc", "gyro", "mag"], required=True, help="Sensor to display (acc, gyro, or mag)")
parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port (default: /dev/ttyACM0)")
parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
args = parser.parse_args()

sensor_key = args.sensor.lower()

try:
    ser = serial.Serial(args.port, args.baud, timeout=1)
except serial.SerialException as e:
    print(f"Could not open serial port: {e}")
    sys.exit(1)

print(f"Reading {sensor_key.upper()} from {args.port}...")

while True:
    try:
        line = ser.readline().decode("utf-8").strip()
        if not line:
            continue
        data = json.loads(line)
        if sensor_key in data:
            x, y, z = data[sensor_key]
            output = f"{sensor_key.upper()}:  X={x:>7.4f}   Y={y:>7.4f}   Z={z:>7.4f}"
            print("\r" + output, end="", flush=True)
        else:
            print("\r" + f"{sensor_key.upper()} not available", end="", flush=True)
    except (json.JSONDecodeError, ValueError):
        continue
    except KeyboardInterrupt:
        print("\nExiting.")
        break
