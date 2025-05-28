# run_session.py
# Perform calibration, then run continuous scan logging until user presses ENTER, saving each data type on its own thread

import sqlite3
import time
import os
import threading
from queue import Queue
from lidar_api import LidarAPI
from imu_api import IMUAPI
from camera_api import CameraAPI
import numpy as np
from calibration import Calibration
import cv2
import sys
import select

def init_db(db_path: str) -> sqlite3.Connection:
    conn = sqlite3.connect(db_path, check_same_thread=False)
    cursor = conn.cursor()
    cursor.executescript("""
    PRAGMA journal_mode=WAL;
    PRAGMA synchronous=NORMAL;

    CREATE TABLE IF NOT EXISTS run_metadata (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT,
        description TEXT,
        type TEXT CHECK(type IN ('calibration', 'scan')),
        start_time_ns BIGINT,
        end_time_ns BIGINT
    );

    CREATE TABLE IF NOT EXISTS calibration_steps (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        run_id INTEGER,
        step_name TEXT,
        instruction TEXT,
        start_time_ns BIGINT,
        end_time_ns BIGINT,
        FOREIGN KEY(run_id) REFERENCES run_metadata(id)
    );

    CREATE TABLE IF NOT EXISTS calibration_imu_data (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        step_id INTEGER,
        pi_timestamp_ns BIGINT NOT NULL,
        arduino_timestamp_s REAL NOT NULL,
        acc_x REAL, acc_y REAL, acc_z REAL,
        gyro_x REAL, gyro_y REAL, gyro_z REAL,
        mag_x REAL, mag_y REAL, mag_z REAL,
        FOREIGN KEY(step_id) REFERENCES calibration_steps(id)
    );

    CREATE TABLE IF NOT EXISTS calibration_lidar_data (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        step_id INTEGER,
        pi_timestamp_ns BIGINT NOT NULL,
        lidar_timestamp_s REAL,
        speed REAL,
        start_angle REAL,
        end_angle REAL,
        FOREIGN KEY(step_id) REFERENCES calibration_steps(id)
    );

    CREATE TABLE IF NOT EXISTS calibration_lidar_points (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        scan_id INTEGER,
        angle REAL,
        distance REAL,
        intensity INTEGER,
        FOREIGN KEY(scan_id) REFERENCES calibration_lidar_data(id)
    );

    CREATE TABLE IF NOT EXISTS imu_data (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        run_id INTEGER,
        pi_timestamp_ns BIGINT NOT NULL,
        arduino_timestamp_s REAL NOT NULL,
        acc_x REAL, acc_y REAL, acc_z REAL,
        gyro_x REAL, gyro_y REAL, gyro_z REAL,
        mag_x REAL, mag_y REAL, mag_z REAL,
        FOREIGN KEY(run_id) REFERENCES run_metadata(id)
    );

    CREATE TABLE IF NOT EXISTS lidar_data (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        run_id INTEGER,
        pi_timestamp_ns BIGINT NOT NULL,
        lidar_timestamp_s REAL,
        speed REAL,
        start_angle REAL,
        end_angle REAL,
        FOREIGN KEY(run_id) REFERENCES run_metadata(id)
    );

    CREATE TABLE IF NOT EXISTS lidar_points (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        scan_id INTEGER,
        angle REAL,
        distance REAL,
        intensity INTEGER,
        FOREIGN KEY(scan_id) REFERENCES lidar_data(id)
    );

    CREATE TABLE IF NOT EXISTS stereo_images (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        run_id INTEGER,
        pi_timestamp_ns BIGINT NOT NULL,
        left_image_path TEXT,
        right_image_path TEXT,
        FOREIGN KEY(run_id) REFERENCES run_metadata(id)
    );
    """)
    conn.commit()
    return conn

def write_run_metadata(conn, name, desc, run_type):
    cur = conn.cursor()
    cur.execute("INSERT INTO run_metadata (name, description, type, start_time_ns) VALUES (?, ?, ?, ?)",
                (name, desc, run_type, time.time_ns()))
    conn.commit()
    return cur.lastrowid

def update_run_end(conn, run_id):
    conn.execute("UPDATE run_metadata SET end_time_ns = ? WHERE id = ?", (time.time_ns(), run_id))
    conn.commit()

def write_calibration(conn, run_id, calib_data):
    cur = conn.cursor()
    for step_name, samples in calib_data.items():
        start_ns = samples[0]['pi_timestamp_ns']
        end_ns = samples[-1]['pi_timestamp_ns']
        cur.execute("INSERT INTO calibration_steps (run_id, step_name, instruction, start_time_ns, end_time_ns) VALUES (?, ?, ?, ?, ?)",
                    (run_id, step_name, step_name, start_ns, end_ns))
        step_id = cur.lastrowid
        for sample in samples:
            ts = sample['pi_timestamp_ns']
            if sample.get('imu'):
                imu = sample['imu']
                cur.execute("""
                    INSERT INTO calibration_imu_data (step_id, pi_timestamp_ns, arduino_timestamp_s,
                    acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (step_id, ts, imu['t'], *imu['acc'], *imu['gyro'], *(imu.get('mag', [None, None, None]))))
            if sample.get('lidar'):
                lidar = sample['lidar']
                cur.execute("""
                    INSERT INTO calibration_lidar_data (step_id, pi_timestamp_ns, lidar_timestamp_s, speed, start_angle, end_angle)
                    VALUES (?, ?, ?, ?, ?, ?)
                """, (step_id, ts, lidar['sensor_timestamp'], lidar['speed'], lidar['start_angle'], lidar['end_angle']))
                scan_id = cur.lastrowid
                for pt in lidar['points']:
                    cur.execute("INSERT INTO calibration_lidar_points (scan_id, angle, distance, intensity) VALUES (?, ?, ?, ?)",
                                (scan_id, pt['angle'], pt['distance'], pt['intensity']))
    conn.commit()

class ThreadSafeDBWriter:
    """Thread-safe database writer with connection pooling"""
    def __init__(self, db_path: str, run_id: int):
        self.db_path = db_path
        self.run_id = run_id
        self.local = threading.local()
        self.lock = threading.Lock()
        
    def get_connection(self):
        """Get a thread-local database connection"""
        if not hasattr(self.local, 'conn'):
            self.local.conn = sqlite3.connect(self.db_path)
            self.local.conn.execute("PRAGMA journal_mode=WAL")
            self.local.conn.execute("PRAGMA synchronous=NORMAL")
        return self.local.conn
    
    def write_imu(self, pi_ts: int, imu_data: dict):
        """Write IMU data atomically"""
        conn = self.get_connection()
        try:
            conn.execute("""
                INSERT INTO imu_data (run_id, pi_timestamp_ns, arduino_timestamp_s,
                    acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (self.run_id, pi_ts, imu_data['t'], 
                  *imu_data['acc'], *imu_data['gyro'], 
                  *(imu_data.get('mag', [None, None, None]))))
            conn.commit()
        except Exception as e:
            conn.rollback()
            print(f"Error writing IMU data: {e}")
    
    def write_lidar(self, pi_ts: int, data: dict):
        """Write LiDAR data and points atomically"""
        conn = self.get_connection()
        try:
            # Use a transaction to ensure atomicity
            with conn:
                cur = conn.cursor()
                cur.execute("""
                    INSERT INTO lidar_data (run_id, pi_timestamp_ns, lidar_timestamp_s, speed, start_angle, end_angle)
                    VALUES (?, ?, ?, ?, ?, ?)
                """, (self.run_id, pi_ts, data['sensor_timestamp'], 
                      data['speed'], data['start_angle'], data['end_angle']))
                
                scan_id = cur.lastrowid
                
                # Insert all points for this scan atomically
                point_data = [(scan_id, pt['angle'], pt['distance'], pt['intensity']) 
                             for pt in data['points']]
                cur.executemany(
                    "INSERT INTO lidar_points (scan_id, angle, distance, intensity) VALUES (?, ?, ?, ?)",
                    point_data
                )
        except Exception as e:
            print(f"Error writing LiDAR data: {e}")
    
    def write_stereo_image(self, pi_ts: int, left_path: str, right_path: str):
        """Write stereo image paths atomically"""
        conn = self.get_connection()
        try:
            conn.execute("""
                INSERT INTO stereo_images (run_id, pi_timestamp_ns, left_image_path, right_image_path) 
                VALUES (?, ?, ?, ?)
            """, (self.run_id, pi_ts, left_path, right_path))
            conn.commit()
        except Exception as e:
            conn.rollback()
            print(f"Error writing image data: {e}")
    
    def close_all(self):
        """Close the thread-local connection if it exists"""
        if hasattr(self.local, 'conn'):
            self.local.conn.close()

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--db', default='data.db')
    parser.add_argument('--name', required=False)
    parser.add_argument('--desc', default='')
    parser.add_argument('--lidar-port', default='/dev/ttyUSB0')
    parser.add_argument('--imu-port', default='/dev/ttyACM0')
    parser.add_argument('--left-cam', type=int, default=2)
    parser.add_argument('--right-cam', type=int, default=0)
    parser.add_argument('--path', default='/media/admin/Crucial X9/test_databases')
    args = parser.parse_args()

    if not args.name:
        args.name = input("Enter a session name (no spaces): ").strip()

    session_path = os.path.join(args.path, args.name)
    os.makedirs(session_path, exist_ok=True)
    db_path = os.path.join(session_path, args.db)
    image_dir = os.path.join(session_path, "images")

    # Initialize main connection for metadata
    conn = init_db(db_path)
    run_id = write_run_metadata(conn, args.name, args.desc, 'calibration')

    # Initialize sensors
    lidar = LidarAPI(port=args.lidar_port, baudrate=230400)
    imu = IMUAPI(port=args.imu_port, baudrate=115200)
    camera = CameraAPI(left_index=args.left_cam, right_index=args.right_cam, 
                      upside_down=True, exposure=-6, gain=10)

    # Run calibration
    calibration = Calibration(lidar=lidar, imu=imu, duration=5.0, countdown=3)
    calib_data = calibration.run()
    write_calibration(conn, run_id, calib_data)
    update_run_end(conn, run_id)

    # Start scan session
    scan_run_id = write_run_metadata(conn, args.name + ":scan", args.desc, 'scan')
    
    # Create thread-safe writer
    db_writer = ThreadSafeDBWriter(db_path, scan_run_id)
    
    # Connect sensors
    lidar.connect()
    imu.connect()
    camera.connect()

    # Create queues for data
    imu_q = Queue()
    lidar_q = Queue()
    cam_q = Queue()
    stop_flag = threading.Event()

    def imu_writer():
        """IMU writer thread"""
        while not stop_flag.is_set() or not imu_q.empty():
            try:
                pi_ts, imu_data = imu_q.get(timeout=0.1)
                db_writer.write_imu(pi_ts, imu_data)
                imu_q.task_done()
            except:
                continue
        db_writer.close_all()

    def lidar_writer():
        """LiDAR writer thread"""
        while not stop_flag.is_set() or not lidar_q.empty():
            try:
                pi_ts, data = lidar_q.get(timeout=0.1)
                db_writer.write_lidar(pi_ts, data)
                lidar_q.task_done()
            except:
                continue
        db_writer.close_all()

    def camera_writer():
        """Camera writer thread"""
        os.makedirs(image_dir, exist_ok=True)
        while not stop_flag.is_set() or not cam_q.empty():
            try:
                pi_ts, left_img, right_img = cam_q.get(timeout=0.1)
                left_path = os.path.join(image_dir, f"{pi_ts}_left.jpg")
                right_path = os.path.join(image_dir, f"{pi_ts}_right.jpg")
                cv2.imwrite(left_path, left_img)
                cv2.imwrite(right_path, right_img)
                db_writer.write_stereo_image(pi_ts, left_path, right_path)
                cam_q.task_done()
            except:
                continue
        db_writer.close_all()

    print("Recording scan data. Press ENTER to stop...")
    
    # Start writer threads
    threads = [
        threading.Thread(target=imu_writer, name="IMU-Writer"),
        threading.Thread(target=lidar_writer, name="LiDAR-Writer"),
        threading.Thread(target=camera_writer, name="Camera-Writer")
    ]
    for t in threads:
        t.start()

    # Data collection loop
    try:
        while not stop_flag.is_set():
            # Get LiDAR data with its original timestamp
            lidar_packet = lidar.get_latest_packet()
            if lidar_packet:
                pi_ts, lidar_data = lidar_packet
                lidar_q.put((pi_ts, lidar_data))
            
            # Get IMU data
            imu_data = imu.get_latest_packet()
            if imu_data:
                pi_ts = time.time_ns()
                imu_q.put((pi_ts, imu_data))
            
            # Get camera data
            try:
                left_img, right_img = camera.capture()
                pi_ts = time.time_ns()
                cam_q.put((pi_ts, left_img, right_img))
            except Exception as e:
                # Camera capture can fail occasionally
                pass
            
            # Check for user input (non-blocking)
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                _ = sys.stdin.readline()
                stop_flag.set()
            
            time.sleep(0.01)  # Small delay to prevent CPU spinning
            
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        stop_flag.set()

    # Wait for threads to finish
    print("Waiting for writers to finish...")
    for t in threads:
        t.join(timeout=5.0)
        if t.is_alive():
            print(f"Warning: {t.name} thread did not finish cleanly")

    # Finalize
    lidar.disconnect()
    imu.disconnect()
    camera.disconnect()
    
    update_run_end(conn, scan_run_id)
    conn.close()
    
    print(f"Session '{args.name}' complete. Data saved to '{session_path}'")

if __name__ == '__main__':
    main()