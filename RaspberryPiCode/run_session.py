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
from calibration import Calibration
import cv2

def init_db(db_path: str) -> sqlite3.Connection:
    conn = sqlite3.connect(db_path, check_same_thread=False)
    return conn

def write_run_metadata(conn: sqlite3.Connection, name: str, description: str, run_type: str) -> int:
    start_ns = time.time_ns()
    cur = conn.cursor()
    cur.execute(
        "INSERT INTO run_metadata (name, description, type, start_time_ns) VALUES (?, ?, ?, ?)",
        (name, description, run_type, start_ns)
    )
    conn.commit()
    return cur.lastrowid

def update_run_end(conn: sqlite3.Connection, run_id: int) -> None:
    end_ns = time.time_ns()
    conn.execute(
        "UPDATE run_metadata SET end_time_ns = ? WHERE id = ?",
        (end_ns, run_id)
    )
    conn.commit()

def write_calibration(conn: sqlite3.Connection, run_id: int, calib_data: dict):
    cur = conn.cursor()
    for step_name, samples in calib_data.items():
        start_ns = samples[0]['pi_timestamp_ns']
        end_ns = samples[-1]['pi_timestamp_ns']
        cur.execute(
            "INSERT INTO calibration_steps (run_id, step_name, instruction, start_time_ns, end_time_ns) VALUES (?, ?, ?, ?, ?)",
            (run_id, step_name, step_name, start_ns, end_ns)
        )
        step_id = cur.lastrowid

        for sample in samples:
            ts = sample['pi_timestamp_ns']
            if sample.get('imu'):
                imu = sample['imu']
                cur.execute(
                    "INSERT INTO calibration_imu_data (step_id, pi_timestamp_ns, arduino_timestamp_s, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z)"
                    " VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                    (step_id, ts, imu['t'], *imu['acc'], *imu['gyro'], *(imu.get('mag', [None, None, None])))
                )
            if sample.get('lidar'):
                lidar = sample['lidar']
                cur.execute(
                    "INSERT INTO calibration_lidar_data (step_id, pi_timestamp_ns, lidar_timestamp_s, speed, start_angle, end_angle)"
                    " VALUES (?, ?, ?, ?, ?, ?)",
                    (step_id, ts, lidar['sensor_timestamp'], lidar['speed'], lidar['start_angle'], lidar['end_angle'])
                )
                scan_id = cur.lastrowid
                for pt in lidar['points']:
                    cur.execute(
                        "INSERT INTO calibration_lidar_points (scan_id, angle, distance, intensity) VALUES (?, ?, ?, ?)",
                        (scan_id, pt['angle'], pt['distance'], pt['intensity'])
                    )
    conn.commit()

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

    conn = init_db(db_path)
    run_id = write_run_metadata(conn, args.name, args.desc, 'calibration')

    lidar = LidarAPI(port=args.lidar_port, baudrate=230400)
    imu = IMUAPI(port=args.imu_port, baudrate=115200)
    camera = CameraAPI(left_index=args.left_cam, right_index=args.right_cam, upside_down=True)

    calibration = Calibration(lidar=lidar, imu=imu, duration=5.0, countdown=3)
    calib_data = calibration.run()
    write_calibration(conn, run_id, calib_data)
    update_run_end(conn, run_id)

    scan_run_id = write_run_metadata(conn, args.name + ":scan", args.desc, 'scan')
    lidar.connect()
    imu.connect()
    camera.connect()

    imu_q = Queue()
    lidar_q = Queue()
    cam_q = Queue()
    stop_flag = [False]

    def imu_writer():
        while not stop_flag[0] or not imu_q.empty():
            try:
                pi_ts, imu_data = imu_q.get(timeout=0.1)
                conn.execute("""
                    INSERT INTO imu_data (run_id, pi_timestamp_ns, arduino_timestamp_s,
                        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (scan_run_id, pi_ts, imu_data['t'], *imu_data['acc'], *imu_data['gyro'], *(imu_data.get('mag', [None, None, None]))))
                conn.commit()
            except:
                continue

    def lidar_writer():
        while not stop_flag[0] or not lidar_q.empty():
            try:
                pi_ts, data = lidar_q.get(timeout=0.1)
                cur = conn.cursor()
                cur.execute("""
                    INSERT INTO lidar_data (run_id, pi_timestamp_ns, lidar_timestamp_s, speed, start_angle, end_angle)
                    VALUES (?, ?, ?, ?, ?, ?)
                """, (scan_run_id, pi_ts, data['sensor_timestamp'], data['speed'], data['start_angle'], data['end_angle']))
                scan_id = cur.lastrowid
                for pt in data['points']:
                    cur.execute("""
                        INSERT INTO lidar_points (scan_id, angle, distance, intensity)
                        VALUES (?, ?, ?, ?)
                    """, (scan_id, pt['angle'], pt['distance'], pt['intensity']))
                conn.commit()
            except:
                continue

    def camera_writer():
        os.makedirs(image_dir, exist_ok=True)
        while not stop_flag[0] or not cam_q.empty():
            try:
                pi_ts, left_img, right_img = cam_q.get(timeout=0.1)
                left_path = os.path.join(image_dir, f"{pi_ts}_left.jpg")
                right_path = os.path.join(image_dir, f"{pi_ts}_right.jpg")
                cv2.imwrite(left_path, left_img)
                cv2.imwrite(right_path, right_img)
                conn.execute("""
                    INSERT INTO stereo_images (run_id, pi_timestamp_ns, left_image_path, right_image_path)
                    VALUES (?, ?, ?, ?)
                """, (scan_run_id, pi_ts, left_path, right_path))
                conn.commit()
            except:
                continue

    print("Recording scan data. Press ENTER to stop.")
    threads = [
        threading.Thread(target=imu_writer),
        threading.Thread(target=lidar_writer),
        threading.Thread(target=camera_writer)
    ]
    for t in threads:
        t.start()

    try:
        while True:
            pi_ts = time.time_ns()
            if (pkt := lidar.get_latest_packet()):
                lidar_q.put((pi_ts, pkt[1]))
            if (pkt := imu.get_latest_packet()):
                imu_q.put((pi_ts, pkt))
            try:
                left_img, right_img = camera.capture()
                cam_q.put((pi_ts, left_img, right_img))
            except:
                pass
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass

    input()
    stop_flag[0] = True
    for t in threads:
        t.join()

    update_run_end(conn, scan_run_id)
    conn.close()
    print(f"Session '{args.name}' complete. Data saved to '{session_path}'")

if __name__ == '__main__':
    main()
