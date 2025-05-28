# Perform calibration, then record continuous scan data.
import os, sys, time, threading, select, sqlite3, cv2, argparse
from queue import Queue
from lidar_api import LidarAPI
from imu_api import IMUAPI
from camera_api import CameraAPI
from calibration import Calibration

# ───────────────────────────────────────────────────────────── database setup ──
def init_db(db_path: str) -> sqlite3.Connection:
    conn = sqlite3.connect(db_path, check_same_thread=False)
    cur = conn.cursor()
    cur.executescript("""
    PRAGMA journal_mode=WAL;
    CREATE TABLE IF NOT EXISTS run_metadata (
        id INTEGER PRIMARY KEY, name TEXT, description TEXT,
        type TEXT CHECK(type IN ('calibration','scan')),
        start_time_ns BIGINT, end_time_ns BIGINT
    );
    CREATE TABLE IF NOT EXISTS calibration_steps (
        id INTEGER PRIMARY KEY, run_id INTEGER, step_name TEXT, instruction TEXT,
        start_time_ns BIGINT, end_time_ns BIGINT, FOREIGN KEY(run_id) REFERENCES run_metadata(id)
    );
    CREATE TABLE IF NOT EXISTS calibration_imu_data (
        id INTEGER PRIMARY KEY, step_id INTEGER, pi_timestamp_ns BIGINT,
        arduino_timestamp_s REAL,
        acc_x REAL, acc_y REAL, acc_z REAL,
        gyro_x REAL, gyro_y REAL, gyro_z REAL,
        mag_x REAL, mag_y REAL, mag_z REAL,
        FOREIGN KEY(step_id) REFERENCES calibration_steps(id)
    );
    CREATE TABLE IF NOT EXISTS calibration_lidar_data (
        id INTEGER PRIMARY KEY, step_id INTEGER, pi_timestamp_ns BIGINT,
        lidar_timestamp_s REAL, speed REAL, start_angle REAL, end_angle REAL,
        FOREIGN KEY(step_id) REFERENCES calibration_steps(id)
    );
    CREATE TABLE IF NOT EXISTS calibration_lidar_points (
        id INTEGER PRIMARY KEY, scan_id INTEGER,
        angle REAL, distance REAL, intensity INTEGER,
        FOREIGN KEY(scan_id) REFERENCES calibration_lidar_data(id)
    );
    CREATE TABLE IF NOT EXISTS imu_data (
        id INTEGER PRIMARY KEY, run_id INTEGER, pi_timestamp_ns BIGINT,
        arduino_timestamp_s REAL,
        acc_x REAL, acc_y REAL, acc_z REAL,
        gyro_x REAL, gyro_y REAL, gyro_z REAL,
        mag_x REAL, mag_y REAL, mag_z REAL,
        FOREIGN KEY(run_id) REFERENCES run_metadata(id)
    );
    CREATE TABLE IF NOT EXISTS lidar_data (
        id INTEGER PRIMARY KEY, run_id INTEGER, pi_timestamp_ns BIGINT,
        lidar_timestamp_s REAL, speed REAL, start_angle REAL, end_angle REAL,
        FOREIGN KEY(run_id) REFERENCES run_metadata(id)
    );
    CREATE UNIQUE INDEX IF NOT EXISTS uniq_scan
        ON lidar_data(run_id, lidar_timestamp_s);
    CREATE TABLE IF NOT EXISTS lidar_points (
        id INTEGER PRIMARY KEY, scan_id INTEGER,
        angle REAL, distance REAL, intensity INTEGER,
        FOREIGN KEY(scan_id) REFERENCES lidar_data(id)
    );
    CREATE TABLE IF NOT EXISTS stereo_images (
        id INTEGER PRIMARY KEY, run_id INTEGER, pi_timestamp_ns BIGINT,
        left_image_path TEXT, right_image_path TEXT,
        FOREIGN KEY(run_id) REFERENCES run_metadata(id)
    );
    """)
    conn.commit()
    return conn


def write_run_metadata(conn, name, desc, run_type):
    cur = conn.cursor()
    cur.execute("""INSERT INTO run_metadata
                   (name, description, type, start_time_ns)
                   VALUES (?,?,?,?)""",
                (name, desc, run_type, time.time_ns()))
    conn.commit()
    return cur.lastrowid


def update_run_end(conn, run_id):
    conn.execute("UPDATE run_metadata SET end_time_ns=? WHERE id=?",
                 (time.time_ns(), run_id))
    conn.commit()


# ──────────────────────────────────────────────────────────────── main entry ──
def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--db', default='data.db')
    ap.add_argument('--name')
    ap.add_argument('--desc', default='')
    ap.add_argument('--lidar-port', default='/dev/ttyUSB0')
    ap.add_argument('--imu-port', default='/dev/ttyACM0')
    ap.add_argument('--left-cam', type=int, default=2)
    ap.add_argument('--right-cam', type=int, default=0)
    ap.add_argument('--path', default='/media/admin/Crucial X9/test_databases')
    args = ap.parse_args()

    if not args.name:
        args.name = input("Enter a session name (no spaces): ").strip()

    session_path = os.path.join(args.path, args.name)
    os.makedirs(session_path, exist_ok=True)
    db_path = os.path.join(session_path, args.db)
    image_dir = os.path.join(session_path, "images")

    # one master DB handle for metadata; each writer gets its own
    master_conn = init_db(db_path)

    # ─────────── calibration run ───────────
    run_id = write_run_metadata(master_conn, args.name, args.desc, 'calibration')

    lidar = LidarAPI(args.lidar_port, 230400)
    imu = IMUAPI(args.imu_port, 115200)
    camera = CameraAPI(args.left_cam, args.right_cam,
                       upside_down=True, exposure=-6, gain=10)

    calibration = Calibration(lidar, imu, duration=5.0, countdown=3)
    calib_data = calibration.run()

    # store calibration results (unchanged helper from earlier version)
    from typing import Dict, List, Any
    def write_calib(conn, run_id, data: Dict[str, List[Dict[str, Any]]]):
        cur = conn.cursor()
        for step_name, samples in data.items():
            start_ns, end_ns = samples[0]['pi_timestamp_ns'], samples[-1]['pi_timestamp_ns']
            cur.execute("""INSERT INTO calibration_steps
                           (run_id, step_name, instruction, start_time_ns, end_time_ns)
                           VALUES (?,?,?,?,?)""",
                        (run_id, step_name, step_name, start_ns, end_ns))
            step_id = cur.lastrowid
            for s in samples:
                ts = s['pi_timestamp_ns']
                if (imu_pkt := s.get('imu')):
                    cur.execute("""INSERT INTO calibration_imu_data
                                   (step_id, pi_timestamp_ns, arduino_timestamp_s,
                                    acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z,
                                    mag_x, mag_y, mag_z)
                                   VALUES (?,?,?,?,?,?,?,?,?,?,?,?)""",
                                (step_id, ts, imu_pkt['t'], *imu_pkt['acc'],
                                 *imu_pkt['gyro'], *(imu_pkt.get('mag', [None]*3))))
                if (lidar_pkt := s.get('lidar')):
                    cur.execute("""INSERT INTO calibration_lidar_data
                                   (step_id, pi_timestamp_ns, lidar_timestamp_s,
                                    speed, start_angle, end_angle)
                                   VALUES (?,?,?,?,?,?)""",
                                (step_id, ts, lidar_pkt['sensor_timestamp'],
                                 lidar_pkt['speed'], lidar_pkt['start_angle'],
                                 lidar_pkt['end_angle']))
                    scan_id = cur.lastrowid
                    cur.executemany(
                        "INSERT INTO calibration_lidar_points "
                        "(scan_id, angle, distance, intensity) VALUES (?,?,?,?)",
                        [(scan_id, p['angle'], p['distance'], p['intensity'])
                         for p in lidar_pkt['points']]
                    )
        conn.commit()

    write_calib(master_conn, run_id, calib_data)
    update_run_end(master_conn, run_id)

    # ─────────── continuous scan run ───────────
    scan_run_id = write_run_metadata(master_conn, f"{args.name}:scan", args.desc, 'scan')

    lidar.connect(); imu.connect(); camera.connect()

    # individual DB handles per writer
    lidar_conn = init_db(db_path)
    imu_conn   = init_db(db_path)
    cam_conn   = init_db(db_path)

    imu_q, lidar_q, cam_q = Queue(), Queue(), Queue()
    stop_flag = threading.Event()

    # ───────── threads ─────────
    def imu_writer():
        while not (stop_flag.is_set() and imu_q.empty()):
            try:
                pi_ts, imu_pkt = imu_q.get(timeout=0.1)
                imu_conn.execute("""INSERT INTO imu_data
                                    (run_id, pi_timestamp_ns, arduino_timestamp_s,
                                     acc_x, acc_y, acc_z,
                                     gyro_x, gyro_y, gyro_z,
                                     mag_x, mag_y, mag_z)
                                    VALUES (?,?,?,?,?,?,?,?,?,?,?,?)""",
                                 (scan_run_id, pi_ts, imu_pkt['t'],
                                  *imu_pkt['acc'], *imu_pkt['gyro'],
                                  *(imu_pkt.get('mag', [None]*3))))
                imu_conn.commit()
            except Exception:
                continue

    def lidar_writer():
        while not (stop_flag.is_set() and lidar_q.empty()):
            try:
                pi_ts, pkt = lidar_q.get(timeout=0.1)
                cur = lidar_conn.cursor()
                cur.execute("""INSERT INTO lidar_data
                               (run_id, pi_timestamp_ns, lidar_timestamp_s,
                                speed, start_angle, end_angle)
                               VALUES (?,?,?,?,?,?)""",
                            (scan_run_id, pi_ts, pkt['sensor_timestamp'],
                             pkt['speed'], pkt['start_angle'], pkt['end_angle']))
                scan_id = cur.lastrowid
                cur.executemany(
                    "INSERT INTO lidar_points "
                    "(scan_id, angle, distance, intensity) VALUES (?,?,?,?)",
                    [(scan_id, p['angle'], p['distance'], p['intensity'])
                     for p in pkt['points']]
                )
                lidar_conn.commit()
            except sqlite3.IntegrityError:
                # duplicate packet (guard index); just drop it
                lidar_conn.rollback()
            except Exception:
                continue

    def camera_writer():
        os.makedirs(image_dir, exist_ok=True)
        while not (stop_flag.is_set() and cam_q.empty()):
            try:
                pi_ts, l_img, r_img = cam_q.get(timeout=0.1)
                l_path = os.path.join(image_dir, f"{pi_ts}_left.jpg")
                r_path = os.path.join(image_dir, f"{pi_ts}_right.jpg")
                cv2.imwrite(l_path, l_img); cv2.imwrite(r_path, r_img)
                cam_conn.execute("""INSERT INTO stereo_images
                                    (run_id, pi_timestamp_ns,
                                     left_image_path, right_image_path)
                                    VALUES (?,?,?,?)""",
                                 (scan_run_id, pi_ts, l_path, r_path))
                cam_conn.commit()
            except Exception:
                continue

    threads = [threading.Thread(target=f) for f in
               (imu_writer, lidar_writer, camera_writer)]
    for t in threads:
        t.start()

    print("Recording...  Press ENTER or Ctrl-C to stop.")
    last_lidar_stamp = None
    try:
        while True:
            pi_ts = time.time_ns()

            pkt = lidar.get_latest_packet()
            if pkt:
                _, data = pkt
                if data['sensor_timestamp'] != last_lidar_stamp:
                    lidar_q.put((pi_ts, data))
                    last_lidar_stamp = data['sensor_timestamp']

            if (imu_pkt := imu.get_latest_packet()):
                imu_q.put((pi_ts, imu_pkt))

            try:
                l_img, r_img = camera.capture()
                cam_q.put((pi_ts, l_img, r_img))
            except Exception:
                pass

            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                break
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        stop_flag.set()
        for t in threads:
            t.join()

        update_run_end(master_conn, scan_run_id)
        master_conn.close()
        lidar.disconnect(); imu.disconnect(); camera.disconnect()
        lidar_conn.close(); imu_conn.close(); cam_conn.close()

        print(f"Session '{args.name}' complete.  Data saved in '{session_path}'")

if __name__ == '__main__':
    main()
