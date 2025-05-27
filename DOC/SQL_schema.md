## Sensor Calibration & Scan Data SQL Schema

This schema supports recording high-resolution calibration and scan data from a Raspberry Pi system interfacing with IMU, LiDAR, and stereo cameras. Designed for multithreaded inserts and post-run synchronization, analysis, or visualization.

Calibration data is kept **completely separate** from scan data through dedicated calibration tables. Each calibration step (e.g. "still\_IMU") is stored as its own unit, ensuring clarity, modularity, and no confusion with general sensor logs.

---

### Table: `run_metadata`

Stores metadata for each recording session.

```sql
CREATE TABLE run_metadata (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT,
    description TEXT,
    type TEXT CHECK(type IN ('calibration', 'scan')),
    start_time_ns BIGINT,
    end_time_ns BIGINT
);
```

---

## Calibration Tables (Isolated Data Structure)

### Table: `calibration_steps`

Each logical calibration segment with a label and timing.

```sql
CREATE TABLE calibration_steps (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id INTEGER,
    step_name TEXT,           -- e.g., 'still_IMU', 'rot_IMU'
    instruction TEXT,         -- Human-readable instruction
    start_time_ns BIGINT,
    end_time_ns BIGINT,
    FOREIGN KEY(run_id) REFERENCES run_metadata(id)
);
```

### Table: `calibration_imu_data`

IMU readings for a calibration step.

```sql
CREATE TABLE calibration_imu_data (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    step_id INTEGER,
    pi_timestamp_ns BIGINT NOT NULL,
    arduino_timestamp_s REAL NOT NULL,
    acc_x REAL, acc_y REAL, acc_z REAL,
    gyro_x REAL, gyro_y REAL, gyro_z REAL,
    mag_x REAL, mag_y REAL, mag_z REAL,
    FOREIGN KEY(step_id) REFERENCES calibration_steps(id)
);
```

### Table: `calibration_lidar_data`

LiDAR scan-level data for calibration steps.

```sql
CREATE TABLE calibration_lidar_data (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    step_id INTEGER,
    pi_timestamp_ns BIGINT NOT NULL,
    lidar_timestamp_s REAL,
    speed REAL,
    start_angle REAL,
    end_angle REAL,
    FOREIGN KEY(step_id) REFERENCES calibration_steps(id)
);
```

### Table: `calibration_lidar_points`

Points tied to a calibration lidar scan.

```sql
CREATE TABLE calibration_lidar_points (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    scan_id INTEGER,
    angle REAL,
    distance REAL,
    intensity INTEGER,
    FOREIGN KEY(scan_id) REFERENCES calibration_lidar_data(id)
);
```

---

## Scan Tables (Active Sensor Logging)

### Table: `imu_data`

Live IMU readings during scans.

```sql
CREATE TABLE imu_data (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id INTEGER,
    pi_timestamp_ns BIGINT NOT NULL,
    arduino_timestamp_s REAL NOT NULL,
    acc_x REAL, acc_y REAL, acc_z REAL,
    gyro_x REAL, gyro_y REAL, gyro_z REAL,
    mag_x REAL, mag_y REAL, mag_z REAL,
    FOREIGN KEY(run_id) REFERENCES run_metadata(id)
);
```

### Table: `lidar_data`

Live LiDAR scan metadata.

```sql
CREATE TABLE lidar_data (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id INTEGER,
    pi_timestamp_ns BIGINT NOT NULL,
    lidar_timestamp_s REAL,
    speed REAL,
    start_angle REAL,
    end_angle REAL,
    FOREIGN KEY(run_id) REFERENCES run_metadata(id)
);
```

### Table: `lidar_points`

Points tied to LiDAR scans.

```sql
CREATE TABLE lidar_points (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    scan_id INTEGER,
    angle REAL,
    distance REAL,
    intensity INTEGER,
    FOREIGN KEY(scan_id) REFERENCES lidar_data(id)
);
```

### Table: `stereo_images`

Stereo image paths and timestamps.

```sql
CREATE TABLE stereo_images (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id INTEGER,
    pi_timestamp_ns BIGINT NOT NULL,
    left_image_path TEXT,
    right_image_path TEXT,
    FOREIGN KEY(run_id) REFERENCES run_metadata(id)
);
```

---

## Notes

* All sensor tables reference `run_metadata.id` for grouping.
* Calibration tables are completely isolated via `calibration_steps`.
* Index `pi_timestamp_ns` for fast cross-sensor time alignment.
* SQLite: enable `PRAGMA journal_mode=WAL` for concurrent writes.

---

