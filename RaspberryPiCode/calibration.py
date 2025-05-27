from lidar_api import LidarAPI
from imu_api import IMUAPI
import time
import json
import pandas as pd
from typing import List, Dict, Any, Optional

class Calibration:
    def __init__(self, lidar: LidarAPI, imu: IMUAPI, duration: float = 5.0, countdown: int = 3):
        """
        Initialize the Calibration sequence.

        Args:
            lidar: An instance of LidarAPI.
            imu: An instance of IMUAPI.
            duration: Duration of data collection per step (in seconds).
            countdown: Seconds to wait before each step to allow user to prepare.
        """
        self.lidar = lidar
        self.imu = imu
        self.duration = duration
        self.countdown = countdown

    def _countdown(self, message: str, wait_seconds: Optional[int] = None) -> None:
        print(f"\n{message}")
        count = wait_seconds if wait_seconds is not None else self.countdown
        for i in reversed(range(1, count + 1)):
            print(f"  Starting in {i} seconds...", end='\r', flush=True)
            time.sleep(1)
        print("  Collecting Calibration Data...\n")

    def _collect_joint_data(self) -> List[Dict[str, Any]]:
        collected = []
        start_time = time.time()
        while time.time() - start_time < self.duration:
            pi_time_ns = time.time_ns()
            lidar_data = self.lidar.get_latest_packet()
            imu_data = self.imu.get_latest_packet()

            # Only store if at least one data source is valid
            if lidar_data or imu_data:
                sample = {
                    "pi_timestamp_ns": pi_time_ns,
                    "lidar": lidar_data[1] if lidar_data else None,
                    "imu": imu_data
                }
                collected.append(sample)

            time.sleep(0.005)   # sampling delay, should be 
                                # slightly less than the shortest expected packet interval

        print(f"  Collected {len(collected)} joint data points.")
        return collected

    def _collect_imu_only(self) -> List[Dict[str, Any]]:
        collected = []
        start_time = time.time()
        while time.time() - start_time < self.duration:
            pi_time_ns = time.time_ns()
            imu_data = self.imu.get_latest_packet()

            if imu_data:
                collected.append({
                    "pi_timestamp_ns": pi_time_ns,
                    "imu": imu_data
                })

            time.sleep(0.005)

        print(f"  Collected {len(collected)} IMU-only data points.")
        return collected

    def run(self) -> pd.DataFrame:
        print("=== Starting Calibration Sequence ===")
        self.lidar.connect()
        self.imu.connect()

        try:
            # Step 1: Collect joint sync data
            self._countdown("Step 1: Collecting synchronized LiDAR + IMU data")
            sync_data = self._collect_joint_data()

            # Step 2: Set down device
            self._countdown("Step 2: Please place the device firmly on a surface", wait_seconds=20)
            still_data = self._collect_imu_only()

            # Step 3: Rotate device
            self._countdown("Step 3: Rotate the device in all directions", wait_seconds=self.countdown)
            rot_data = self._collect_imu_only()

        finally:
            self.lidar.disconnect()
            self.imu.disconnect()
            print("Calibration complete.")

        # Combine into a labeled dataset
        df_sync = pd.DataFrame(sync_data)
        df_still = pd.DataFrame(still_data)
        df_still["label"] = "still_IMU"
        df_rot = pd.DataFrame(rot_data)
        df_rot["label"] = "rot_IMU"

        return {
            "sync": sync_data,
            "still_IMU": still_data,
            "rot_IMU": rot_data
        }

